/*
 * SPDX-License-Identifier: MIT
 *
 * USB vendor control channel v1. See headers/controlchannel.h.
 *
 * Wire format (all multi-byte fields little-endian):
 *   bmRequestType 0xC0 (IN) / 0x40 (OUT), bRequest 0x47 ('G'),
 *   wIndex 0x50nn (0x50 = 'P' magic, nn = command), wLength <= 64.
 *   OUT payloads start with an 8-byte header: u32 token, u16 seq, u16 0.
 *
 * Runs on core0 from tud_task(). IN replies are built in the SETUP stage;
 * OUT commands are validated and applied in the DATA stage. Nothing here
 * writes flash or touches GPIO: commands only change RAM state and raise the
 * same events the hotkeys and web-config raise.
 * SET_LED is the exception: it is queued here and applied on core1.
 */

#include "controlchannel.h"

#include <string.h>

#include "pico/critical_section.h"
#include "pico/time.h"
#include "pico/unique_id.h"

#include "BoardConfig.h"
#include "animationstation.h"
#include "config.pb.h"
#include "enums.pb.h"
#include "drivermanager.h"
#include "eventmanager.h"
#include "storagemanager.h"
#include "system.h"
#include "version.h"

namespace {

constexpr uint8_t REQ_TYPE_IN  = 0xC0; // device-to-host, vendor, device
constexpr uint8_t REQ_TYPE_OUT = 0x40; // host-to-device, vendor, device
constexpr uint8_t REQ_CODE     = 0x47; // 'G'
constexpr uint8_t INDEX_MAGIC  = 0x50; // 'P'

constexpr uint8_t PROTO_MAJOR = 1;
constexpr uint8_t PROTO_MINOR = 1;

constexpr uint16_t MAX_LEN    = 64;
constexpr uint16_t HEADER_LEN = 8;
constexpr uint16_t MAX_STRING = 63;

enum Command : uint8_t {
    CMD_HELLO          = 0x01,
    CMD_STATUS         = 0x02,
    CMD_STRING         = 0x03,
    CMD_LED_STATUS     = 0x04,
    CMD_SET_PROFILE    = 0x10,
    CMD_REBOOT         = 0x11,
    CMD_SET_INPUT_MODE = 0x12,
    CMD_SET_LED        = 0x13,
};

enum Result : uint8_t {
    RES_OK               = 0,
    RES_BAD_TOKEN        = 1,
    RES_BAD_LENGTH       = 2,
    RES_BAD_ARG          = 3,
    RES_LOCKED           = 4,
    RES_UNSUPPORTED      = 5,
    RES_PROFILE_DISABLED = 6,
    RES_BUSY             = 7,
};

enum Capability : uint32_t {
    CAP_STATUS         = 1u << 0,
    CAP_STRING         = 1u << 1,
    CAP_SET_PROFILE    = 1u << 2,
    CAP_REBOOT         = 1u << 3,
    CAP_REBOOT_BOOTSEL = 1u << 4, // never set: BOOTSEL is not offered over the channel
    CAP_SET_INPUT_MODE = 1u << 5,
    CAP_LEDS           = 1u << 8,
};

constexpr uint32_t CAPABILITIES = CAP_STATUS | CAP_STRING | CAP_SET_PROFILE | CAP_REBOOT | CAP_SET_INPUT_MODE;

// InputMode values 0-31 that DriverManager::setup() can start in this build.
constexpr uint32_t BOOTABLE_MODES =
    (1u << INPUT_MODE_XINPUT) | (1u << INPUT_MODE_SWITCH) | (1u << INPUT_MODE_PS3) |
    (1u << INPUT_MODE_KEYBOARD) | (1u << INPUT_MODE_PS4) | (1u << INPUT_MODE_XBONE) |
    (1u << INPUT_MODE_MDMINI) | (1u << INPUT_MODE_NEOGEO) | (1u << INPUT_MODE_PCEMINI) |
    (1u << INPUT_MODE_EGRET) | (1u << INPUT_MODE_ASTRO) | (1u << INPUT_MODE_PSCLASSIC) |
    (1u << INPUT_MODE_XBOXORIGINAL) | (1u << INPUT_MODE_PS5) | (1u << INPUT_MODE_GENERIC) |
    (1u << INPUT_MODE_SWITCH_PRO);

enum StatusFlag : uint8_t {
    FLAG_PROFILE_PENDING = 1u << 0,
    FLAG_REBOOT_PENDING  = 1u << 1,
};

enum LedStatusFlag : uint8_t {
    LED_FLAG_RUNNING      = 1u << 0,
    LED_FLAG_CUSTOM_THEME = 1u << 1,
    LED_FLAG_UNSAVED      = 1u << 2,
};

enum LedMask : uint8_t {
    LED_ANIMATION    = 1u << 0,
    LED_BRIGHTNESS   = 1u << 1,
    LED_STATIC_COLOR = 1u << 2,
    LED_BUTTON_COLOR = 1u << 3,
    LED_THEME        = 1u << 4,
    LED_MASK_ALL     = 0x1f,
};

// AddTheme() calls in StaticTheme::StaticTheme()
constexpr uint8_t STATIC_THEME_COUNT = 20;

const char *const EFFECT_NAMES[] = {
    "EFFECT_STATIC_COLOR", "EFFECT_RAINBOW", "EFFECT_CHASE", "EFFECT_STATIC_THEME", "EFFECT_CUSTOM_THEME",
};
constexpr uint16_t STRING_EFFECT_BASE = 0x20;

struct LedValues {
    uint8_t animation, brightness, staticColor, buttonColor, theme;
};

// tud_control_xfer() keeps the buffer pointer until the transfer completes.
uint8_t buffer[MAX_LEN];

uint32_t sessionToken = 0;
bool rebootPending = false;
uint16_t lastSeq = 0;
uint8_t lastCmd = 0;
uint8_t lastResult = RES_OK;

// Shared with core1; ledLock guards the pending request and ledSaved.
volatile bool ledsRunning = false;
critical_section_t ledLock;
volatile bool ledPendingValid = false;
uint8_t ledPendingMask = 0;
bool ledPendingPersist = false;
LedValues ledPending = {};
LedValues ledSaved = {};

inline void put16(uint8_t *p, uint16_t v) { p[0] = v & 0xff; p[1] = v >> 8; }
inline void put32(uint8_t *p, uint32_t v) { put16(p, v & 0xffff); put16(p + 2, v >> 16); }
inline uint16_t get16(const uint8_t *p) { return p[0] | (p[1] << 8); }
inline uint32_t get32(const uint8_t *p) { return get16(p) | ((uint32_t)get16(p + 2) << 16); }

inline uint8_t commandOf(tusb_control_request_t const *request) { return request->wIndex & 0xff; }

// Direction of a known command: TUSB_DIR_IN / TUSB_DIR_OUT, or -1 if unknown.
int commandDirection(uint8_t cmd) {
    switch (cmd) {
        case CMD_HELLO:
        case CMD_STATUS:
        case CMD_STRING:
        case CMD_LED_STATUS:
            return TUSB_DIR_IN;
        case CMD_SET_PROFILE:
        case CMD_REBOOT:
        case CMD_SET_INPUT_MODE:
        case CMD_SET_LED:
            return TUSB_DIR_OUT;
        default:
            return -1;
    }
}

uint8_t effectCount() {
    return TOTAL_EFFECTS + (Storage::getInstance().getAnimationOptions().hasCustomTheme ? 1 : 0);
}

LedValues currentLeds() {
    const AnimationOptions &o = Storage::getInstance().getAnimationOptions();
    return LedValues{(uint8_t)o.baseAnimationIndex, (uint8_t)o.brightness, (uint8_t)o.staticColorIndex,
                     (uint8_t)o.buttonColorIndex, (uint8_t)o.themeIndex};
}

void overlay(LedValues &v, uint8_t mask, const LedValues &with) {
    if (mask & LED_ANIMATION)    v.animation   = with.animation;
    if (mask & LED_BRIGHTNESS)   v.brightness  = with.brightness;
    if (mask & LED_STATIC_COLOR) v.staticColor = with.staticColor;
    if (mask & LED_BUTTON_COLOR) v.buttonColor = with.buttonColor;
    if (mask & LED_THEME)        v.theme       = with.theme;
}

void handleStorageSave() {
    LedValues now = currentLeds();
    critical_section_enter_blocking(&ledLock);
    ledSaved = now;
    critical_section_exit(&ledLock);
}

uint32_t makeToken() {
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    uint32_t h = 2166136261u; // FNV-1a over the board id
    for (uint8_t b : id.id) { h ^= b; h *= 16777619u; }
    h ^= time_us_32();
    h ^= h >> 16; h *= 0x85ebca6bu; // murmur3 finaliser
    h ^= h >> 13; h *= 0xc2b2ae35u;
    h ^= h >> 16;
    return h ? h : 1;
}

bool forcedLock(bool webConfig) {
    ForcedSetupMode mode = Storage::getInstance().getForcedSetupOptions().mode;
    if (mode == FORCED_SETUP_MODE_LOCK_BOTH) return true;
    return webConfig ? mode == FORCED_SETUP_MODE_LOCK_WEB_CONFIG
                     : mode == FORCED_SETUP_MODE_LOCK_MODE_SWITCH;
}

uint16_t buildHello(uint16_t nonce) {
    if (sessionToken == 0) sessionToken = makeToken();
    memcpy(buffer, "GPii", 4);
    put16(buffer + 4, nonce);
    buffer[6] = PROTO_MAJOR;
    buffer[7] = PROTO_MINOR;
    put32(buffer + 8, CAPABILITIES | (ledsRunning ? CAP_LEDS : 0));
    put32(buffer + 12, sessionToken);
    put32(buffer + 16, BOOTABLE_MODES);
    return 20;
}

uint16_t buildStatus() {
    Storage &storage = Storage::getInstance();
    const ProfileOptions &profiles = storage.getProfileOptions();
    const uint32_t profileNumber = storage.getGamepadOptions().profileNumber;

    uint8_t enabled = 0x01; // profile 1 is always enabled
    for (pb_size_t i = 0; i < profiles.gpioMappingsSets_count && i < 7; i++) {
        if (profiles.gpioMappingsSets[i].enabled) enabled |= 1u << (i + 1);
    }

    uint8_t flags = 0;
    Gamepad *gamepad = storage.GetGamepad();
    if (gamepad != nullptr && gamepad->lastReinitProfileNumber != profileNumber) flags |= FLAG_PROFILE_PENDING;
    if (rebootPending) flags |= FLAG_REBOOT_PENDING;

    memset(buffer, 0, 16);
    buffer[0] = PROTO_MAJOR;
    buffer[1] = (uint8_t)DriverManager::getInstance().getInputMode();
    buffer[2] = (uint8_t)profileNumber;
    buffer[3] = enabled;
    buffer[4] = (uint8_t)storage.getForcedSetupOptions().mode;
    buffer[5] = flags;
    put16(buffer + 6, lastSeq);
    buffer[8] = lastCmd;
    buffer[9] = lastResult;
    put32(buffer + 12, to_ms_since_boot(get_absolute_time()));
    return 16;
}

uint16_t buildLedStatus() {
    Storage &storage = Storage::getInstance();
    const AnimationOptions &animation = storage.getAnimationOptions();
    const LEDOptions &led = storage.getLedOptions();

    LedValues v = currentLeds();
    critical_section_enter_blocking(&ledLock);
    if (ledPendingValid) overlay(v, ledPendingMask, ledPending);
    const LedValues saved = ledSaved;
    critical_section_exit(&ledLock);

    uint8_t flags = LED_FLAG_RUNNING;
    if (animation.hasCustomTheme) flags |= LED_FLAG_CUSTOM_THEME;
    if (memcmp(&v, &saved, sizeof v) != 0) flags |= LED_FLAG_UNSAVED;

    memset(buffer, 0, 12);
    buffer[0] = flags;
    buffer[1] = v.animation;
    buffer[2] = effectCount();
    buffer[3] = v.brightness;
    buffer[4] = (uint8_t)(led.brightnessSteps > 255 ? 255 : led.brightnessSteps);
    buffer[5] = (uint8_t)(led.brightnessMaximum > 255 ? 255 : led.brightnessMaximum);
    buffer[6] = v.staticColor;
    buffer[7] = v.buttonColor;
    buffer[8] = v.theme;
    buffer[9] = (uint8_t)colors.size();
    buffer[10] = STATIC_THEME_COUNT;
    return 12;
}

// Returns false for an unknown string id.
bool buildString(uint16_t id, uint16_t *len) {
    const char *s = nullptr;
    Storage &storage = Storage::getInstance();
    switch (id) {
        case 0x01: s = GP2040VERSION; break;
        case 0x02: s = GP2040_BOARDCONFIG; break;
        case 0x03: s = BOARD_CONFIG_LABEL; break;
        case 0x04: s = GP2040CONFIG; break;
        case 0x11: s = storage.getGpioMappings().profileLabel; break;
        case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: {
            const ProfileOptions &profiles = storage.getProfileOptions();
            pb_size_t i = id - 0x12;
            s = i < profiles.gpioMappingsSets_count ? profiles.gpioMappingsSets[i].profileLabel : "";
            break;
        }
        default:
            if (id >= STRING_EFFECT_BASE && id < STRING_EFFECT_BASE + effectCount() &&
                id - STRING_EFFECT_BASE < (int)(sizeof EFFECT_NAMES / sizeof EFFECT_NAMES[0])) {
                s = EFFECT_NAMES[id - STRING_EFFECT_BASE];
                break;
            }
            return false;
    }
    size_t n = strnlen(s, MAX_STRING);
    memcpy(buffer, s, n);
    *len = (uint16_t)n;
    return true;
}

uint8_t doSetProfile(const uint8_t *arg, uint16_t argLen) {
    if (argLen < 2) return RES_BAD_LENGTH;
    const uint8_t profile = arg[0], persist = arg[1];
    if (profile < 1 || profile > 6 || persist > 1) return RES_BAD_ARG;
    if (!Storage::getInstance().setProfile(profile)) return RES_PROFILE_DISABLED;
    // The main loop's getReinitGamepad() applies the change on its next pass.
    if (persist) EventManager::getInstance().triggerEvent(new GPStorageSaveEvent(true));
    return RES_OK;
}

uint8_t doReboot(const uint8_t *arg, uint16_t argLen) {
    if (argLen < 1) return RES_BAD_LENGTH;
    System::BootMode mode;
    switch (arg[0]) {
        case 0: mode = System::BootMode::GAMEPAD; break;
        case 1:
            if (forcedLock(true)) return RES_LOCKED;
            mode = System::BootMode::WEBCONFIG;
            break;
        case 2: return RES_UNSUPPORTED; // BOOTSEL: deliberately not offered
        default: return RES_BAD_ARG;
    }
    // checkSaveRebootState() reboots 500 ms later, after our status stage.
    EventManager::getInstance().triggerEvent(new GPRestartEvent(mode));
    rebootPending = true;
    return RES_OK;
}

uint8_t doSetInputMode(const uint8_t *arg, uint16_t argLen) {
    if (argLen < 2) return RES_BAD_LENGTH;
    const uint8_t mode = arg[0];
    if (mode >= 32 || !(BOOTABLE_MODES & (1u << mode))) return RES_UNSUPPORTED;
    if (forcedLock(false)) return RES_LOCKED;
    Storage::getInstance().getGamepadOptions().inputMode = (InputMode)mode;
    // The save handler resets the reboot mode to DEFAULT, so the restart goes
    // second. GAMEPAD skips the boot-time button scan, so no held button can
    // override the new mode.
    EventManager::getInstance().triggerEvent(new GPStorageSaveEvent(true));
    EventManager::getInstance().triggerEvent(new GPRestartEvent(System::BootMode::GAMEPAD));
    rebootPending = true;
    return RES_OK;
}

uint8_t doSetLed(const uint8_t *arg, uint16_t argLen) {
    if (argLen < 8) return RES_BAD_LENGTH;
    if (!ledsRunning) return RES_UNSUPPORTED;
    const uint8_t mask = arg[0], persist = arg[6];
    const LedValues want{arg[1], arg[2], arg[3], arg[4], arg[5]};
    if ((mask & ~LED_MASK_ALL) || persist > 1) return RES_BAD_ARG;
    if ((mask & LED_ANIMATION) && want.animation >= effectCount()) return RES_BAD_ARG;
    if ((mask & LED_BRIGHTNESS) && want.brightness > Storage::getInstance().getLedOptions().brightnessSteps) return RES_BAD_ARG;
    if ((mask & LED_STATIC_COLOR) && want.staticColor >= colors.size()) return RES_BAD_ARG;
    if ((mask & LED_BUTTON_COLOR) && want.buttonColor >= colors.size()) return RES_BAD_ARG;
    if ((mask & LED_THEME) && want.theme >= STATIC_THEME_COUNT) return RES_BAD_ARG;

    critical_section_enter_blocking(&ledLock);
    if (!ledPendingValid) { ledPendingMask = 0; ledPendingPersist = false; }
    overlay(ledPending, mask, want);
    ledPendingMask |= mask;
    ledPendingPersist = ledPendingPersist || persist;
    ledPendingValid = true;
    critical_section_exit(&ledLock);
    return RES_OK;
}

uint8_t runCommand(uint8_t cmd, uint16_t len) {
    const uint8_t *arg = buffer + HEADER_LEN;
    const uint16_t argLen = len - HEADER_LEN;
    if (get32(buffer) != sessionToken || sessionToken == 0) return RES_BAD_TOKEN;
    if (rebootPending) return RES_BUSY;
    switch (cmd) {
        case CMD_SET_PROFILE:    return doSetProfile(arg, argLen);
        case CMD_REBOOT:         return doReboot(arg, argLen);
        case CMD_SET_INPUT_MODE: return doSetInputMode(arg, argLen);
        case CMD_SET_LED:        return doSetLed(arg, argLen);
        default:                 return RES_UNSUPPORTED;
    }
}

} // namespace

bool ControlChannel::matches(tusb_control_request_t const *request) {
    if (request->bRequest != REQ_CODE || (request->wIndex >> 8) != INDEX_MAGIC) return false;
    if (request->bmRequestType != REQ_TYPE_IN && request->bmRequestType != REQ_TYPE_OUT) return false;
    // A known command sent in the wrong direction is not ours. An unknown
    // command carrying our magic is ours and STALLs.
    const int dir = commandDirection(commandOf(request));
    return dir < 0 || dir == request->bmRequestType_bit.direction;
}

bool ControlChannel::handle(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
    const uint8_t cmd = commandOf(request);

    if (stage == CONTROL_STAGE_SETUP) {
        if (commandDirection(cmd) < 0 || request->wLength > MAX_LEN) {
            if (request->bmRequestType_bit.direction == TUSB_DIR_OUT) {
                lastCmd = cmd; lastSeq = 0; lastResult = commandDirection(cmd) < 0 ? RES_UNSUPPORTED : RES_BAD_LENGTH;
            }
            return false;
        }
        uint16_t len = 0;
        switch (cmd) {
            case CMD_HELLO:  len = buildHello(request->wValue); break;
            case CMD_STATUS: len = buildStatus(); break;
            case CMD_STRING: if (!buildString(request->wValue, &len)) return false; break;
            case CMD_LED_STATUS: if (!ledsRunning) return false; len = buildLedStatus(); break;
            default: // OUT command: receive the payload into our buffer
                if (request->wLength < HEADER_LEN) {
                    lastCmd = cmd; lastSeq = 0; lastResult = RES_BAD_LENGTH;
                    return false;
                }
                return tud_control_xfer(rhport, request, buffer, request->wLength);
        }
        return tud_control_xfer(rhport, request, buffer, len);
    }

    if (stage == CONTROL_STAGE_DATA && request->bmRequestType_bit.direction == TUSB_DIR_OUT) {
        // The whole payload has arrived (wLength <= 64, a single packet).
        lastCmd = cmd;
        lastSeq = get16(buffer + 4);
        lastResult = runCommand(cmd, request->wLength);
        return lastResult == RES_OK; // false STALLs the status stage
    }

    return true;
}

void ControlChannel::ledSetup() {
    // core1 setup: core0 is still waiting for us, so registering can't race it
    critical_section_init(&ledLock);
    ledSaved = currentLeds();
    EventManager::getInstance().registerEventHandler(GP_EVENT_STORAGE_SAVE, [](GPEvent *) { handleStorageSave(); });
    ledsRunning = true;
}

void ControlChannel::ledProcess(AnimationStation &as) {
    if (!ledPendingValid) return;
    critical_section_enter_blocking(&ledLock);
    const uint8_t mask = ledPendingMask;
    const bool persist = ledPendingPersist;
    const LedValues v = ledPending;
    ledPendingValid = false;
    critical_section_exit(&ledLock);

    // theme before SetMode(): StaticTheme's constructor checks it
    AnimationOptions &options = Storage::getInstance().getAnimationOptions();
    if (mask & LED_STATIC_COLOR) options.staticColorIndex = v.staticColor;
    if (mask & LED_BUTTON_COLOR) options.buttonColorIndex = v.buttonColor;
    if (mask & LED_THEME)        options.themeIndex = v.theme;
    if ((mask & LED_ANIMATION) && options.baseAnimationIndex != v.animation) as.SetMode(v.animation);
    if (mask & LED_BRIGHTNESS)   as.SetBrightness(v.brightness);
    if (persist) EventManager::getInstance().triggerEvent(new GPStorageSaveEvent(false));
}
