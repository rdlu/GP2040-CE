/*
 * SPDX-License-Identifier: MIT
 */

#include "controlchannel.h"

#include <algorithm>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "hardware/watchdog.h"
#include "pico/critical_section.h"
#include "pico/time.h"
#include "pico/unique_id.h"
#include "pb_common.h"
#include "pb_decode.h"
#include "pb_encode.h"

#include "BoardConfig.h"
#include "CRC32.h"
#include "FlashPROM.h"
#include "animationstation.h"
#include "addons/turbo.h"
#include "config.pb.h"
#include "enums.pb.h"
#include "drivermanager.h"
#include "drivers/keyboard/KeyboardDescriptors.h"
#include "eventmanager.h"
#include "storagemanager.h"
#include "system.h"
#include "version.h"

namespace {

constexpr uint8_t REQ_TYPE_IN  = 0xC0;
constexpr uint8_t REQ_TYPE_OUT = 0x40;
constexpr uint8_t REQ_CODE     = 0x47;
constexpr uint8_t INDEX_MAGIC  = 0x50;

constexpr uint8_t PROTO_MAJOR = 1;
constexpr uint8_t PROTO_MINOR = 13;

constexpr uint16_t MAX_LEN    = 64;
constexpr uint16_t HEADER_LEN = 8;
constexpr uint16_t MAX_STRING = 63;

// READ_SLOT reply and WRITE_SLOT argument: a 20-byte head, then 10 bytes for each pin
constexpr uint16_t SLOT_HEAD_LEN = 20;
constexpr uint16_t SLOT_PIN_LEN  = 10;
constexpr uint16_t SLOT_LABEL    = 16;
constexpr uint16_t SLOT_LEN      = SLOT_HEAD_LEN + SLOT_PIN_LEN * NUM_BANK0_GPIOS;
// READ_LIGHTS reply and WRITE_LIGHTS argument: a 4-byte head, then an AnimationProfile protobuf
constexpr uint16_t LIGHTS_HEAD_LEN = 4;
constexpr uint16_t LIGHTS_LEN      = LIGHTS_HEAD_LEN + AnimationProfile_size;
constexpr uint16_t BUFFER_LEN      = HEADER_LEN + std::max(SLOT_LEN, LIGHTS_LEN);

// The saved config is the protobuf data at the end of the FlashPROM block, then this footer
// (ConfigFooter in config_utils.cpp): u32 data size, u32 CRC-32, u32 magic
constexpr uint32_t CONFIG_FOOTER_LEN = 12;
constexpr uint32_t CONFIG_MAGIC      = 0xd2f1e365;
// READ_CONFIG image: u32 size, u32 CRC-32, u32 most bytes WRITE_CONFIG can take, then the data
constexpr uint16_t CONFIG_HEAD_LEN   = 12;
constexpr uint16_t CONFIG_CHUNK_HEAD = 4;

constexpr uint16_t INPUT_LEN       = 36;
// SET_INPUT_MODE with ONE_BOOT keeps the mode here for the next boot; a power-on clears it
constexpr uint8_t  ONE_BOOT_SCRATCH = 6;
constexpr uint32_t ONE_BOOT_MAGIC   = 0x4f420000;
constexpr uint32_t SCAN_TIMEOUT_MS = 10000;

constexpr uint16_t COLORS_HEAD_LEN = 4;
static_assert(COLORS_HEAD_LEN + 3 * MAX_CUSTOM_COLORS <= MAX_LEN);

// READ_KEYMAP reply and WRITE_KEYMAP argument: a 4-byte head, then a keycode for each KeyboardMapping field
constexpr uint16_t KEYMAP_HEAD_LEN = 4;
constexpr uint8_t  KEYMAP_KEYS     = 32;
static_assert(KEYMAP_HEAD_LEN + KEYMAP_KEYS <= MAX_LEN);
// READ_HOTKEYS reply and WRITE_HOTKEYS argument: a 4-byte head, then 8 bytes for each HotkeyEntry
constexpr uint16_t HOTKEYS_HEAD_LEN = 4;
constexpr uint8_t  HOTKEY_LEN       = 8;
constexpr uint8_t  HOTKEYS          = 16;
constexpr uint16_t HOTKEYS_LEN      = HOTKEYS_HEAD_LEN + HOTKEY_LEN * HOTKEYS;
static_assert(HOTKEYS_LEN <= BUFFER_LEN - HEADER_LEN && _GamepadHotkey_MAX <= 0xff);
// READ_INPUT_OPTIONS reply and WRITE_INPUT_OPTIONS argument: the d-pad and SOCD modes, then the turbo options
constexpr uint16_t INPUT_OPTIONS_LEN = 28;

constexpr uint16_t LOG_RECORDS     = 64;
constexpr uint16_t LOG_DATA        = 24;
constexpr uint16_t LOG_HEAD_LEN    = 12;
constexpr uint32_t LOG_MAGIC       = 0x474c4f47;
constexpr uint32_t LOG_EARLY_MS    = 10000;
constexpr uint8_t  LOG_EARLY_MAX   = 3;
constexpr uint32_t STEPS_MAGIC     = 0x53544550;
constexpr uint32_t WATCHDOG_MS     = 3000;
static_assert(sizeof(GpioMappings::profileLabel) > SLOT_LABEL);

enum Command : uint8_t {
    CMD_HELLO          = 0x01,
    CMD_STATUS         = 0x02,
    CMD_STRING         = 0x03,
    CMD_LIGHTS_STATUS  = 0x05,
    CMD_READ_SLOT      = 0x06,
    CMD_READ_LIGHTS    = 0x07,
    CMD_READ_CONFIG    = 0x08,
    CMD_READ_INPUT     = 0x09,
    CMD_READ_LOG       = 0x0A,
    CMD_READ_CUSTOM_COLORS = 0x0B,
    CMD_READ_KEYMAP    = 0x0C,
    CMD_READ_HOTKEYS   = 0x0D,
    CMD_READ_INPUT_OPTIONS = 0x0E,
    CMD_SET_PROFILE    = 0x10,
    CMD_REBOOT         = 0x11,
    CMD_SET_INPUT_MODE = 0x12,
    CMD_SET_LIGHTS     = 0x14,
    CMD_WRITE_SLOT     = 0x15,
    CMD_WRITE_LIGHTS   = 0x16,
    CMD_WRITE_CONFIG   = 0x17,
    CMD_COMMIT_CONFIG  = 0x18,
    CMD_PIN_SCAN       = 0x19,
    CMD_WRITE_CUSTOM_COLORS = 0x1A,
    CMD_WRITE_KEYMAP   = 0x1B,
    CMD_WRITE_HOTKEYS  = 0x1C,
    CMD_WRITE_INPUT_OPTIONS = 0x1D,
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
    CAP_SET_INPUT_MODE = 1u << 5,
    CAP_LIGHTS         = 1u << 9,
    CAP_SLOTS          = 1u << 10,
    CAP_LIGHTS_PROFILES = 1u << 11,
    CAP_CONFIG          = 1u << 12,
    CAP_INPUT           = 1u << 13,
    CAP_LOG             = 1u << 14,
    CAP_CUSTOM_COLORS   = 1u << 15,
    CAP_KEYMAP          = 1u << 16,
    CAP_HOTKEYS         = 1u << 17,
    CAP_ONE_BOOT_MODE   = 1u << 18,
    CAP_INPUT_OPTIONS   = 1u << 19,
    CAP_DIAG            = 1u << 20,
};

constexpr uint32_t CAPABILITIES = CAP_STATUS | CAP_STRING | CAP_SET_PROFILE | CAP_REBOOT | CAP_SET_INPUT_MODE | CAP_SLOTS |
                                  CAP_CONFIG | CAP_INPUT | CAP_LOG | CAP_KEYMAP | CAP_HOTKEYS |
                                  CAP_ONE_BOOT_MODE | CAP_INPUT_OPTIONS | CAP_DIAG;

// The modes DriverManager::setup() can start
constexpr uint32_t BOOTABLE_MODES =
    (1u << INPUT_MODE_XINPUT) | (1u << INPUT_MODE_SWITCH) | (1u << INPUT_MODE_PS3) |
    (1u << INPUT_MODE_KEYBOARD) | (1u << INPUT_MODE_PS4) | (1u << INPUT_MODE_XBONE) |
    (1u << INPUT_MODE_MDMINI) | (1u << INPUT_MODE_NEOGEO) | (1u << INPUT_MODE_PCEMINI) |
    (1u << INPUT_MODE_EGRET) | (1u << INPUT_MODE_ASTRO) | (1u << INPUT_MODE_PSCLASSIC) |
    (1u << INPUT_MODE_XBOXORIGINAL) | (1u << INPUT_MODE_PS5) | (1u << INPUT_MODE_GENERIC) |
    (1u << INPUT_MODE_SWITCH_PRO) | (1u << INPUT_MODE_P5GENERAL) | (1u << INPUT_MODE_SINPUT) |
    (1u << INPUT_MODE_SWITCH2_PRO);

enum StatusFlag : uint8_t {
    FLAG_PROFILE_PENDING = 1u << 0,
    FLAG_REBOOT_PENDING  = 1u << 1,
    FLAG_ONE_BOOT_MODE   = 1u << 2,
};

constexpr uint8_t INPUT_MODE_ONE_BOOT = 1u << 0;

enum InputOptionsMask : uint8_t {
    INPUT_OPTION_DPAD_MODE     = 1u << 0,
    INPUT_OPTION_SOCD_MODE     = 1u << 1,
    INPUT_OPTION_TURBO         = 1u << 2,
    INPUT_OPTION_TURBO_SPEED   = 1u << 3,
    INPUT_OPTION_TURBO_BUTTONS = 1u << 4,
    INPUT_OPTION_SHMUP         = 1u << 5,
    INPUT_OPTIONS_ALL          = 0x3f,
};

constexpr uint8_t INPUT_OPTIONS_TURBO_RUNS = 1u << 0;

enum SlotFlag : uint8_t {
    SLOT_ENABLED = 1u << 0, // WRITE_SLOT enables the profile
    SLOT_ACTIVE  = 1u << 1, // WRITE_SLOT selects it now
    SLOT_PERSIST = 1u << 2, // WRITE_SLOT only: save to flash
    SLOT_FLAGS_ALL = 0x07,
};

enum LightsFlag : uint8_t {
    LIGHTS_FLAG_RUNNING = 1u << 0,
    LIGHTS_FLAG_UNSAVED = 1u << 1,
    LIGHTS_FLAG_PREVIEW = 1u << 2,
    LIGHTS_FLAG_PENDING = 1u << 3, // a change waits for the LED add-on on core1
};

enum LightsMask : uint8_t {
    LIGHTS_PROFILE    = 1u << 0,
    LIGHTS_BRIGHTNESS = 1u << 1,
    LIGHTS_MASK_ALL   = 0x03,
};

struct LightsValues {
    int8_t profile;
    uint8_t brightness;
};

enum LogCode : uint8_t {
    LOG_BOOT     = 0x01,
    LOG_PANIC    = 0x02,
    LOG_COMMAND  = 0x03,
    LOG_PROFILE  = 0x04,
    LOG_SAVE     = 0x05,
    LOG_RESTART  = 0x06,
    LOG_ERROR    = 0x07,
    LOG_USB_HOST = 0x08,
    LOG_WATCHDOG = 0x09,
    LOG_FAULT    = 0x0A,
};

struct LogRecord {
    uint32_t ms;
    uint16_t boot;
    uint8_t code;
    uint8_t len;
    uint8_t data[LOG_DATA];
};
static_assert(sizeof(LogRecord) == 32);

// A soft reboot keeps RAM; check makes a power-on or a new build start a new log
struct LogRing {
    uint32_t magic;
    uint32_t layout;
    uint32_t next;
    uint16_t boot;
    uint8_t earlyPanics;
    uint8_t reserved;
    uint32_t check;
    LogRecord records[LOG_RECORDS];
};

constexpr uint32_t LOG_LAYOUT = (LOG_RECORDS << 16) | sizeof(LogRecord);

// The last step of core 0, core 1 and the flash write (in an alarm on core 0), and when each one began
struct Steps {
    uint32_t magic;
    volatile uint16_t step[3];
    volatile uint32_t ms[3];
    volatile uint32_t core1Loop;
};

// tud_control_xfer() keeps this pointer until the transfer completes
uint8_t buffer[BUFFER_LEN];

uint32_t sessionToken = 0;
bool rebootPending = false;
uint16_t lastSeq = 0;
uint8_t lastCmd = 0;
uint8_t lastResult = RES_OK;

// Shared with core1; lightsLock guards the pending request and lightsSaved
volatile bool lightsRunning = false;
critical_section_t lightsLock;
volatile bool lightsPendingValid = false;
uint8_t lightsPendingMask = 0;
bool lightsPendingPersist = false;
LightsValues lightsPending = {};
LightsValues lightsSaved = {};
volatile bool lightsProfileValid = false;
int8_t lightsProfileIndex = 0;
uint8_t lightsProfileFlags = 0;
AnimationProfile lightsProfile = {};
volatile bool lightsColorsValid = false;
uint8_t lightsColorsCount = 0;
bool lightsColorsPersist = false;
uint32_t lightsColors[MAX_CUSTOM_COLORS] = {};

// Pins that PIN_SCAN made inputs, and when the scan stops if READ_INPUT stops
uint64_t scanPins = 0;
bool scanActive = false;
absolute_time_t scanDeadline;

LogRing __uninitialized_ram(logRing);
Steps __uninitialized_ram(steps);
critical_section_t logLock;

inline void put16(uint8_t *p, uint16_t v) { p[0] = v & 0xff; p[1] = v >> 8; }
inline void put32(uint8_t *p, uint32_t v) { put16(p, v & 0xffff); put16(p + 2, v >> 16); }
inline uint16_t get16(const uint8_t *p) { return p[0] | (p[1] << 8); }
inline uint32_t get32(const uint8_t *p) { return get16(p) | ((uint32_t)get16(p + 2) << 16); }

inline uint32_t logCheck() { return logRing.magic ^ logRing.layout ^ logRing.next ^ logRing.boot ^ logRing.earlyPanics; }

void logAddUnlocked(uint8_t code, const void *data, uint8_t len) {
    LogRecord &r = logRing.records[logRing.next % LOG_RECORDS];
    r.ms = to_ms_since_boot(get_absolute_time());
    r.boot = logRing.boot;
    r.code = code;
    r.len = std::min<uint8_t>(len, LOG_DATA);
    memset(r.data, 0, LOG_DATA);
    memcpy(r.data, data, r.len);
    logRing.next++;
    logRing.check = logCheck();
}

void logAdd(uint8_t code, const void *data, uint8_t len) {
    critical_section_enter_blocking(&logLock);
    logAddUnlocked(code, data, len);
    critical_section_exit(&logLock);
}

void logText(uint8_t code, const char *text) { logAdd(code, text, strnlen(text, LOG_DATA)); }

uint8_t bootModeIndex(uint32_t mode) {
    switch ((System::BootMode)mode) {
        case System::BootMode::DEFAULT:   return 0;
        case System::BootMode::GAMEPAD:   return 1;
        case System::BootMode::WEBCONFIG: return 2;
        case System::BootMode::USB:       return 3;
        default:                          return 0xff;
    }
}

void setResult(uint8_t cmd, uint16_t seq, uint8_t result) {
    lastCmd = cmd;
    lastSeq = seq;
    lastResult = result;
    const uint8_t data[4] = { cmd, result, (uint8_t)(seq & 0xff), (uint8_t)(seq >> 8) };
    logAdd(LOG_COMMAND, data, sizeof(data));
}

inline uint8_t commandOf(tusb_control_request_t const *request) { return request->wIndex & 0xff; }

int commandDirection(uint8_t cmd) {
    switch (cmd) {
        case CMD_HELLO:
        case CMD_STATUS:
        case CMD_STRING:
        case CMD_LIGHTS_STATUS:
        case CMD_READ_SLOT:
        case CMD_READ_LIGHTS:
        case CMD_READ_CONFIG:
        case CMD_READ_INPUT:
        case CMD_READ_LOG:
        case CMD_READ_CUSTOM_COLORS:
        case CMD_READ_KEYMAP:
        case CMD_READ_HOTKEYS:
        case CMD_READ_INPUT_OPTIONS:
            return TUSB_DIR_IN;
        case CMD_SET_PROFILE:
        case CMD_REBOOT:
        case CMD_SET_INPUT_MODE:
        case CMD_SET_LIGHTS:
        case CMD_WRITE_SLOT:
        case CMD_WRITE_LIGHTS:
        case CMD_WRITE_CONFIG:
        case CMD_COMMIT_CONFIG:
        case CMD_PIN_SCAN:
        case CMD_WRITE_CUSTOM_COLORS:
        case CMD_WRITE_KEYMAP:
        case CMD_WRITE_HOTKEYS:
        case CMD_WRITE_INPUT_OPTIONS:
            return TUSB_DIR_OUT;
        default:
            return -1;
    }
}

// The host can't know the pin count before READ_SLOT, so its wLength isn't limited
uint16_t maxLength(uint8_t cmd) {
    switch (cmd) {
        case CMD_READ_SLOT:
        case CMD_READ_LIGHTS:
        case CMD_READ_CONFIG:
        case CMD_READ_HOTKEYS:
        case CMD_READ_LOG:     return UINT16_MAX;
        case CMD_WRITE_CONFIG: return BUFFER_LEN;
        case CMD_WRITE_SLOT:   return HEADER_LEN + SLOT_LEN;
        case CMD_WRITE_LIGHTS: return HEADER_LEN + LIGHTS_LEN;
        case CMD_WRITE_HOTKEYS: return HEADER_LEN + HOTKEYS_LEN;
        default:             return MAX_LEN;
    }
}

inline bool fixedAction(GpioAction action) {
    return action == GpioAction::RESERVED || action == GpioAction::ASSIGNED_TO_ADDON;
}

LightsValues currentLights() {
    return LightsValues{AnimationStation::getInstance().GetSavedMode(),
                        (uint8_t)Storage::getInstance().getAnimationOptions().brightness};
}

void overlay(LightsValues &v, uint8_t mask, const LightsValues &with) {
    if (mask & LIGHTS_PROFILE)    v.profile    = with.profile;
    if (mask & LIGHTS_BRIGHTNESS) v.brightness = with.brightness;
}

void handleStorageSave() {
    LightsValues now = currentLights();
    critical_section_enter_blocking(&lightsLock);
    lightsSaved = now;
    critical_section_exit(&lightsLock);
}

bool profileEnabled(int8_t profile) {
    if (profile == -1) return true;
    return profile >= 0 && profile < MAX_ANIMATION_PROFILES &&
           Storage::getInstance().getAnimationOptions().profiles[profile].bEnabled;
}

uint32_t makeToken() {
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    uint32_t h = 2166136261u;
    for (uint8_t b : id.id) { h ^= b; h *= 16777619u; }
    h ^= time_us_32();
    h ^= h >> 16; h *= 0x85ebca6bu;
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
    put32(buffer + 8, CAPABILITIES | (lightsRunning ? CAP_LIGHTS | CAP_LIGHTS_PROFILES | CAP_CUSTOM_COLORS : 0));
    put32(buffer + 12, sessionToken);
    put32(buffer + 16, BOOTABLE_MODES);
    return 20;
}

uint16_t buildStatus() {
    Storage &storage = Storage::getInstance();
    const ProfileOptions &profiles = storage.getProfileOptions();
    const uint32_t profileNumber = storage.getGamepadOptions().profileNumber;

    uint8_t enabled = 0x01;
    for (pb_size_t i = 0; i < profiles.gpioMappingsSets_count && i < 7; i++) {
        if (profiles.gpioMappingsSets[i].enabled) enabled |= 1u << (i + 1);
    }

    uint8_t flags = 0;
    Gamepad *gamepad = storage.GetGamepad();
    if (gamepad != nullptr && gamepad->lastReinitProfileNumber != profileNumber) flags |= FLAG_PROFILE_PENDING;
    if (rebootPending) flags |= FLAG_REBOOT_PENDING;
    if (storage.isOneBootInputMode()) flags |= FLAG_ONE_BOOT_MODE;

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
    buffer[10] = (uint8_t)storage.getSavedInputMode();
    const uint32_t core1Age = steps.core1Loop ? (to_ms_since_boot(get_absolute_time()) - steps.core1Loop) / 10 : UINT32_MAX;
    buffer[11] = (uint8_t)std::min<uint32_t>(core1Age, 255);
    put32(buffer + 12, to_ms_since_boot(get_absolute_time()));
    return 16;
}

uint16_t buildLightsStatus() {
    Storage &storage = Storage::getInstance();
    const AnimationOptions &options = storage.getAnimationOptions();
    AnimationStation &as = AnimationStation::getInstance();

    LightsValues v = currentLights();
    critical_section_enter_blocking(&lightsLock);
    if (lightsPendingValid) overlay(v, lightsPendingMask, lightsPending);
    const LightsValues saved = lightsSaved;
    critical_section_exit(&lightsLock);

    uint8_t flags = LIGHTS_FLAG_RUNNING;
    if (v.profile != saved.profile || v.brightness != saved.brightness) flags |= LIGHTS_FLAG_UNSAVED;
    if (as.IsTestModeActive()) flags |= LIGHTS_FLAG_PREVIEW;
    if (lightsPendingValid || lightsProfileValid || lightsColorsValid) flags |= LIGHTS_FLAG_PENDING;

    uint8_t enabled = 0;
    for (int i = 0; i < MAX_ANIMATION_PROFILES; i++) {
        if (options.profiles[i].bEnabled) enabled |= 1u << i;
    }

    memset(buffer, 0xff, 16);
    buffer[0] = flags;
    buffer[1] = (uint8_t)v.profile;
    buffer[2] = enabled;
    buffer[3] = MAX_ANIMATION_PROFILES;
    buffer[4] = v.brightness;
    buffer[5] = as.getBrightnessSteps();
    buffer[6] = (uint8_t)std::min<uint32_t>(storage.getLedOptions().brightnessMaximum, 255);
    if (v.profile >= 0 && v.profile < MAX_ANIMATION_PROFILES) {
        const AnimationProfile &p = options.profiles[v.profile];
        buffer[7] = (uint8_t)p.baseNonPressedEffect;
        buffer[8] = (uint8_t)p.basePressedEffect;
        buffer[9] = (uint8_t)p.baseCaseEffect;
        buffer[10] = (uint8_t)std::clamp<int32_t>(p.baseCycleTime, 0, 254);
        buffer[11] = (uint8_t)std::clamp<int32_t>(p.basePressedCycleTime, 0, 254);
        buffer[12] = (uint8_t)std::clamp<int32_t>(p.baseCaseCycleTime, 0, 254);
    }
    buffer[13] = 0;
    put16(buffer + 14, (uint16_t)std::min<uint32_t>(options.autoDisableTime / 1000, 0xffff));
    return 16;
}

bool buildString(uint16_t id, uint16_t *len) {
    const char *s = nullptr;
    Storage &storage = Storage::getInstance();
    char boardId[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
    switch (id) {
        case 0x01: s = GP2040VERSION; break;
        case 0x02: s = GP2040_BOARDCONFIG; break;
        case 0x03: s = BOARD_CONFIG_LABEL; break;
        case 0x04: s = GP2040CONFIG; break;
        case 0x05:
            pico_get_unique_board_id_string(boardId, sizeof(boardId));
            s = boardId;
            break;
        case 0x11: s = storage.getGpioMappings().profileLabel; break;
        case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: {
            const ProfileOptions &profiles = storage.getProfileOptions();
            pb_size_t i = id - 0x12;
            s = i < profiles.gpioMappingsSets_count ? profiles.gpioMappingsSets[i].profileLabel : "";
            break;
        }
        default:
            return false;
    }
    size_t n = strnlen(s, MAX_STRING);
    memcpy(buffer, s, n);
    *len = (uint16_t)n;
    return true;
}

bool buildSlot(uint16_t profile, uint16_t *len) {
    Storage &storage = Storage::getInstance();
    const ProfileOptions &profiles = storage.getProfileOptions();
    if (profile < 1 || profile > 6) return false;
    const bool defined = profile == 1 || profile - 2u < profiles.gpioMappingsSets_count;
    const GpioMappings &slot = profile == 1 ? storage.getGpioMappings() : profiles.gpioMappingsSets[profile - 2];

    memset(buffer, 0, SLOT_LEN);
    buffer[0] = (uint8_t)profile;
    if (defined && (profile == 1 || slot.enabled)) buffer[1] |= SLOT_ENABLED;
    if (storage.getGamepadOptions().profileNumber == profile) buffer[1] |= SLOT_ACTIVE;
    buffer[2] = NUM_BANK0_GPIOS;
    buffer[3] = SLOT_PIN_LEN;
    memcpy(buffer + 4, slot.profileLabel, strnlen(slot.profileLabel, SLOT_LABEL));
    for (Pin_t pin = 0; defined && pin < (Pin_t)NUM_BANK0_GPIOS; pin++) {
        uint8_t *p = buffer + SLOT_HEAD_LEN + pin * SLOT_PIN_LEN;
        put16(p, (uint16_t)(int16_t)slot.pins[pin].action);
        put32(p + 2, slot.pins[pin].customButtonMask);
        put32(p + 6, slot.pins[pin].customDpadMask);
    }
    *len = SLOT_LEN;
    return true;
}

bool buildLights(uint16_t profile, uint16_t *len) {
    if (!lightsRunning || profile >= MAX_ANIMATION_PROFILES) return false;
    const AnimationProfile &p = Storage::getInstance().getAnimationOptions().profiles[profile];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer + LIGHTS_HEAD_LEN, AnimationProfile_size);
    if (!pb_encode(&stream, AnimationProfile_fields, &p)) return false;
    buffer[0] = (uint8_t)profile;
    buffer[1] = (p.bEnabled ? SLOT_ENABLED : 0) | (AnimationStation::getInstance().GetSavedMode() == profile ? SLOT_ACTIVE : 0);
    put16(buffer + 2, (uint16_t)stream.bytes_written);
    *len = LIGHTS_HEAD_LEN + stream.bytes_written;
    return true;
}

template <typename T>
bool colorsValid(const T &c) {
    const size_t count = colors.size() + customColors.size();
    for (pb_size_t i = 0; i < c.size; i++) {
        if (c.bytes[i] >= count) return false;
    }
    return true;
}

inline bool cycleValid(int32_t step) { return step >= 0 && step < CYCLE_STEPS; }

uint32_t savedConfigSize() {
    const uint8_t *footer = EEPROM.writeCache + EEPROM_SIZE_BYTES - CONFIG_FOOTER_LEN;
    const uint32_t size = get32(footer);
    if (get32(footer + 8) != CONFIG_MAGIC || size > EEPROM_SIZE_BYTES - CONFIG_FOOTER_LEN) return 0;
    return size;
}

// WRITE_CONFIG uses only the free start of the cache, so a save that waits for its flash write stays intact
uint32_t configStageMax() { return EEPROM_SIZE_BYTES - CONFIG_FOOTER_LEN - savedConfigSize(); }

bool buildConfig(uint16_t offset, uint16_t wLength, uint16_t *len) {
    const uint32_t size = savedConfigSize();
    const uint32_t imageLen = CONFIG_HEAD_LEN + size;
    if (offset > imageLen) return false;
    const uint8_t *data = EEPROM.writeCache + EEPROM_SIZE_BYTES - CONFIG_FOOTER_LEN - size;
    uint8_t head[CONFIG_HEAD_LEN];
    put32(head, size);
    put32(head + 4, size ? get32(EEPROM.writeCache + EEPROM_SIZE_BYTES - CONFIG_FOOTER_LEN + 4) : 0);
    put32(head + 8, configStageMax());
    const uint16_t n = (uint16_t)std::min<uint32_t>({wLength, BUFFER_LEN, imageLen - offset});
    for (uint16_t i = 0; i < n; i++) {
        const uint32_t at = offset + i;
        buffer[i] = at < CONFIG_HEAD_LEN ? head[at] : data[at - CONFIG_HEAD_LEN];
    }
    *len = n;
    return true;
}

// A light check that the blob is protobuf: each top-level field has a valid tag and length
bool configWireValid(const uint8_t *data, uint32_t size) {
    pb_istream_t stream = pb_istream_from_buffer(data, size);
    while (stream.bytes_left > 0) {
        pb_wire_type_t type;
        uint32_t tag;
        bool eof;
        if (!pb_decode_tag(&stream, &type, &tag, &eof) || tag == 0 || !pb_skip_field(&stream, type)) return false;
    }
    return true;
}

// Like web-config's held pins: unassigned pins that nothing uses become inputs with a pull-up
void startScan() {
    const GpioMappingInfo *pins = Storage::getInstance().getProfilePinMappings();
    for (Pin_t pin = 0; pin < (Pin_t)NUM_BANK0_GPIOS; pin++) {
        if (pins[pin].action == GpioAction::NONE && gpio_get_function(pin) == GPIO_FUNC_NULL) {
            gpio_init(pin);
            gpio_set_dir(pin, GPIO_IN);
            gpio_pull_up(pin);
            scanPins |= uint64_t{1} << pin;
        }
    }
    scanActive = true;
    scanDeadline = make_timeout_time_ms(SCAN_TIMEOUT_MS);
}

void stopScan() {
    // A profile change can give one of these pins an action; then the gamepad owns it
    const GpioMappingInfo *pins = Storage::getInstance().getProfilePinMappings();
    for (Pin_t pin = 0; pin < (Pin_t)NUM_BANK0_GPIOS; pin++) {
        if ((scanPins & (uint64_t{1} << pin)) && pins[pin].action <= 0) gpio_deinit(pin);
    }
    scanPins = 0;
    scanActive = false;
}

uint16_t buildInput() {
    if (scanActive) {
        if (time_reached(scanDeadline)) stopScan();
        else scanDeadline = make_timeout_time_ms(SCAN_TIMEOUT_MS);
    }
    const uint64_t all = (uint64_t{1} << NUM_BANK0_GPIOS) - 1;
    uint64_t readable = 0;
    for (Pin_t pin = 0; pin < (Pin_t)NUM_BANK0_GPIOS; pin++) {
        if (gpio_get_function(pin) == GPIO_FUNC_SIO && !gpio_is_dir_out(pin)) readable |= uint64_t{1} << pin;
    }
    const uint64_t pressed = ~gpio_get_all64() & all & readable;

    memset(buffer, 0, INPUT_LEN);
    put32(buffer, (uint32_t)pressed);
    put32(buffer + 4, (uint32_t)(pressed >> 32));
    put32(buffer + 8, (uint32_t)readable);
    put32(buffer + 12, (uint32_t)(readable >> 32));
    Gamepad *gamepad = Storage::getInstance().GetGamepad();
    if (gamepad != nullptr) {
        const GamepadState &st = gamepad->state;
        buffer[16] = st.dpad;
        put16(buffer + 18, st.aux);
        put32(buffer + 20, st.buttons);
        put16(buffer + 24, st.lx);
        put16(buffer + 26, st.ly);
        put16(buffer + 28, st.rx);
        put16(buffer + 30, st.ry);
        buffer[32] = st.lt;
        buffer[33] = st.rt;
    }
    buffer[17] = scanActive ? 1 : 0;
    return INPUT_LEN;
}

// wValue is the low 16 bits of the first sequence number to read; older records are gone
uint16_t buildLog(uint16_t from, uint16_t wLength) {
    critical_section_enter_blocking(&logLock);
    const uint32_t next = logRing.next;
    const uint32_t oldest = next > LOG_RECORDS ? next - LOG_RECORDS : 0;
    uint32_t first = (next & ~0xffffu) | from;
    if (first > next) first = first >= 0x10000 ? first - 0x10000 : 0;
    if (first < oldest) first = oldest;
    const uint16_t room = (std::min(wLength, BUFFER_LEN) - LOG_HEAD_LEN) / sizeof(LogRecord);
    const uint8_t count = std::min<uint32_t>(next - first, room);
    for (uint8_t i = 0; i < count; i++) {
        memcpy(buffer + LOG_HEAD_LEN + i * sizeof(LogRecord), &logRing.records[(first + i) % LOG_RECORDS], sizeof(LogRecord));
    }
    const uint16_t boot = logRing.boot;
    critical_section_exit(&logLock);

    put32(buffer, first);
    put32(buffer + 4, next);
    put16(buffer + 8, boot);
    buffer[10] = sizeof(LogRecord);
    buffer[11] = count;
    return LOG_HEAD_LEN + count * sizeof(LogRecord);
}

uint8_t doPinScan(const uint8_t *arg, uint16_t argLen) {
    if (argLen < 1) return RES_BAD_LENGTH;
    if (arg[0] > 1) return RES_BAD_ARG;
    if (arg[0] && !scanActive) startScan();
    else if (!arg[0] && scanActive) stopScan();
    else if (arg[0]) scanDeadline = make_timeout_time_ms(SCAN_TIMEOUT_MS);
    return RES_OK;
}

uint8_t doSetProfile(const uint8_t *arg, uint16_t argLen) {
    if (argLen < 2) return RES_BAD_LENGTH;
    const uint8_t profile = arg[0], persist = arg[1];
    if (profile < 1 || profile > 6 || persist > 1) return RES_BAD_ARG;
    if (!Storage::getInstance().setProfile(profile)) return RES_PROFILE_DISABLED;
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
        case 2: return RES_UNSUPPORTED;
        default: return RES_BAD_ARG;
    }
    EventManager::getInstance().triggerEvent(new GPRestartEvent(mode));
    rebootPending = true;
    return RES_OK;
}

uint8_t doSetInputMode(const uint8_t *arg, uint16_t argLen) {
    if (argLen < 2) return RES_BAD_LENGTH;
    const uint8_t mode = arg[0];
    if (mode >= 32 || !(BOOTABLE_MODES & (1u << mode))) return RES_UNSUPPORTED;
    if (forcedLock(false)) return RES_LOCKED;
    if (arg[1] & INPUT_MODE_ONE_BOOT) {
        watchdog_hw->scratch[ONE_BOOT_SCRATCH] = ONE_BOOT_MAGIC | mode;
    } else {
        Storage::getInstance().clearOneBootInputMode();
        Storage::getInstance().getGamepadOptions().inputMode = (InputMode)mode;
        EventManager::getInstance().triggerEvent(new GPStorageSaveEvent(true));
    }
    // GAMEPAD boot mode skips the boot button scan, so a held button can't override the new mode
    EventManager::getInstance().triggerEvent(new GPRestartEvent(System::BootMode::GAMEPAD));
    rebootPending = true;
    return RES_OK;
}

uint8_t doWriteSlot(const uint8_t *arg, uint16_t argLen) {
    if (argLen != SLOT_LEN) return RES_BAD_LENGTH;
    const uint8_t profile = arg[0], flags = arg[1];
    if (profile < 2 || profile > 6 || (flags & ~SLOT_FLAGS_ALL) ||
        arg[2] != NUM_BANK0_GPIOS || arg[3] != SLOT_PIN_LEN) return RES_BAD_ARG;
    for (Pin_t pin = 0; pin < (Pin_t)NUM_BANK0_GPIOS; pin++) {
        const int16_t action = (int16_t)get16(arg + SLOT_HEAD_LEN + pin * SLOT_PIN_LEN);
        if (action < _GpioAction_MIN || action > _GpioAction_MAX) return RES_BAD_ARG;
    }

    Storage &storage = Storage::getInstance();
    ProfileOptions &profiles = storage.getProfileOptions();
    const bool enable = flags & SLOT_ENABLED;
    const bool select = flags & SLOT_ACTIVE;
    if (!enable && (select || storage.getGamepadOptions().profileNumber == profile)) return RES_PROFILE_DISABLED;

    // Configs from before profile 6 have only 4 slots
    while (profiles.gpioMappingsSets_count < profile - 1u) {
        GpioMappings &added = profiles.gpioMappingsSets[profiles.gpioMappingsSets_count++];
        added = GpioMappings{};
        added.pins_count = NUM_BANK0_GPIOS;
    }

    // Like web-config: pins that profile 1 reserves or gives to an add-on keep its action
    GpioMappings &slot = profiles.gpioMappingsSets[profile - 2];
    const GpioMappingInfo *core = storage.getGpioMappings().pins;
    for (Pin_t pin = 0; pin < (Pin_t)NUM_BANK0_GPIOS; pin++) {
        const uint8_t *p = arg + SLOT_HEAD_LEN + pin * SLOT_PIN_LEN;
        GpioMappingInfo &info = slot.pins[pin];
        if (fixedAction(core[pin].action)) {
            info.action = core[pin].action;
            info.customButtonMask = 0;
            info.customDpadMask = 0;
        } else {
            info.action = (GpioAction)(int16_t)get16(p);
            info.customButtonMask = get32(p + 2);
            info.customDpadMask = get32(p + 6);
        }
    }
    slot.pins_count = NUM_BANK0_GPIOS;
    memset(slot.profileLabel, 0, sizeof(slot.profileLabel));
    memcpy(slot.profileLabel, arg + 4, strnlen((const char *)arg + 4, SLOT_LABEL));
    slot.enabled = enable;

    if (select) storage.setProfile(profile);
    // The reinit in the main loop only runs when the profile number changes, so force it
    Gamepad *gamepad = storage.GetGamepad();
    if (gamepad != nullptr && storage.getGamepadOptions().profileNumber == profile) gamepad->lastReinitProfileNumber = 0;
    if (flags & SLOT_PERSIST) EventManager::getInstance().triggerEvent(new GPStorageSaveEvent(true));
    return RES_OK;
}

uint8_t doWriteLights(const uint8_t *arg, uint16_t argLen) {
    if (argLen < LIGHTS_HEAD_LEN) return RES_BAD_LENGTH;
    if (!lightsRunning) return RES_UNSUPPORTED;
    const uint8_t profile = arg[0], flags = arg[1];
    if (profile >= MAX_ANIMATION_PROFILES || (flags & ~SLOT_FLAGS_ALL) || get16(arg + 2) != 0) return RES_BAD_ARG;
    AnimationStation &as = AnimationStation::getInstance();
    if (as.IsTestModeActive() || lightsProfileValid) return RES_BUSY;
    const bool enable = flags & SLOT_ENABLED;
    if (!enable && ((flags & SLOT_ACTIVE) || as.GetSavedMode() == profile)) return RES_PROFILE_DISABLED;

    // Check only the fields that the host sends: stored profiles can have colors out of range
    static AnimationProfile sent;
    sent = AnimationProfile_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(arg + LIGHTS_HEAD_LEN, argLen - LIGHTS_HEAD_LEN);
    if (!pb_decode(&stream, AnimationProfile_fields, &sent)) return RES_BAD_ARG;
    if (sent.baseNonPressedEffect < _AnimationNonPressedEffects_MIN || sent.baseNonPressedEffect > _AnimationNonPressedEffects_MAX ||
        sent.baseCaseEffect < _AnimationNonPressedEffects_MIN || sent.baseCaseEffect > _AnimationNonPressedEffects_MAX ||
        sent.basePressedEffect < _AnimationPressedEffects_MIN || sent.basePressedEffect > _AnimationPressedEffects_MAX ||
        !cycleValid(sent.baseCycleTime) || !cycleValid(sent.basePressedCycleTime) || !cycleValid(sent.baseCaseCycleTime) ||
        !colorsValid(sent.notPressedStaticColors) || !colorsValid(sent.pressedStaticColors) || !colorsValid(sent.nonButtonStaticColors))
        return RES_BAD_ARG;

    // Fields that the host leaves out keep their values. Core1 reads lightsProfile only while lightsProfileValid
    lightsProfile = Storage::getInstance().getAnimationOptions().profiles[profile];
    stream = pb_istream_from_buffer(arg + LIGHTS_HEAD_LEN, argLen - LIGHTS_HEAD_LEN);
    if (!pb_decode_ex(&stream, AnimationProfile_fields, &lightsProfile, PB_DECODE_NOINIT)) return RES_BAD_ARG;
    lightsProfile.bEnabled = enable;
    lightsProfile.has_bEnabled = true;

    critical_section_enter_blocking(&lightsLock);
    lightsProfileIndex = (int8_t)profile;
    lightsProfileFlags = flags;
    lightsProfileValid = true;
    critical_section_exit(&lightsLock);
    return RES_OK;
}

uint8_t doWriteConfig(const uint8_t *arg, uint16_t argLen) {
    if (argLen <= CONFIG_CHUNK_HEAD) return RES_BAD_LENGTH;
    if (forcedLock(true)) return RES_LOCKED;
    const uint32_t offset = get16(arg), n = argLen - CONFIG_CHUNK_HEAD;
    if (get16(arg + 2) != 0 || offset + n > configStageMax()) return RES_BAD_ARG;
    memcpy(EEPROM.writeCache + offset, arg + CONFIG_CHUNK_HEAD, n);
    return RES_OK;
}

uint8_t doCommitConfig(const uint8_t *arg, uint16_t argLen) {
    if (argLen < 8) return RES_BAD_LENGTH;
    if (forcedLock(true)) return RES_LOCKED;
    const uint32_t size = get32(arg), crc = get32(arg + 4);
    uint8_t *cache = EEPROM.writeCache;
    // A save during the upload clears the free space, and then the CRC is different
    if (size == 0 || size > configStageMax() || CRC32::calculate(cache, (uint16_t)size) != crc ||
        !configWireValid(cache, size)) return RES_BAD_ARG;

    // Put the blob where ConfigUtils::save() puts it. load() runs the migrations on the next boot
    uint8_t *footer = cache + EEPROM_SIZE_BYTES - CONFIG_FOOTER_LEN;
    memmove(footer - size, cache, size);
    memset(cache, 0, EEPROM_SIZE_BYTES - CONFIG_FOOTER_LEN - size);
    put32(footer, size);
    put32(footer + 4, crc);
    put32(footer + 8, CONFIG_MAGIC);
    EEPROM.commit();
    // The flash write waits EEPROM_WRITE_WAIT ms, and the reboot waits longer
    EventManager::getInstance().triggerEvent(new GPRestartEvent(System::BootMode::GAMEPAD));
    rebootPending = true;
    return RES_OK;
}

// All slots, also the ones past the count: AnimationStation uses all of them
bool buildCustomColors(uint16_t *len) {
    if (!lightsRunning) return false;
    const AnimationOptions &options = Storage::getInstance().getAnimationOptions();
    buffer[0] = options.customColors_count;
    buffer[1] = MAX_CUSTOM_COLORS;
    put16(buffer + 2, 0);
    for (int i = 0; i < MAX_CUSTOM_COLORS; i++) {
        const uint32_t c = options.customColors[i];
        buffer[COLORS_HEAD_LEN + i * 3] = (c >> 16) & 0xff;
        buffer[COLORS_HEAD_LEN + i * 3 + 1] = (c >> 8) & 0xff;
        buffer[COLORS_HEAD_LEN + i * 3 + 2] = c & 0xff;
    }
    *len = COLORS_HEAD_LEN + 3 * MAX_CUSTOM_COLORS;
    return true;
}

uint8_t doWriteCustomColors(const uint8_t *arg, uint16_t argLen) {
    if (argLen < COLORS_HEAD_LEN) return RES_BAD_LENGTH;
    if (!lightsRunning) return RES_UNSUPPORTED;
    const uint8_t count = arg[0], flags = arg[1];
    if (count > MAX_CUSTOM_COLORS || (flags & ~SLOT_PERSIST) || get16(arg + 2) != 0) return RES_BAD_ARG;
    if (argLen < COLORS_HEAD_LEN + 3 * count) return RES_BAD_LENGTH;
    if (AnimationStation::getInstance().IsTestModeActive() || lightsColorsValid) return RES_BUSY;

    critical_section_enter_blocking(&lightsLock);
    for (int i = 0; i < MAX_CUSTOM_COLORS; i++) {
        const uint8_t *c = arg + COLORS_HEAD_LEN + i * 3;
        lightsColors[i] = i < count ? (c[0] << 16) | (c[1] << 8) | c[2] : 0;
    }
    lightsColorsCount = count;
    lightsColorsPersist = flags & SLOT_PERSIST;
    lightsColorsValid = true;
    critical_section_exit(&lightsLock);
    return RES_OK;
}

// KeyboardDriver::pressKey() sends these; the fields are in tag order: the d-pad, then B1 to E12
inline bool keycodeValid(uint8_t code) {
    return code <= HID_KEY_GUI_RIGHT || code == KEYBOARD_MULTIMEDIA_NEXT_TRACK || code == KEYBOARD_MULTIMEDIA_PREV_TRACK ||
           (code >= KEYBOARD_MULTIMEDIA_STOP && code <= KEYBOARD_MULTIMEDIA_VOLUME_DOWN);
}

uint16_t buildKeymap() {
    pb_field_iter_t field;
    uint8_t count = 0;
    if (pb_field_iter_begin(&field, KeyboardMapping_fields, &Storage::getInstance().getKeyboardMapping())) {
        do buffer[KEYMAP_HEAD_LEN + count++] = (uint8_t)*(const uint32_t *)field.pData;
        while (count < KEYMAP_KEYS && pb_field_iter_next(&field));
    }
    buffer[0] = count;
    buffer[1] = 0;
    put16(buffer + 2, 0);
    return KEYMAP_HEAD_LEN + count;
}

uint8_t doWriteKeymap(const uint8_t *arg, uint16_t argLen) {
    if (argLen < KEYMAP_HEAD_LEN) return RES_BAD_LENGTH;
    const uint8_t count = arg[0], flags = arg[1];
    if (count != KEYMAP_KEYS || (flags & ~SLOT_PERSIST) || get16(arg + 2) != 0) return RES_BAD_ARG;
    if (argLen < KEYMAP_HEAD_LEN + count) return RES_BAD_LENGTH;
    const uint8_t *keys = arg + KEYMAP_HEAD_LEN;
    for (uint8_t i = 0; i < count; i++) {
        if (!keycodeValid(keys[i])) return RES_BAD_ARG;
    }

    // KeyboardDriver reads the mapping for each report
    pb_field_iter_t field;
    uint8_t i = 0;
    if (pb_field_iter_begin(&field, KeyboardMapping_fields, &Storage::getInstance().getKeyboardMapping())) {
        do {
            *(uint32_t *)field.pData = keys[i++];
            *(bool *)field.pSize = true;
        } while (i < count && pb_field_iter_next(&field));
    }
    if (flags & SLOT_PERSIST) EventManager::getInstance().triggerEvent(new GPStorageSaveEvent(true));
    return RES_OK;
}

uint16_t buildHotkeys() {
    pb_field_iter_t field;
    uint8_t count = 0;
    if (pb_field_iter_begin(&field, HotkeyOptions_fields, &Storage::getInstance().getHotkeyOptions())) {
        do {
            const HotkeyEntry &e = *(const HotkeyEntry *)field.pData;
            uint8_t *p = buffer + HOTKEYS_HEAD_LEN + count++ * HOTKEY_LEN;
            p[0] = (uint8_t)e.action;
            p[1] = (uint8_t)e.dpadMask;
            put16(p + 2, (uint16_t)e.auxMask);
            put32(p + 4, e.buttonsMask);
        } while (count < HOTKEYS && pb_field_iter_next(&field));
    }
    buffer[0] = count;
    buffer[1] = 0;
    buffer[2] = HOTKEY_LEN;
    buffer[3] = 0;
    return HOTKEYS_HEAD_LEN + count * HOTKEY_LEN;
}

uint8_t doWriteHotkeys(const uint8_t *arg, uint16_t argLen) {
    if (argLen < HOTKEYS_HEAD_LEN) return RES_BAD_LENGTH;
    const uint8_t count = arg[0], flags = arg[1];
    if (count > HOTKEYS || (flags & ~SLOT_PERSIST) || arg[2] != HOTKEY_LEN || arg[3] != 0) return RES_BAD_ARG;
    if (argLen < HOTKEYS_HEAD_LEN + count * HOTKEY_LEN) return RES_BAD_LENGTH;
    for (uint8_t i = 0; i < count; i++) {
        const uint8_t *p = arg + HOTKEYS_HEAD_LEN + i * HOTKEY_LEN;
        // Like web-config: the d-pad is in dpadMask, not in bits 16-19 of buttonsMask
        if (p[0] > _GamepadHotkey_MAX || (p[1] & ~GAMEPAD_MASK_DPAD) || (get16(p + 2) & ~AUX_MASK_FUNCTION) ||
            (get32(p + 4) & (GAMEPAD_MASK_DU | GAMEPAD_MASK_DD | GAMEPAD_MASK_DL | GAMEPAD_MASK_DR))) return RES_BAD_ARG;
    }

    // The entries after count become empty, so that RAM agrees with what a save writes
    pb_field_iter_t field;
    uint8_t i = 0;
    if (pb_field_iter_begin(&field, HotkeyOptions_fields, &Storage::getInstance().getHotkeyOptions())) {
        do {
            const uint8_t *p = arg + HOTKEYS_HEAD_LEN + i * HOTKEY_LEN;
            const bool sent = i++ < count;
            HotkeyEntry &e = *(HotkeyEntry *)field.pData;
            e.action = sent ? (GamepadHotkey)p[0] : HOTKEY_NONE;
            e.dpadMask = sent ? p[1] : 0;
            e.auxMask = sent ? get16(p + 2) : 0;
            e.buttonsMask = sent ? get32(p + 4) : 0;
            e.has_action = e.has_dpadMask = e.has_auxMask = e.has_buttonsMask = true;
            *(bool *)field.pSize = true;
        } while (i < HOTKEYS && pb_field_iter_next(&field));
    }
    // Gamepad::hotkey() uses the copy that setup() makes
    Gamepad *gamepad = Storage::getInstance().GetGamepad();
    if (gamepad != nullptr) gamepad->loadHotkeys();
    if (flags & SLOT_PERSIST) EventManager::getInstance().triggerEvent(new GPStorageSaveEvent(true));
    return RES_OK;
}

// The values in RAM; byte 10 and 11 give the modes that apply now: a held DP/LS/RS pin, and SOCD bypass where the mode can't take it
uint16_t buildInputOptions() {
    const GamepadOptions &options = Storage::getInstance().getGamepadOptions();
    const TurboOptions &turbo = Storage::getInstance().getAddonOptions().turboOptions;
    Gamepad *gamepad = Storage::getInstance().GetGamepad();
    memset(buffer, 0, INPUT_OPTIONS_LEN);
    buffer[0] = TurboInput::active != nullptr ? INPUT_OPTIONS_TURBO_RUNS : 0;
    buffer[2] = (uint8_t)options.dpadMode;
    buffer[3] = (uint8_t)options.socdMode;
    buffer[4] = turbo.enabled ? 1 : 0;
    buffer[5] = (uint8_t)turbo.shotCount;
    put16(buffer + 6, TurboInput::active != nullptr ? TurboInput::active->getTurboButtons() : 0);
    buffer[8] = turbo.shmupModeEnabled ? 1 : 0;
    buffer[9] = (uint8_t)turbo.shmupMixMode;
    buffer[10] = (uint8_t)(gamepad != nullptr ? gamepad->getActiveDpadMode() : options.dpadMode);
    buffer[11] = (uint8_t)Gamepad::resolveSOCDMode(options);
    const uint32_t shmup[8] = { turbo.shmupAlwaysOn1, turbo.shmupAlwaysOn2, turbo.shmupAlwaysOn3, turbo.shmupAlwaysOn4,
                                turbo.shmupBtnMask1, turbo.shmupBtnMask2, turbo.shmupBtnMask3, turbo.shmupBtnMask4 };
    for (int i = 0; i < 8; i++) put16(buffer + 12 + i * 2, (uint16_t)shmup[i]);
    return INPUT_OPTIONS_LEN;
}

uint8_t doWriteInputOptions(const uint8_t *arg, uint16_t argLen) {
    if (argLen < INPUT_OPTIONS_LEN) return RES_BAD_LENGTH;
    const uint8_t mask = arg[0], flags = arg[1];
    const uint16_t buttons = get16(arg + 6);
    bool shmupValid = arg[8] <= 1 && arg[9] <= _ShmupMixMode_MAX;
    for (int i = 0; i < 4; i++) shmupValid = shmupValid && !(get16(arg + 12 + i * 2) & ~TURBO_BUTTON_MASK);
    if ((mask & ~INPUT_OPTIONS_ALL) || (flags & ~SLOT_PERSIST) || get16(arg + 10) != 0 ||
        ((mask & INPUT_OPTION_DPAD_MODE) && arg[2] > _DpadMode_MAX) ||
        ((mask & INPUT_OPTION_SOCD_MODE) && arg[3] > _SOCDMode_MAX) ||
        ((mask & INPUT_OPTION_TURBO) && arg[4] > 1) ||
        ((mask & INPUT_OPTION_TURBO_SPEED) && (arg[5] < TURBO_SHOT_MIN || arg[5] > TURBO_SHOT_MAX)) ||
        ((mask & INPUT_OPTION_TURBO_BUTTONS) && (buttons & ~TURBO_BUTTON_MASK)) ||
        ((mask & INPUT_OPTION_SHMUP) && !shmupValid)) return RES_BAD_ARG;

    // Gamepad reads the modes for each report, and TurboInput sees a new shotCount, as after a hotkey
    GamepadOptions &options = Storage::getInstance().getGamepadOptions();
    TurboOptions &turbo = Storage::getInstance().getAddonOptions().turboOptions;
    if (mask & INPUT_OPTION_DPAD_MODE) options.dpadMode = (DpadMode)arg[2];
    if (mask & INPUT_OPTION_SOCD_MODE) options.socdMode = (SOCDMode)arg[3];
    if (mask & INPUT_OPTION_TURBO) turbo.enabled = arg[4];
    if (mask & INPUT_OPTION_TURBO_SPEED) turbo.shotCount = arg[5];
    if (mask & INPUT_OPTION_SHMUP) {
        uint32_t *shmup[8] = { &turbo.shmupAlwaysOn1, &turbo.shmupAlwaysOn2, &turbo.shmupAlwaysOn3, &turbo.shmupAlwaysOn4,
                               &turbo.shmupBtnMask1, &turbo.shmupBtnMask2, &turbo.shmupBtnMask3, &turbo.shmupBtnMask4 };
        for (int i = 0; i < 8; i++) *shmup[i] = get16(arg + 12 + i * 2);
        turbo.shmupModeEnabled = arg[8];
        turbo.shmupMixMode = (ShmupMixMode)arg[9];
        // setup() reads the SHMUP options, and clears the turbo buttons to the always-on ones
        if (TurboInput::active != nullptr) TurboInput::active->setup();
    }
    if ((mask & INPUT_OPTION_TURBO_BUTTONS) && TurboInput::active != nullptr) TurboInput::active->setTurboButtons(buttons);
    if (flags & SLOT_PERSIST) EventManager::getInstance().triggerEvent(new GPStorageSaveEvent(true));
    return RES_OK;
}

uint8_t doSetLights(const uint8_t *arg, uint16_t argLen) {
    if (argLen < 4) return RES_BAD_LENGTH;
    if (!lightsRunning) return RES_UNSUPPORTED;
    const uint8_t mask = arg[0], persist = arg[3];
    const LightsValues want{(int8_t)arg[1], arg[2]};
    if ((mask & ~LIGHTS_MASK_ALL) || persist > 1) return RES_BAD_ARG;
    AnimationStation &as = AnimationStation::getInstance();
    // A preview runs on the test profile and its changes are never saved
    if (as.IsTestModeActive()) return RES_BUSY;
    if ((mask & LIGHTS_PROFILE) && !profileEnabled(want.profile)) return RES_BAD_ARG;
    if ((mask & LIGHTS_BRIGHTNESS) && want.brightness > as.getBrightnessSteps()) return RES_BAD_ARG;
    if (mask == 0 && !persist) return RES_OK;

    critical_section_enter_blocking(&lightsLock);
    if (!lightsPendingValid) { lightsPendingMask = 0; lightsPendingPersist = false; }
    overlay(lightsPending, mask, want);
    lightsPendingMask |= mask;
    lightsPendingPersist = lightsPendingPersist || persist;
    lightsPendingValid = true;
    critical_section_exit(&lightsLock);
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
        case CMD_SET_LIGHTS:     return doSetLights(arg, argLen);
        case CMD_WRITE_SLOT:     return doWriteSlot(arg, argLen);
        case CMD_WRITE_LIGHTS:   return doWriteLights(arg, argLen);
        case CMD_WRITE_CONFIG:   return doWriteConfig(arg, argLen);
        case CMD_COMMIT_CONFIG:  return doCommitConfig(arg, argLen);
        case CMD_PIN_SCAN:       return doPinScan(arg, argLen);
        case CMD_WRITE_CUSTOM_COLORS: return doWriteCustomColors(arg, argLen);
        case CMD_WRITE_KEYMAP:   return doWriteKeymap(arg, argLen);
        case CMD_WRITE_HOTKEYS:  return doWriteHotkeys(arg, argLen);
        case CMD_WRITE_INPUT_OPTIONS: return doWriteInputOptions(arg, argLen);
        default:                 return RES_UNSUPPORTED;
    }
}

}

bool ControlChannel::matches(tusb_control_request_t const *request) {
    if (request->bRequest != REQ_CODE || (request->wIndex >> 8) != INDEX_MAGIC) return false;
    if (request->bmRequestType != REQ_TYPE_IN && request->bmRequestType != REQ_TYPE_OUT) return false;
    // A known command in the wrong direction goes to the driver; an unknown one is ours and stalls
    const int dir = commandDirection(commandOf(request));
    return dir < 0 || dir == request->bmRequestType_bit.direction;
}

bool ControlChannel::handle(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
    const uint8_t cmd = commandOf(request);

    if (stage == CONTROL_STAGE_SETUP) {
        if (commandDirection(cmd) < 0 || request->wLength > maxLength(cmd)) {
            if (request->bmRequestType_bit.direction == TUSB_DIR_OUT) {
                setResult(cmd, 0, commandDirection(cmd) < 0 ? RES_UNSUPPORTED : RES_BAD_LENGTH);
            }
            return false;
        }
        uint16_t len = 0;
        switch (cmd) {
            case CMD_HELLO:  len = buildHello(request->wValue); break;
            case CMD_STATUS: len = buildStatus(); break;
            case CMD_STRING: if (!buildString(request->wValue, &len)) return false; break;
            case CMD_LIGHTS_STATUS: if (!lightsRunning) return false; len = buildLightsStatus(); break;
            case CMD_READ_SLOT: if (!buildSlot(request->wValue, &len)) return false; break;
            case CMD_READ_LIGHTS: if (!buildLights(request->wValue, &len)) return false; break;
            case CMD_READ_CONFIG: if (!buildConfig(request->wValue, request->wLength, &len)) return false; break;
            case CMD_READ_INPUT: len = buildInput(); break;
            case CMD_READ_CUSTOM_COLORS: if (!buildCustomColors(&len)) return false; break;
            case CMD_READ_KEYMAP: len = buildKeymap(); break;
            case CMD_READ_HOTKEYS: len = buildHotkeys(); break;
            case CMD_READ_INPUT_OPTIONS: len = buildInputOptions(); break;
            case CMD_READ_LOG: if (request->wLength < LOG_HEAD_LEN) return false; len = buildLog(request->wValue, request->wLength); break;
            default:
                if (request->wLength < HEADER_LEN) {
                    setResult(cmd, 0, RES_BAD_LENGTH);
                    return false;
                }
                return tud_control_xfer(rhport, request, buffer, request->wLength);
        }
        return tud_control_xfer(rhport, request, buffer, len);
    }

    if (stage == CONTROL_STAGE_DATA && request->bmRequestType_bit.direction == TUSB_DIR_OUT) {
        setResult(cmd, get16(buffer + 4), runCommand(cmd, request->wLength));
        return lastResult == RES_OK;
    }

    return true;
}

void ControlChannel::step(uint16_t step) {
    const uint core = get_core_num();
    const uint32_t ms = to_ms_since_boot(get_absolute_time());
    steps.step[core] = step;
    steps.ms[core] = ms;
    if (step == STEP_LOOP) {
        if (core == 0) watchdog_update();
        else steps.core1Loop = ms;
    }
}

void ControlChannel::watchdogStart() { watchdog_enable(WATCHDOG_MS, true); }

extern "C" void flashWriteStep(uint8_t step) {
    steps.step[2] = step;
    steps.ms[2] = to_ms_since_boot(get_absolute_time());
}

bool ControlChannel::takeOneBootMode(InputMode &mode) {
    const uint32_t value = watchdog_hw->scratch[ONE_BOOT_SCRATCH];
    watchdog_hw->scratch[ONE_BOOT_SCRATCH] = 0;
    const uint32_t m = value & 0xffff;
    if (!watchdog_caused_reboot() || (value & 0xffff0000) != ONE_BOOT_MAGIC || m >= 32 || !(BOOTABLE_MODES & (1u << m)))
        return false;
    mode = (InputMode)m;
    return true;
}

void ControlChannel::lightsSetup() {
    // TinyUSB takes the free spin locks (24-31); with one more claim, a later claim panics at boot
    critical_section_init_with_lock_num(&lightsLock, next_striped_spin_lock_num());
    lightsSaved = currentLights();
    EventManager::getInstance().registerEventHandler(GP_EVENT_STORAGE_SAVE, [](GPEvent *) { handleStorageSave(); }, nullptr);
    lightsRunning = true;
}

static void applyLightsProfile(AnimationStation &as) {
    critical_section_enter_blocking(&lightsLock);
    const int8_t index = lightsProfileIndex;
    const uint8_t flags = lightsProfileFlags;
    Storage::getInstance().getAnimationOptions().profiles[index] = lightsProfile;
    lightsProfileValid = false;
    critical_section_exit(&lightsLock);

    // SetMode builds the effects again from the profile
    if (!as.IsTestModeActive() && ((flags & SLOT_ACTIVE) || as.GetSavedMode() == index)) {
        const bool profileChange = as.GetSavedMode() != index;
        as.SetMode(index);
        if (profileChange) EventManager::getInstance().triggerEvent(new GPLEDEvent(true, false, false, false, false));
    }
    if (flags & SLOT_PERSIST) EventManager::getInstance().triggerEvent(new GPStorageSaveEvent(false));
}

static void applyCustomColors(AnimationStation &as) {
    AnimationOptions &options = Storage::getInstance().getAnimationOptions();
    critical_section_enter_blocking(&lightsLock);
    memcpy(options.customColors, lightsColors, sizeof(lightsColors));
    options.customColors_count = lightsColorsCount;
    const bool persist = lightsColorsPersist;
    lightsColorsValid = false;
    critical_section_exit(&lightsLock);

    // The effects keep the colors that they got when SetMode built them
    as.InitSettings();
    if (!as.IsTestModeActive()) as.SetMode(as.GetSavedMode());
    if (persist) EventManager::getInstance().triggerEvent(new GPStorageSaveEvent(false));
}

void ControlChannel::lightsProcess(AnimationStation &as) {
    if (lightsColorsValid) applyCustomColors(as);
    if (lightsProfileValid) applyLightsProfile(as);
    if (!lightsPendingValid) return;
    critical_section_enter_blocking(&lightsLock);
    const uint8_t mask = lightsPendingMask;
    const bool persist = lightsPendingPersist;
    const LightsValues v = lightsPending;
    lightsPendingValid = false;
    critical_section_exit(&lightsLock);

    if (as.IsTestModeActive()) return;
    AnimationOptions &options = Storage::getInstance().getAnimationOptions();
    const bool profileChange = (mask & LIGHTS_PROFILE) && as.GetSavedMode() != v.profile;
    const bool brightnessChange = (mask & LIGHTS_BRIGHTNESS) && options.brightness != v.brightness;
    if (profileChange) as.SetMode(v.profile);
    if (brightnessChange) {
        options.brightness = v.brightness;
        as.SetBrightnessStepValue(v.brightness);
    }
    if (profileChange || brightnessChange)
        EventManager::getInstance().triggerEvent(new GPLEDEvent(profileChange, false, false, false, brightnessChange));
    if (persist) EventManager::getInstance().triggerEvent(new GPStorageSaveEvent(false));
}

// Runs before main(): the watchdog scratch register still holds the boot mode
__attribute__((constructor)) static void logSetup() {
    critical_section_init_with_lock_num(&logLock, next_striped_spin_lock_num());
    const bool kept = watchdog_caused_reboot() && logRing.magic == LOG_MAGIC && logRing.layout == LOG_LAYOUT &&
                      logRing.check == logCheck();
    if (!kept) {
        memset(&logRing, 0, sizeof(logRing));
        logRing.magic = LOG_MAGIC;
        logRing.layout = LOG_LAYOUT;
    } else {
        logRing.boot++;
    }

    const bool timeout = watchdog_enable_caused_reboot();
    uint8_t data[LOG_DATA] = { (uint8_t)((kept ? 1 : 0) | (watchdog_caused_reboot() ? 2 : 0) | (timeout ? 4 : 0)),
                               bootModeIndex(watchdog_hw->scratch[5]) };
    const uint8_t versionLen = strnlen(GP2040VERSION, LOG_DATA - 2);
    memcpy(data + 2, GP2040VERSION, versionLen);
    logAddUnlocked(LOG_BOOT, data, 2 + versionLen);
    if (kept && timeout && steps.magic == STEPS_MAGIC) {
        uint8_t d[22];
        for (int i = 0; i < 3; i++) {
            put16(d + i * 2, steps.step[i]);
            put32(d + 6 + i * 4, steps.ms[i]);
        }
        put32(d + 18, steps.core1Loop);
        logAddUnlocked(LOG_WATCHDOG, d, sizeof(d));
    }
    memset(&steps, 0, sizeof(steps));
    steps.magic = STEPS_MAGIC;

    EventManager &events = EventManager::getInstance();
    events.registerEventHandler(GP_EVENT_PROFILE_CHANGE, [](GPEvent *e) {
        const GPProfileChangeEvent *p = static_cast<GPProfileChangeEvent *>(e);
        const uint8_t d[2] = { p->previousValue, p->currentValue };
        logAdd(LOG_PROFILE, d, sizeof(d));
    }, nullptr);
    events.registerEventHandler(GP_EVENT_STORAGE_SAVE, [](GPEvent *e) {
        const GPStorageSaveEvent *p = static_cast<GPStorageSaveEvent *>(e);
        const uint8_t d[2] = { p->forceSave, p->restartAfterSave };
        logAdd(LOG_SAVE, d, sizeof(d));
    }, nullptr);
    events.registerEventHandler(GP_EVENT_RESTART, [](GPEvent *e) {
        const uint8_t d = bootModeIndex((uint32_t)static_cast<GPRestartEvent *>(e)->bootMode);
        critical_section_enter_blocking(&logLock);
        logRing.earlyPanics = 0;
        logAddUnlocked(LOG_RESTART, &d, 1);
        critical_section_exit(&logLock);
    }, nullptr);
    events.registerEventHandler(GP_EVENT_SYSTEM_ERROR, [](GPEvent *e) {
        logText(LOG_ERROR, static_cast<GPSystemErrorEvent *>(e)->errorMessage.c_str());
    }, nullptr);
    for (GPEventType type : { GP_EVENT_USBHOST_MOUNT, GP_EVENT_USBHOST_UNMOUNT }) {
        events.registerEventHandler(type, [](GPEvent *e) {
            const GPUSBHostEvent *p = static_cast<GPUSBHostEvent *>(e);
            const uint8_t d[6] = { e->eventType() == GP_EVENT_USBHOST_MOUNT, p->deviceAddress,
                                   (uint8_t)(p->vendorID & 0xff), (uint8_t)(p->vendorID >> 8),
                                   (uint8_t)(p->productID & 0xff), (uint8_t)(p->productID >> 8) };
            logAdd(LOG_USB_HOST, d, sizeof(d));
        }, nullptr);
    }
}

// The HardFault handler: keep the core, its last step, PC, LR and xPSR, then reboot as after a panic
extern "C" void __attribute__((used, noreturn)) gamepadiiFault(const uint32_t *frame) {
    const uint8_t core = get_core_num();
    uint8_t d[18] = { core, 0 };
    put16(d + 2, steps.step[core]);
    put32(d + 4, frame[6]);
    put32(d + 8, frame[5]);
    put32(d + 12, frame[7]);
    put16(d + 16, steps.step[core ^ 1]);
    logAddUnlocked(LOG_FAULT, d, sizeof(d));
    if (to_ms_since_boot(get_absolute_time()) < LOG_EARLY_MS) logRing.earlyPanics++;
    else logRing.earlyPanics = 0;
    logRing.check = logCheck();
    if (logRing.earlyPanics >= LOG_EARLY_MAX) for (;;) __wfi();
    watchdog_reboot(0, 0, 0);
    for (;;) __wfi();
}

extern "C" void __attribute__((naked)) isr_hardfault() {
    asm volatile(
        "movs r0, #4\n"
        "mov r1, lr\n"
        "tst r0, r1\n"
        "beq 1f\n"
        "mrs r0, psp\n"
        "b 2f\n"
        "1: mrs r0, msp\n"
        "2: ldr r1, =gamepadiiFault\n"
        "bx r1\n"
        ".ltorg\n");
}

// PICO_PANIC_FUNCTION: keep the message, then reboot. A pad that panics again and again soon after boot halts as before.
extern "C" void __attribute__((noreturn)) gamepadiiPanic(const char *fmt, ...) {
    char text[LOG_DATA + 1] = "panic";
    if (fmt) {
        va_list args;
        va_start(args, fmt);
        vsnprintf(text, sizeof(text), fmt, args);
        va_end(args);
    }
    // No lock: the other core can hold it, and this core does not come back
    logAddUnlocked(LOG_PANIC, text, strnlen(text, LOG_DATA));
    if (to_ms_since_boot(get_absolute_time()) < LOG_EARLY_MS) logRing.earlyPanics++;
    else logRing.earlyPanics = 0;
    logRing.check = logCheck();
    if (logRing.earlyPanics >= LOG_EARLY_MAX) _exit(1);
    watchdog_reboot(0, 0, 0);
    for (;;) __wfi();
}
