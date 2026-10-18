/*
 * SPDX-License-Identifier: MIT
 */

#include "controlchannel.h"

#include <string.h>

#include "pico/time.h"
#include "pico/unique_id.h"

#include "BoardConfig.h"
#include "config.pb.h"
#include "enums.pb.h"
#include "drivermanager.h"
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
constexpr uint8_t PROTO_MINOR = 0;

constexpr uint16_t MAX_LEN    = 64;
constexpr uint16_t HEADER_LEN = 8;
constexpr uint16_t MAX_STRING = 63;

enum Command : uint8_t {
    CMD_HELLO          = 0x01,
    CMD_STATUS         = 0x02,
    CMD_STRING         = 0x03,
    CMD_SET_PROFILE    = 0x10,
    CMD_REBOOT         = 0x11,
    CMD_SET_INPUT_MODE = 0x12,
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
};

constexpr uint32_t CAPABILITIES = CAP_STATUS | CAP_STRING | CAP_SET_PROFILE | CAP_REBOOT | CAP_SET_INPUT_MODE;

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
};

// tud_control_xfer() keeps this pointer until the transfer completes
uint8_t buffer[MAX_LEN];

uint32_t sessionToken = 0;
bool rebootPending = false;
uint16_t lastSeq = 0;
uint8_t lastCmd = 0;
uint8_t lastResult = RES_OK;

inline void put16(uint8_t *p, uint16_t v) { p[0] = v & 0xff; p[1] = v >> 8; }
inline void put32(uint8_t *p, uint32_t v) { put16(p, v & 0xffff); put16(p + 2, v >> 16); }
inline uint16_t get16(const uint8_t *p) { return p[0] | (p[1] << 8); }
inline uint32_t get32(const uint8_t *p) { return get16(p) | ((uint32_t)get16(p + 2) << 16); }

inline uint8_t commandOf(tusb_control_request_t const *request) { return request->wIndex & 0xff; }

int commandDirection(uint8_t cmd) {
    switch (cmd) {
        case CMD_HELLO:
        case CMD_STATUS:
        case CMD_STRING:
            return TUSB_DIR_IN;
        case CMD_SET_PROFILE:
        case CMD_REBOOT:
        case CMD_SET_INPUT_MODE:
            return TUSB_DIR_OUT;
        default:
            return -1;
    }
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
    put32(buffer + 8, CAPABILITIES);
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
    Storage::getInstance().getGamepadOptions().inputMode = (InputMode)mode;
    // GAMEPAD boot mode skips the boot button scan, so a held button can't override the new mode
    EventManager::getInstance().triggerEvent(new GPStorageSaveEvent(true));
    EventManager::getInstance().triggerEvent(new GPRestartEvent(System::BootMode::GAMEPAD));
    rebootPending = true;
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
            default:
                if (request->wLength < HEADER_LEN) {
                    lastCmd = cmd; lastSeq = 0; lastResult = RES_BAD_LENGTH;
                    return false;
                }
                return tud_control_xfer(rhport, request, buffer, request->wLength);
        }
        return tud_control_xfer(rhport, request, buffer, len);
    }

    if (stage == CONTROL_STAGE_DATA && request->bmRequestType_bit.direction == TUSB_DIR_OUT) {
        lastCmd = cmd;
        lastSeq = get16(buffer + 4);
        lastResult = runCommand(cmd, request->wLength);
        return lastResult == RES_OK;
    }

    return true;
}
