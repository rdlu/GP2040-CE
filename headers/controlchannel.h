/*
 * SPDX-License-Identifier: MIT
 */

#ifndef _CONTROLCHANNEL_H_
#define _CONTROLCHANNEL_H_

#include <stdint.h>
#include "tusb.h"
#include "enums.pb.h"

class AnimationStation;

namespace ControlChannel {
    bool matches(tusb_control_request_t const *request);
    bool handle(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request);

    // Called on core1 by NeoPicoLEDAddon, which owns AnimationStation
    void lightsSetup();
    void lightsProcess(AnimationStation &as);

    // Called once at boot: the mode of a one-boot SET_INPUT_MODE, if one was sent
    bool takeOneBootMode(InputMode &mode);

    // The last step of each core survives a watchdog reboot; a loop step on core 0 feeds the watchdog
    enum Step : uint16_t {
        STEP_LOOP = 1,
        STEP_REINIT,
        STEP_READ,
        STEP_PROCESS,
        STEP_HOTKEY,
        STEP_DRIVER,
        STEP_USB,
        STEP_SAVE_REBOOT,
        STEP_REBOOT_LOCKOUT,
        STEP_PREPROCESS_ADDON = 0x100, // + the add-on index
        STEP_PROCESS_ADDON    = 0x200, // + the add-on index
        STEP_AUX_DRIVER       = 0x300,
    };
    void step(uint16_t step);
    void watchdogStart();
}

#ifdef GP2040_CONTROL_CHANNEL
#define CONTROL_CHANNEL_STEP(s) ControlChannel::step(s)
#else
#define CONTROL_CHANNEL_STEP(s) ((void)0)
#endif

#endif
