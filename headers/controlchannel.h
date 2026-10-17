/*
 * SPDX-License-Identifier: MIT
 *
 * Optional USB vendor control channel for host tools (compile-time option
 * GP2040_CONTROL_CHANNEL). Device-recipient vendor requests on EP0 with
 * bRequest 0x47 and wIndex 0x50nn are answered here; every other vendor
 * request goes to the active driver unchanged. No descriptors are added or
 * changed in any mode.
 */

#ifndef _CONTROLCHANNEL_H_
#define _CONTROLCHANNEL_H_

#include <stdint.h>
#include "tusb.h"

namespace ControlChannel {
    // True when the request is addressed to the channel. Stateless, so it can
    // be re-run on the stored request in the DATA and ACK stages.
    bool matches(tusb_control_request_t const *request);

    // Handles a matched request in any stage. false STALLs EP0.
    bool handle(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request);
}

#endif
