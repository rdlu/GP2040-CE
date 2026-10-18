/*
 * SPDX-License-Identifier: MIT
 */

#ifndef _CONTROLCHANNEL_H_
#define _CONTROLCHANNEL_H_

#include <stdint.h>
#include "tusb.h"

namespace ControlChannel {
    bool matches(tusb_control_request_t const *request);
    bool handle(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request);
}

#endif
