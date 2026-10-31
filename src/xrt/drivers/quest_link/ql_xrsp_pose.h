// Copyright 2022, Collabora, Ltd.
// Copyright 2022 Max Thomas
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  quest_link XRSP Pose topic packets
 * @author Max Thomas <mtinc2@gmail.com>
 * @ingroup drv_quest_link
 */

#pragma once

#include <stdlib.h>

#include "ql_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void ql_controller_battery_reset(void);

void ql_xrsp_handle_pose(struct ql_xrsp_segpkt* segpkt, struct ql_xrsp_host* host);

uint32_t
ql_system_button_holds(void);

float
ql_system_button_hold_progress(int index);

#ifdef __cplusplus
}
#endif
