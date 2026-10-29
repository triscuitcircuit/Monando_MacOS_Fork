// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Head pose history for the passthrough portal
 * @ingroup drv_quest_link

 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "xrt/xrt_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

void
ql_passthrough_pose_push(int64_t timestamp_ns, const struct xrt_pose *pose, const struct xrt_vec3 *linear_velocity,
                         const struct xrt_vec3 *angular_velocity);

void
ql_passthrough_pose_reset(void);


bool
ql_passthrough_head_pose_at(int64_t timestamp_ns, float out_pose[7]);

bool
ql_passthrough_head_latest(int64_t *out_timestamp_ns, float out_pose[7], float *out_angular_speed);

#define QL_PASSTHROUGH_HAND_JOINTS 26

void
ql_passthrough_hand_push(int hand, int64_t timestamp_ns, const float joints[QL_PASSTHROUGH_HAND_JOINTS * 7]);


bool
ql_passthrough_hand_joints_at(int hand, int64_t timestamp_ns, float out_joints[QL_PASSTHROUGH_HAND_JOINTS * 7]);

#ifdef __cplusplus
}
#endif
