// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  quest_link XRSP camera stream metadata (diagnostic only)
 * @ingroup drv_quest_link
 */

#pragma once

#include "ql_types.h"

#ifdef __cplusplus
extern "C" {
#endif


void ql_xrsp_camera_consume(struct ql_xrsp_host *host, struct ql_xrsp_topic_pkt *pkt);
void ql_xrsp_camera_log_summary(struct ql_xrsp_host *host);

bool ql_xrsp_camera_decoder_stalled(struct ql_xrsp_host *host);
void ql_xrsp_camera_destroy(struct ql_xrsp_host *host);
void ql_xrsp_camera_clear_frames(void);

#ifdef __cplusplus
}
#endif
