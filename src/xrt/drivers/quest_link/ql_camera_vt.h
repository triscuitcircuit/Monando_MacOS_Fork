// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  quest_link camera HEVC decode test with VideoToolbox (diagnostic only)
 * @ingroup drv_quest_link
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

struct ql_camera_vt;


struct ql_camera_vt *
ql_camera_vt_create(bool keep_frames);


void
ql_camera_vt_submit(struct ql_camera_vt *vt, int stream, const uint8_t *data, size_t len, uint64_t frame_id);


void
ql_camera_vt_mark_gap(struct ql_camera_vt *vt, int stream);

bool
ql_camera_vt_stalled(struct ql_camera_vt *vt);

void
ql_camera_vt_log_summary(struct ql_camera_vt *vt);

// Releases the held portal frames; the decoder keeps publishing new ones.
void
ql_camera_vt_clear_frames(void);

// Waits for in-flight frames, then tears down the sessions.
void
ql_camera_vt_destroy(struct ql_camera_vt *vt);

#ifdef __cplusplus
extern "C" {
#endif

bool
ql_passthrough_copy_latest(uint64_t *io_seq, void **out_left, void **out_right);
bool
ql_passthrough_copy_latest_ex(uint64_t *io_seq, void **out_left, void **out_right, int64_t *out_capture_ns);
#ifdef __cplusplus
}
#endif
