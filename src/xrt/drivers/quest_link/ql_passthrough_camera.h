// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Passthrough portal camera intrinsics, shared with the in-process app
 * @ingroup drv_quest_link
 *
 * Plain C layout: the app declares the same struct and passes its size, so either
 * side can grow it later. Only lens geometry is exposed, never the calibration
 * file's device identifiers.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ql_passthrough_camera
{
    uint32_t struct_size;
    char projection_model[32];
    char distortion_model[32];
    int32_t width, height;
    int32_t projection_count;
    int32_t distortion_count;
    double projection[4];
    double distortion[12];
    double device_from_camera[16];
};


bool
ql_passthrough_copy_camera(int eye, struct ql_passthrough_camera *out);

bool
ql_passthrough_copy_float(int eye, uint64_t *io_seq, float *out, int capacity, int *out_width, int *out_height,
                          int64_t *out_capture_ns);

#ifdef __cplusplus
}
#endif
