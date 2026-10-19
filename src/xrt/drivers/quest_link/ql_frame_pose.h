#pragma once
#include "xrt/xrt_defines.h"
#include <cmath>
#include <cstdint>

struct ql_frame_pose_snapshot {
    xrt_pose pose;
    uint64_t display_time;
};

// Graphics fast-path pixels are the application's original eye images: no host
// timewarp ran and slot.head_relation is deliberately identity. Other paths have
// already reprojected to the compositor's head pose and display time.
inline ql_frame_pose_snapshot
ql_rendered_frame_pose(bool graphics_fast_path, const xrt_pose& left, const xrt_pose& right,
                       const xrt_pose& compositor_head, uint64_t application_time, uint64_t compositor_time)
{
    if (!graphics_fast_path) return {compositor_head, compositor_time};
    xrt_pose head = left;
    head.position = {(left.position.x+right.position.x)*0.5f,
                     (left.position.y+right.position.y)*0.5f,
                     (left.position.z+right.position.z)*0.5f};
    const auto& a=left.orientation; const auto& b=right.orientation;
    const float sign=(a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w)<0 ? -1.0f : 1.0f;
    head.orientation={a.x+sign*b.x,a.y+sign*b.y,a.z+sign*b.z,a.w+sign*b.w};
    auto& q=head.orientation;
    const float norm=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
    if (norm>1e-6f) {q.x/=norm;q.y/=norm;q.z/=norm;q.w/=norm;}
    return {head, application_time};
}
