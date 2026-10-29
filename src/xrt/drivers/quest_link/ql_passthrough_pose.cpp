// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Head pose history for the passthrough portal
 * @ingroup drv_quest_link
 */

#include "ql_passthrough_pose.h"

#include <algorithm>
#include <cmath>
#include <mutex>

#include "math/m_relation_history.h"

namespace {
std::mutex g_mutex;
m_relation_history *g_history = nullptr;
float g_angular_speed = 0;

constexpr int kHandHistory = 48;
constexpr int kHandFloats = QL_PASSTHROUGH_HAND_JOINTS * 7;
constexpr int64_t kHandMaxGapNs = 100 * 1000000LL;
struct HandSample
{
    int64_t ns;
    float joints[kHandFloats];
};
struct HandHistory
{
    HandSample samples[kHandHistory];
    int count = 0, next = 0;
};
HandHistory g_hands[2];

void
write_pose(const xrt_pose &pose, float out[7])
{
    out[0] = pose.position.x;
    out[1] = pose.position.y;
    out[2] = pose.position.z;
    out[3] = pose.orientation.x;
    out[4] = pose.orientation.y;
    out[5] = pose.orientation.z;
    out[6] = pose.orientation.w;
}
} // namespace

extern "C" void
ql_passthrough_pose_push(int64_t timestamp_ns, const struct xrt_pose *pose, const struct xrt_vec3 *linear_velocity,
                         const struct xrt_vec3 *angular_velocity)
{
    if (timestamp_ns <= 0 || pose == nullptr) return;
    xrt_space_relation relation = {};
    relation.pose = *pose;
    relation.relation_flags = (xrt_space_relation_flags)(
        XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
        XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
    float speed = 0;
    if (linear_velocity && std::isfinite(linear_velocity->x) && std::isfinite(linear_velocity->y) &&
        std::isfinite(linear_velocity->z)) {
        relation.linear_velocity = *linear_velocity;
        relation.relation_flags =
            (xrt_space_relation_flags)(relation.relation_flags | XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT);
    }
    if (angular_velocity && std::isfinite(angular_velocity->x) && std::isfinite(angular_velocity->y) &&
        std::isfinite(angular_velocity->z)) {
        relation.angular_velocity = *angular_velocity;
        relation.relation_flags =
            (xrt_space_relation_flags)(relation.relation_flags | XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
        speed = std::sqrt(angular_velocity->x * angular_velocity->x + angular_velocity->y * angular_velocity->y +
                          angular_velocity->z * angular_velocity->z);
    }
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_history == nullptr) m_relation_history_create(&g_history);
    m_relation_history_push(g_history, &relation, (uint64_t)timestamp_ns);
    g_angular_speed = speed;
}

extern "C" void
ql_passthrough_pose_reset(void)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_history != nullptr) m_relation_history_destroy(&g_history);
    g_angular_speed = 0;
    g_hands[0] = HandHistory{};
    g_hands[1] = HandHistory{};
}

extern "C" void
ql_passthrough_hand_push(int hand, int64_t timestamp_ns, const float joints[QL_PASSTHROUGH_HAND_JOINTS * 7])
{
    if (hand < 0 || hand > 1 || timestamp_ns <= 0 || joints == nullptr) return;
    for (int i = 0; i < kHandFloats; ++i)
        if (!std::isfinite(joints[i])) return;
    std::lock_guard<std::mutex> lock(g_mutex);
    HandHistory &h = g_hands[hand];
    if (h.count > 0) {
        const HandSample &last = h.samples[(h.next + kHandHistory - 1) % kHandHistory];
        if (timestamp_ns <= last.ns) return;
    }
    HandSample &slot = h.samples[h.next];
    slot.ns = timestamp_ns;
    std::copy(joints, joints + kHandFloats, slot.joints);
    h.next = (h.next + 1) % kHandHistory;
    h.count = std::min(h.count + 1, kHandHistory);
}

extern "C" __attribute__((visibility("default"))) bool
ql_passthrough_hand_joints_at(int hand, int64_t timestamp_ns, float out_joints[QL_PASSTHROUGH_HAND_JOINTS * 7])
{
    if (hand < 0 || hand > 1 || timestamp_ns <= 0 || out_joints == nullptr) return false;
    std::lock_guard<std::mutex> lock(g_mutex);
    const HandHistory &h = g_hands[hand];
    const HandSample *before = nullptr, *after = nullptr;
    for (int i = 0; i < h.count; ++i) {
        const HandSample &s = h.samples[(h.next - h.count + i + kHandHistory) % kHandHistory];
        if (s.ns <= timestamp_ns) before = &s;
        if (s.ns >= timestamp_ns) { after = &s; break; }
    }
    if (before == nullptr || after == nullptr || after->ns - before->ns > kHandMaxGapNs) return false;
    const float t = after->ns == before->ns ? 0.0f : float(timestamp_ns - before->ns) / float(after->ns - before->ns);
    for (int j = 0; j < QL_PASSTHROUGH_HAND_JOINTS; ++j) {
        const float *a = before->joints + j * 7, *b = after->joints + j * 7;
        float *o = out_joints + j * 7;
        for (int k = 0; k < 3; ++k) o[k] = a[k] + (b[k] - a[k]) * t;
        const float sign = a[3] * b[3] + a[4] * b[4] + a[5] * b[5] + a[6] * b[6] < 0 ? -1.0f : 1.0f;
        float len = 0;
        for (int k = 3; k < 7; ++k) { o[k] = a[k] + (sign * b[k] - a[k]) * t; len += o[k] * o[k]; }
        len = std::sqrt(len);
        for (int k = 3; k < 7; ++k) o[k] = len > 1e-6f ? o[k] / len : a[k];
    }
    return true;
}

extern "C" __attribute__((visibility("default"))) bool
ql_passthrough_head_pose_at(int64_t timestamp_ns, float out_pose[7])
{
    if (timestamp_ns <= 0 || out_pose == nullptr) return false;
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_history == nullptr) return false;
    xrt_space_relation relation = {};
    const m_relation_history_result result = m_relation_history_get(g_history, (uint64_t)timestamp_ns, &relation);
    if (result != M_RELATION_HISTORY_RESULT_EXACT && result != M_RELATION_HISTORY_RESULT_INTERPOLATED) return false;
    write_pose(relation.pose, out_pose);
    return true;
}

extern "C" __attribute__((visibility("default"))) bool
ql_passthrough_head_latest(int64_t *out_timestamp_ns, float out_pose[7], float *out_angular_speed)
{
    if (out_timestamp_ns == nullptr || out_pose == nullptr || out_angular_speed == nullptr) return false;
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_history == nullptr) return false;
    uint64_t timestamp = 0;
    xrt_space_relation relation = {};
    if (!m_relation_history_get_latest(g_history, &timestamp, &relation)) return false;
    *out_timestamp_ns = (int64_t)timestamp;
    write_pose(relation.pose, out_pose);
    *out_angular_speed = g_angular_speed;
    return true;
}
