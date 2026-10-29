// Copyright 2022, Collabora, Ltd.
// Copyright 2022 Max Thomas
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  quest_link XRSP Pose topic packets
 * @author Max Thomas <mtinc2@gmail.com>
 * @ingroup drv_quest_link
 */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include "math/m_api.h"
#include "math/m_vec3.h"

#include "ql_xrsp_segmented_pkt.h"
#include "ql_xrsp_pose.h"
#include "ql_xrsp_hostinfo.h"
#include "ql_xrsp_types.h"
#include "ql_xrsp.h"
#include "ql_types.h"
#include "ql_utils.h"
#include "ql_passthrough_pose.h"

#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include "protos/Pose.capnp.h"

#include <atomic>
#include "os/os_time.h"


namespace {
constexpr uint64_t kRecenterHoldNs = 1000ULL * 1000 * 1000;
std::atomic<uint32_t> g_system_holds{0};
uint64_t g_system_down_ns[2] = {};
bool g_system_fired[2] = {};
std::atomic<uint64_t> g_system_down_at[2] = {}; // 0 while released (read by the app)
}

extern "C" __attribute__((visibility("default"))) float
ql_system_button_hold_progress(int index)
{
    if (index < 0 || index > 1) return -1;
    const uint64_t down = g_system_down_at[index].load();
    if (down == 0) return -1;
    const double progress = double(os_monotonic_get_ns() - down) / double(kRecenterHoldNs);
    return float(progress > 1 ? 1 : progress);
}

extern "C" __attribute__((visibility("default"))) uint32_t
ql_system_button_holds(void)
{
    return g_system_holds.load();
}

extern "C"
{

void ql_xrsp_handle_pose(struct ql_xrsp_segpkt* segpkt, struct ql_xrsp_host* host)
{
    // TODO parse segment header
    os_mutex_lock(&host->pose_mutex);

    if (!host || !host->sys || !host->sys->hmd) {
        os_mutex_unlock(&host->pose_mutex);
        return;
    }

    size_t num_words = segpkt->segs_valid[0] >> 3;

    kj::ArrayPtr<const capnp::word> dataptr[1] = {kj::arrayPtr((capnp::word*)segpkt->segs[0], num_words)};
    capnp::SegmentArrayMessageReader message(kj::arrayPtr(dataptr, 1));

    PayloadPose::Reader pose;

    try
    {
        pose = message.getRoot<PayloadPose>();
    }
    catch(...)
    {
        os_mutex_unlock(&host->pose_mutex);
        return;
    }

    OvrPoseF::Reader headsetPose = pose.getHeadset();
    int64_t hmd_pose_ns = xrsp_ts_ns_from_target(host, headsetPose.getTimestamp());

    for (PoseTrackedController::Reader controller: pose.getControllers()) {
        int idx = 1;
        if (!(controller.getFeatures() & OVR_TOUCH_FEAT_RIGHT)) {
            idx = 0;
        }

        struct ql_controller* ctrl = host->sys->controllers[idx];
        if (!ctrl) {
            continue;
        }
        OvrPoseF::Reader controllerPose = controller.getPose();

        ctrl->pose.position.x = controllerPose.getPosX();
        ctrl->pose.position.y = controllerPose.getPosY();
        ctrl->pose.position.z = controllerPose.getPosZ();

        ctrl->pose.orientation.x = controllerPose.getQuatX();
        ctrl->pose.orientation.y = controllerPose.getQuatY();
        ctrl->pose.orientation.z = controllerPose.getQuatZ();
        ctrl->pose.orientation.w = controllerPose.getQuatW();

        ctrl->vel = {controllerPose.getLinVelX(), controllerPose.getLinVelY(), controllerPose.getLinVelZ()};
        ctrl->acc = {controllerPose.getLinAccX(), controllerPose.getLinAccY(), controllerPose.getLinAccZ()};
        ctrl->angvel = {controllerPose.getAngVelX(), controllerPose.getAngVelY(), controllerPose.getAngVelZ()};
        ctrl->angacc = {controllerPose.getAngAccX(), controllerPose.getAngAccY(), controllerPose.getAngAccZ()};

        uint64_t ctrl_ts = controllerPose.getTimestamp();
        int64_t pose_ns = ctrl_ts ? xrsp_ts_ns_from_target(host, ctrl_ts) : hmd_pose_ns;
        ctrl->pose_ns = pose_ns;

        uint32_t features_raw = controller.getFeatures();
        uint32_t prev_buttons = ctrl->buttons;
        bool first_pkt = (!ctrl->feat_2 && !ctrl->features && !ctrl->battery);

        ctrl->features = features_raw & 0xFF;
        ctrl->battery = (features_raw >> 8) & 0x7F;
        ctrl->feat_2 = (features_raw >> 15);
        ctrl->buttons = controller.getButtons();
        ctrl->capacitance = controller.getCapacitance();
        ctrl->joystick_x = controller.getStickX();
        ctrl->joystick_y = controller.getStickY();
        ctrl->grip_z = controller.getGripZ();
        ctrl->trigger_z = controller.getTriggerZ();
        ctrl->stylus_pressure = controller.getStylusPressure();

        if (idx >= 0 && idx < 2) {
            const bool down = (ctrl->buttons & OVR_TOUCH_BTN_SYSTEM) != 0;
            const uint64_t now = os_monotonic_get_ns();
            if (!down) g_system_down_at[idx] = 0;
            if (down && !(prev_buttons & OVR_TOUCH_BTN_SYSTEM)) {
                g_system_down_ns[idx] = now;
                g_system_down_at[idx] = now;
                g_system_fired[idx] = false;
            } else if (down && !g_system_fired[idx] && now - g_system_down_ns[idx] >= kRecenterHoldNs) {
                g_system_fired[idx] = true;
                g_system_holds++;
                fprintf(stderr, "INFO [ql_xrsp_pose] Controller[%d] system button held 1 s: recenter\n", idx);
            }
        }

        if (first_pkt || ctrl->buttons != prev_buttons) {
            fprintf(stderr, "INFO [ql_xrsp_pose] Controller[%d] feat=0x%x btns=0x%x cap=0x%x trig=%.2f grip=%.2f stick=(%.2f, %.2f) pos=(%.2f, %.2f, %.2f)\n",
                    idx, features_raw, ctrl->buttons, ctrl->capacitance,
                    ctrl->trigger_z, ctrl->grip_z, ctrl->joystick_x, ctrl->joystick_y,
                    ctrl->pose.position.x, ctrl->pose.position.y, ctrl->pose.position.z);
        }
    }
    struct ql_hmd* hmd = host->sys->hmd;
    if (!hmd) {
        os_mutex_unlock(&host->pose_mutex);
        return;
    }

    int64_t pose_ns = xrsp_ts_ns_from_target(host, headsetPose.getTimestamp());
    host->last_pose_target_ts = headsetPose.getTimestamp();
    host->is_inactive = false;


    hmd->pose_ns = pose_ns;
    hmd->pose.position.x = headsetPose.getPosX();
    hmd->pose.position.y = headsetPose.getPosY();
    hmd->pose.position.z = headsetPose.getPosZ();

    hmd->pose.orientation.x = headsetPose.getQuatX();
    hmd->pose.orientation.y = headsetPose.getQuatY();
    hmd->pose.orientation.z = headsetPose.getQuatZ();
    hmd->pose.orientation.w = headsetPose.getQuatW();

    hmd->ipd_meters = pose.getIpd();

    hmd->vel = {0.0, 0.0, 0.0};
    hmd->acc = {0.0, 0.0, 0.0};
    hmd->angvel = {0.0, 0.0, 0.0};
    hmd->angacc = {0.0, 0.0, 0.0};

    hmd->vel = {headsetPose.getLinVelX(), headsetPose.getLinVelY(), headsetPose.getLinVelZ()};
    hmd->acc = {headsetPose.getLinAccX(), headsetPose.getLinAccY(), headsetPose.getLinAccZ()};
    hmd->angvel = {headsetPose.getAngVelX(), headsetPose.getAngVelY(), headsetPose.getAngVelZ()};
    hmd->angacc = {headsetPose.getAngAccX(), headsetPose.getAngAccY(), headsetPose.getAngAccZ()};
    ql_passthrough_pose_push(pose_ns, &hmd->pose, &hmd->vel, &hmd->angvel);

    // TODO: how is this even calculated??
    // Quest 2:
    // 58mm (0.057928182) angle_left -> -52deg
    // 65mm (0.065298356) angle_left -> -49deg
    // 68mm (0.068259589) angle_left -> -43deg
    float angle_calc = hmd->fov_angle_left;

    if (hmd->device_type == DEVICE_TYPE_QUEST_2)
    {
        if (hmd->ipd_meters <= 0.059)
        {
            angle_calc -= 0.0;
        }
        else if (hmd->ipd_meters <= 0.066)
        {
            angle_calc -= 3.0;
        }
        else {
            angle_calc -= 9.0;
        }
    }

    // Pull FOV information
    float next_angle_left = -angle_calc * M_PI / 180;
    float next_angle_right = angle_calc * M_PI / 180;

    if (hmd->base.hmd->distortion.fov[0].angle_left != next_angle_left || hmd->base.hmd->distortion.fov[1].angle_right != next_angle_right)
    {
        hmd->base.hmd->distortion.fov[0].angle_left = next_angle_left;
        hmd->base.hmd->distortion.fov[1].angle_right = next_angle_right;
    }

    os_mutex_unlock(&host->pose_mutex);
}

}
