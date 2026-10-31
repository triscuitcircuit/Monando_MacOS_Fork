// Copyright 2022, Collabora, Ltd.
// Copyright 2022 Max Thomas
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to quest_link XRSP protocol.
 * @author Max Thomas <mtinc2@gmail.com>
 * @ingroup drv_quest_link
 */

#include <libusb.h>
#include <stdio.h>

#include "util/u_debug.h"
#include "util/u_trace_marker.h"
#include "math/m_api.h"
#include "math/m_vec3.h"

#include "os/os_time.h"

#include "ql_system.h"
#include "ql_hmd.h"
#include "ql_xrsp.h"
#include "ql_xrsp_hostinfo.h"
#include "ql_xrsp_topic.h"
#include "ql_xrsp_types.h"
#include "ql_utils.h"
#include "ql_xrsp_pose.h"
#include "ql_xrsp_ipc.h"
#include "ql_xrsp_hands.h"
#include "ql_xrsp_logging.h"
#include "ql_xrsp_segmented_pkt.h"
#include "ql_invite_ok.h"
#include "ql_xrsp_camera.h"

#include <algorithm>
#include <atomic>
#include <mutex>
#include <vector>

#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <capnp/any.h>
#include "protos/Audio.capnp.h"
#include "ql_audio_buffer.h"
#include "protos/HostInfo.capnp.h"
#include "protos/Slice.capnp.h"
#include "protos/Haptic.capnp.h"
#include "protos/RuntimeIPC.capnp.h"
#include "protos/Mesh.capnp.h"

DEBUG_GET_ONCE_NUM_OPTION(force_fps, "QL_OVERRIDE_FPS", -1)
DEBUG_GET_ONCE_NUM_OPTION(force_w, "QL_OVERRIDE_FB_W", -1)
DEBUG_GET_ONCE_NUM_OPTION(force_h, "QL_OVERRIDE_FB_H", -1)
DEBUG_GET_ONCE_FLOAT_OPTION(force_scale, "QL_OVERRIDE_SCALE", 0.0)
DEBUG_GET_ONCE_BOOL_OPTION(audio_probe, "QL_AUDIO_PROBE", false)
// Diagnostic: a quiet 2 s 440 Hz tone to the headset speakers once frames start flowing.
DEBUG_GET_ONCE_BOOL_OPTION(audio_test_tone, "QL_AUDIO_TEST_TONE", false)
DEBUG_GET_ONCE_BOOL_OPTION(link_probe, "QL_LINK_PROBE", false)
DEBUG_GET_ONCE_BOOL_OPTION(camera_probe, "QL_CAMERA_PROBE", false)
DEBUG_GET_ONCE_OPTION(camera_ok_formats, "QL_CAMERA_OK_FORMATS", NULL)
DEBUG_GET_ONCE_NUM_OPTION(camera_drop_test_window, "QL_CAMERA_DROP_TEST", 0)
DEBUG_GET_ONCE_NUM_OPTION(usb_read_size, "QL_USB_READ_SIZE", 0x400)
DEBUG_GET_ONCE_NUM_OPTION(camera_probe_seconds, "QL_CAMERA_PROBE_SECONDS", 0)

static bool g_passthrough_requested = false; // __atomic; written by the app's thread

extern "C" __attribute__((visibility("default"))) void
ql_passthrough_set_requested(bool requested)
{
    __atomic_store_n(&g_passthrough_requested, requested, __ATOMIC_RELEASE);
}

static bool xrsp_camera_enabled(const struct ql_xrsp_host *host)
{
    return debug_get_bool_option_camera_probe() || host->passthrough;
}

static void *
ql_xrsp_read_thread(void *ptr);
static void *
ql_xrsp_write_thread(void *ptr);
static void xrsp_reset_echo(struct ql_xrsp_host *host);
static bool xrsp_read_usb(struct ql_xrsp_host *host);
static void xrsp_send_usb(struct ql_xrsp_host *host, const uint8_t* data, int32_t data_size);
static void xrsp_send_to_topic_raw(struct ql_xrsp_host *host, uint8_t topic, const uint8_t* data, int32_t data_size);

static void xrsp_flush_stream(struct ql_xrsp_host *host, int64_t target_ns, int index, int slice_idx);
static void xrsp_start_encode(struct ql_xrsp_host *host, int64_t target_ns, int index, int slice_idx, const struct xrt_pose* rendered_pose);
static void xrsp_send_csd(struct ql_xrsp_host *host, const uint8_t* data, size_t data_len, int index, int slice_idx);
static void xrsp_send_idr(struct ql_xrsp_host *host, const uint8_t* data, size_t data_len, int index, int slice_idx);
static void xrsp_send_video(struct ql_xrsp_host *host, int index, int slice_idx, int frame_idx, int64_t frame_started_ns, const uint8_t* csd_dat, size_t csd_len,
                            const uint8_t* video_dat, size_t video_len, int blit_y_pos);
static void xrsp_init_session_bye(struct ql_xrsp_host *host);
int ql_xrsp_usb_init(struct ql_xrsp_host* host, bool do_reset);
static void xrsp_send_mesh(struct ql_xrsp_host *host);
static void xrsp_send_audio_control(struct ql_xrsp_host *host, uint16_t a, uint16_t b, uint32_t c, float d, float e);
static void xrsp_send_audio(struct ql_xrsp_host *host, const float *interleaved, uint32_t frames, int64_t host_ns);
static void xrsp_audio_test_tone_tick(struct ql_xrsp_host *host);
static void xrsp_audio_drain(struct ql_xrsp_host *host);
static void xrsp_send_input_control(struct ql_xrsp_host *host, uint16_t a, uint16_t b, uint32_t c, float d, float e);

int ql_xrsp_host_create(struct ql_xrsp_host* host, uint16_t vid, uint16_t pid, int if_num)
{
    int ret;

    ql_controller_battery_reset();
    *host = (struct ql_xrsp_host){0};
    host->if_num = if_num;
    host->vid = vid;
    host->pid = pid;

    if (debug_get_bool_option_link_probe()) {
        QUEST_LINK_INFO("LINK_PROBE enabled: traffic counters (no payload dumps)");
    }

    host->num_slices = QL_NUM_SLICES;

    host->ready_to_send_frames = false;
    host->sent_first_frame = false;
    host->stream_read_idx = 0;
    host->stream_write_idx = 0;
    for (int i = 0; i < QL_SWAPCHAIN_DEPTH; i++)
    {
        for (int j = 0; j < QL_NUM_SLICES; j++)
        {
            host->csd_stream[QL_IDX_SLICE(j, i)] = (uint8_t*)malloc(0x1000000);
            host->idr_stream[QL_IDX_SLICE(j, i)] = (uint8_t*)malloc(0x1000000);
            host->csd_stream_len[QL_IDX_SLICE(j, i)] = 0;
            host->idr_stream_len[QL_IDX_SLICE(j, i)] = 0;

            host->stream_started_ns[QL_IDX_SLICE(j, i)] = 0;
            host->encode_started_ns[QL_IDX_SLICE(j, i)] = 0;
            host->encode_done_ns[QL_IDX_SLICE(j, i)] = 0;
            host->encode_duration_ns[QL_IDX_SLICE(j, i)] = 0;
            host->tx_started_ns[QL_IDX_SLICE(j, i)] = 0;
            host->tx_done_ns[QL_IDX_SLICE(j, i)] = 0;
            host->tx_duration_ns[QL_IDX_SLICE(j, i)] = 0;

            ret = os_mutex_init(&host->stream_mutex[QL_IDX_SLICE(j, i)]);
            if (ret != 0) {
                QUEST_LINK_ERROR("Failed to init usb mutex");
                goto cleanup;
            }
        }
    }

    host->frame_idx = 0;
    ret = os_mutex_init(&host->usb_mutex);
    if (ret != 0) {
        QUEST_LINK_ERROR("Failed to init usb mutex");
        goto cleanup;
    }

    ret = os_mutex_init(&host->pose_mutex);
    if (ret != 0) {
        QUEST_LINK_ERROR("Failed to init pose mutex");
        goto cleanup;
    }


    //
    // Thread and other state.
    ret = os_thread_helper_init(&host->read_thread);
    if (ret != 0) {
        QUEST_LINK_ERROR("Failed to init packet read processing thread");
        goto cleanup;
    }

    //
    // Thread and other state.
    ret = os_thread_helper_init(&host->write_thread);
    if (ret != 0) {
        QUEST_LINK_ERROR("Failed to init packet write processing thread");
        goto cleanup;
    }

    host->dev = NULL;

    ret = libusb_init(&host->ctx);
    if (ret < 0) {
        QUEST_LINK_ERROR("Failed libusb_init");
        goto cleanup;
    }

    ret = ql_xrsp_usb_init(host, false);
    if (ret != 0) {
        goto cleanup;
    }

    //QUEST_LINK_INFO("Endpoints %x %x", host->ep_out, host->ep_in);

    host->pairing_state = PAIRINGSTATE_WAIT_FIRST;
    host->start_ns = os_monotonic_get_ns();
    {
        const char* passthrough = getenv("QL_PASSTHROUGH");
        host->passthrough = passthrough != NULL && *passthrough != '\0' && strcmp(passthrough, "0") != 0;
        if (host->passthrough) QUEST_LINK_INFO("PASSTHROUGH allowed: camera streams only while the portal is open");
    }
    host->paired_ns = os_monotonic_get_ns()*2;
    host->last_read_ns = 0;
    xrsp_reset_echo(host);

    host->start_encode = xrsp_start_encode;
    host->send_csd = xrsp_send_csd;
    host->send_idr = xrsp_send_idr;
    host->flush_stream = xrsp_flush_stream;

    host->client_id = 0x4a60dcca;
    host->session_idx = 3;
    host->runtime_connected = false;
    host->bodyapi_connected = false;
    host->eyetrack_connected = false;

    // Start the packet reading thread
    ret = os_thread_helper_start(&host->read_thread, ql_xrsp_read_thread, host);
    if (ret != 0) {
        QUEST_LINK_ERROR("Failed to start packet processing thread");
        goto cleanup;
    }

    // Start the packet reading thread
    ret = os_thread_helper_start(&host->write_thread, ql_xrsp_write_thread, host);
    if (ret != 0) {
        QUEST_LINK_ERROR("Failed to start packet processing thread");
        goto cleanup;
    }

    return 0;
cleanup:
    return -1;
}

int ql_xrsp_usb_init(struct ql_xrsp_host* host, bool do_reset)
{
    int ret;
    const struct libusb_interface_descriptor* pIf = NULL;
    struct libusb_config_descriptor *config = NULL;
    libusb_device *usb_dev = NULL;

    QUEST_LINK_INFO("(Re)initializing Quest Link USB device...");

    host->last_init_ns = os_monotonic_get_ns();

    os_mutex_lock(&host->usb_mutex);

    if (host->dev) {
        libusb_close(host->dev);
    }

    host->usb_speed = LIBUSB_SPEED_LOW;
    host->usb_valid = false;
    host->pairing_state = PAIRINGSTATE_WAIT_FIRST;
    host->ready_to_send_frames = false;
    host->sent_first_frame = false;

    host->dev = libusb_open_device_with_vid_pid(host->ctx, host->vid, host->pid);
    if (host->dev == NULL) {
        QUEST_LINK_ERROR("Failed initial libusb_open_device_with_vid_pid");
        QUEST_LINK_ERROR("libusb error: %s", libusb_strerror((libusb_error)ret));
        goto cleanup;
    }

    if (do_reset) {
        QUEST_LINK_INFO("Reset?");
        ret = libusb_reset_device(host->dev);
        if (ret == LIBUSB_ERROR_NOT_FOUND) {
            // We're reconnecting anyhow.
            QUEST_LINK_ERROR("libusb error: %s", libusb_strerror((libusb_error)ret));
            QUEST_LINK_INFO("Device needs reconnect...");
        }
        else if (ret != LIBUSB_SUCCESS) {
            QUEST_LINK_ERROR("Failed libusb_reset_device");
            QUEST_LINK_ERROR("libusb error: %s", libusb_strerror((libusb_error)ret));
            goto cleanup;
        }
        else {
            libusb_close(host->dev);
        }



        QUEST_LINK_INFO("Reset done?");

        for (int i = 0; i < 10; i++)
        {
            // Re-initialize the device
            host->dev = libusb_open_device_with_vid_pid(host->ctx, host->vid, host->pid);
            if (host->dev) break;

            os_nanosleep(U_TIME_1MS_IN_NS * 500);
        }

        if (host->dev == NULL) {
            QUEST_LINK_ERROR("Failed post-reset libusb_open_device_with_vid_pid");
            QUEST_LINK_ERROR("libusb error: %s", libusb_strerror((libusb_error)ret));
            goto cleanup;
        }
    }

    ret = libusb_claim_interface(host->dev, host->if_num);
    if (ret < 0) {
        QUEST_LINK_ERROR("Failed libusb_claim_interface");
        QUEST_LINK_ERROR("libusb error: %s", libusb_strerror((libusb_error)ret));

        // Reset, there's probably something weird.
        libusb_reset_device(host->dev);
        goto cleanup;
    }

    //libusb_set_interface_alt_setting(host->dev, host->if_num, 1);

    usb_dev = libusb_get_device(host->dev);
    ret = libusb_get_active_config_descriptor(usb_dev, &config);
    if (ret < 0 || !config) {
        QUEST_LINK_ERROR("Failed libusb_get_active_config_descriptor");
        QUEST_LINK_ERROR("libusb error: %s", libusb_strerror((libusb_error)ret));
        goto cleanup;
    }


    for (uint8_t i = 0; i < config->bNumInterfaces; i++) {
        if (pIf) break;
        const struct libusb_interface* pIfAlts = &config->interface[i];
        for (uint8_t j = 0; j < pIfAlts->num_altsetting; j++) {
            const struct libusb_interface_descriptor* pIfIt = &pIfAlts->altsetting[j];
            if (pIfIt->bInterfaceNumber == host->if_num) {
                pIf = pIfIt;
                break;
            }
        }
    }

    host->ep_out = 0;
    host->ep_in = 0;
    for (uint8_t i = 0; i < pIf->bNumEndpoints; i++) {
        const struct libusb_endpoint_descriptor* pEp = &pIf->endpoint[i];

        if (!host->ep_out && !(pEp->bEndpointAddress & LIBUSB_ENDPOINT_IN)) {
            host->ep_out = pEp->bEndpointAddress;
        }
        else if (!host->ep_in && (pEp->bEndpointAddress & LIBUSB_ENDPOINT_IN)) {
            host->ep_in = pEp->bEndpointAddress;
        }
    }

    host->usb_slow_cable = false;
    host->usb_speed = libusb_get_device_speed(usb_dev);
    switch (host->usb_speed)
    {
    case LIBUSB_SPEED_LOW:
        host->usb_slow_cable = true;
        QUEST_LINK_ERROR("Headset is operating at 1.5Mbit/s");
        break;
    case LIBUSB_SPEED_FULL:
        host->usb_slow_cable = true;
        QUEST_LINK_ERROR("Headset is operating at 12Mbit/s");
        break;
    case LIBUSB_SPEED_HIGH:
        host->usb_slow_cable = true;
        QUEST_LINK_ERROR("Headset is operating at 480Mbit/s");
        break;
    case LIBUSB_SPEED_SUPER:
        QUEST_LINK_INFO("Headset is operating at 5000Mbit/s");
        break;
    case LIBUSB_SPEED_SUPER_PLUS:
        QUEST_LINK_INFO("Headset is operating at 10000Mbit/s");
        break;
    default:
        host->usb_slow_cable = true;
        QUEST_LINK_ERROR("libusb_get_device_speed returned unknown value!");
        break;
    }

    libusb_clear_halt(host->dev, host->ep_in);
    libusb_clear_halt(host->dev, host->ep_out);
    libusb_clear_halt(host->dev, host->ep_in);
    libusb_clear_halt(host->dev, host->ep_out);

    host->usb_valid = true;

    os_mutex_unlock(&host->usb_mutex);

    //xrsp_init_session_bye(host);

    return 0;

cleanup:
    os_mutex_unlock(&host->usb_mutex);
    return -1;
}

void ql_xrsp_host_stop_camera(struct ql_xrsp_host* host)
{

    if (host->read_thread.initialized && __atomic_load_n(&host->camera_streaming, __ATOMIC_ACQUIRE)) {
        __atomic_store_n(&host->camera_stop_requested, true, __ATOMIC_RELEASE);
        const int64_t deadline = os_monotonic_get_ns() + 500 * U_TIME_1MS_IN_NS;
        int64_t now = os_monotonic_get_ns();
        while (now < deadline && (__atomic_load_n(&host->camera_streaming, __ATOMIC_ACQUIRE) ||
                                  now - __atomic_load_n(&host->camera_last_rx_ns, __ATOMIC_ACQUIRE) <
                                      100 * U_TIME_1MS_IN_NS)) {
            os_nanosleep(5 * U_TIME_1MS_IN_NS);
            now = os_monotonic_get_ns();
        }
        QUEST_LINK_INFO("CAMERA_PROBE teardown: camera %s before USB release",
                        now < deadline ? "traffic stopped" : "still sending after 500 ms");
    }
}

void ql_xrsp_host_destroy(struct ql_xrsp_host* host)
{
    ql_xrsp_host_stop_camera(host);
    if (host->read_thread.initialized) {
        os_thread_helper_destroy(&host->read_thread);
    }
    ql_xrsp_camera_destroy(host);
    if (host->write_thread.initialized) {
        os_thread_helper_destroy(&host->write_thread);
    }

    if (host->dev) {
        libusb_release_interface(host->dev, host->if_num);
        libusb_close(host->dev);
        host->dev = NULL;
    }
    if (host->ctx) {
        libusb_exit(host->ctx);
        host->ctx = NULL;
    }

    os_mutex_destroy(&host->pose_mutex);
    os_mutex_destroy(&host->usb_mutex);
    for (int i = 0; i < QL_SWAPCHAIN_DEPTH; i++)
    {
        for (int j = 0; j < QL_NUM_SLICES; j++)
        {
            free(host->csd_stream[QL_IDX_SLICE(j, i)]);
            free(host->idr_stream[QL_IDX_SLICE(j, i)]);
            os_mutex_destroy(&host->stream_mutex[QL_IDX_SLICE(j, i)]);
        }
    }
}

static void xrsp_flush_stream(struct ql_xrsp_host *host, int64_t target_ns, int index, int slice_idx)
{
    if (!host->ready_to_send_frames) return;

    int stream_write_idx = QL_IDX_SLICE(slice_idx, index);
    host->encode_done_ns[stream_write_idx] = xrsp_ts_ns(host);

    bool wait = false;
    os_mutex_lock(&host->stream_mutex[stream_write_idx]);

    if (host->csd_stream_len[stream_write_idx] || host->idr_stream_len[stream_write_idx]) {
        //QUEST_LINK_INFO("Write idx %x slice %x", index, slice_idx);
        host->needs_flush[stream_write_idx] = true;
        wait = true;
        host->stream_started_ns[stream_write_idx] = target_ns;

        struct ql_hmd* hmd = host->sys ? host->sys->hmd : NULL;
        struct xrt_space_relation out_head_relation;
        U_ZERO(&out_head_relation);

        host->encode_duration_ns[stream_write_idx] = host->encode_done_ns[stream_write_idx] - host->encode_started_ns[stream_write_idx];

        static int64_t last_ns = 0;
        int64_t delta = host->stream_started_ns[stream_write_idx] - last_ns;
        //QUEST_LINK_INFO("%zx -> %ffps", delta, 1000000000.0 / (double)delta);

        last_ns = target_ns;
        os_mutex_unlock(&host->stream_mutex[stream_write_idx]);
    }
    else {
        os_mutex_unlock(&host->stream_mutex[stream_write_idx]);
    }
}

static void xrsp_start_encode(struct ql_xrsp_host *host, int64_t target_ns, int index, int slice_idx, const struct xrt_pose* rendered_pose)
{
    int write_index = QL_IDX_SLICE(slice_idx, index);

    while (host->needs_flush[write_index] && os_thread_helper_is_running(&host->write_thread)) {
        os_nanosleep(U_TIME_1MS_IN_NS / 10);
    }
    if (!os_thread_helper_is_running(&host->write_thread) || !host->sys || !host->sys->hmd) {
        return;
    }
    os_mutex_lock(&host->stream_mutex[write_index]);
    host->encode_started_ns[write_index] = xrsp_ts_ns(host);

    // Slice workers use the pose captured with the rendered image, never a newer tracking sample.
    host->stream_poses[write_index] = *rendered_pose;
    host->stream_pose_ns[write_index] = target_ns;
    os_mutex_unlock(&host->stream_mutex[write_index]);
}

static void xrsp_send_csd(struct ql_xrsp_host *host, const uint8_t* data, size_t data_len, int index, int slice_idx)
{
    int write_index = QL_IDX_SLICE(slice_idx, index);
    //QUEST_LINK_INFO("CSD");
    //if (!host->ready_to_send_frames) return;
    while (host->needs_flush[write_index] && os_thread_helper_is_running(&host->write_thread)) {
        os_nanosleep(U_TIME_1MS_IN_NS / 10);
    }
    if (!os_thread_helper_is_running(&host->write_thread)) {
        return;
    }
    os_mutex_lock(&host->stream_mutex[write_index]);
    //bool success = xrsp_read_usb(host);

    //QUEST_LINK_INFO("CSD: %x into %x", data_len, host->csd_stream_len[write_index]);
    //hex_dump(data, data_len);

    if (host->csd_stream_len[write_index] + data_len < 0x1000000) {
        memcpy(host->csd_stream[write_index] + host->csd_stream_len[write_index], data, data_len);
        host->csd_stream_len[write_index] += data_len;
    }

    os_mutex_unlock(&host->stream_mutex[write_index]);
}

static void xrsp_send_idr(struct ql_xrsp_host *host, const uint8_t* data, size_t data_len, int index, int slice_idx)
{
    int write_index = QL_IDX_SLICE(slice_idx, index);

    while (host->needs_flush[write_index] && os_thread_helper_is_running(&host->write_thread)) {
        os_nanosleep(U_TIME_1MS_IN_NS / 10);
    }
    if (!os_thread_helper_is_running(&host->write_thread)) {
        return;
    }
    os_mutex_lock(&host->stream_mutex[write_index]);

    //QUEST_LINK_INFO("IDR: %x into %x for slice %x, index %x", data_len, host->idr_stream_len[write_index], slice_idx, index);

    if (host->idr_stream_len[write_index] + data_len < 0x1000000) {
        memcpy(host->idr_stream[write_index] + host->idr_stream_len[write_index], data, data_len);
        host->idr_stream_len[write_index] += data_len;
    }

    os_mutex_unlock(&host->stream_mutex[write_index]);
}

static void xrsp_send_usb(struct ql_xrsp_host *host, const uint8_t* data, int32_t data_size)
{
    //QUEST_LINK_INFO("Send to usb:");
    //hex_dump(data, data_size);

    if (!host->usb_valid) return;

    int sent_len = 0;
    int r = libusb_bulk_transfer(host->dev, host->ep_out, (uint8_t*)data, data_size, &sent_len, 1000);
    if (r != 0 || !sent_len) {
        QUEST_LINK_ERROR("Failed to send %x bytes (sent %x)", data_size, sent_len);
        QUEST_LINK_ERROR("libusb error: %s", libusb_strerror((libusb_error)r));

        if (r == LIBUSB_ERROR_NO_DEVICE || r == LIBUSB_ERROR_TIMEOUT) {
           host->usb_valid = false;
           host->pairing_state = PAIRINGSTATE_WAIT_FIRST;
        }
    }
    else {
        //QUEST_LINK_INFO("Sent %x bytes", sent_len);
    }
}

void xrsp_send_to_topic_capnp_wrapped(struct ql_xrsp_host *host, uint8_t topic, uint32_t idx, const uint8_t* data, int32_t data_size)
{
    uint32_t preamble[2] = {idx, static_cast<uint32_t>(data_size) >> 3};
    xrsp_send_to_topic(host, topic, (uint8_t*)preamble, sizeof(uint32_t) * 2);
    xrsp_send_to_topic(host, topic, data, data_size);
}

void xrsp_send_to_topic_capnp_segments(struct ql_xrsp_host *host, uint8_t topic, uint32_t idx, kj::ArrayPtr<const kj::ArrayPtr<const capnp::word>>& data)
{
    int num_segments = data.size();

    uint32_t* preamble = (uint32_t*)malloc(sizeof(uint32_t) * (num_segments + 1));
    preamble[0] = idx;
    for (int i = 0; i < num_segments; i++)
    {
        size_t packed_data_size = data[i].size()*sizeof(uint64_t);

        preamble[i+1] = static_cast<uint32_t>(packed_data_size) >> 3;
    }

    xrsp_send_to_topic(host, topic, (uint8_t*)preamble, sizeof(uint32_t) * (num_segments + 1));
    free(preamble);

    for (int i = 0; i < num_segments; i++)
    {
        uint8_t* packed_data = (uint8_t*)data[i].begin();
        size_t packed_data_size = data[i].size()*sizeof(uint64_t);

        xrsp_send_to_topic(host, topic, packed_data, packed_data_size);
    }
}

void xrsp_send_to_topic(struct ql_xrsp_host *host, uint8_t topic, const uint8_t* data, int32_t data_size)
{
    //QUEST_LINK_INFO("Send to topic %s", xrsp_topic_str(topic));
    //hex_dump(data, data_size);

    os_mutex_lock(&host->usb_mutex);

    if (!host) return;
    if (data_size <= 0) return;

    int32_t idx = 0;
    int32_t to_send = data_size;
    while (true)
    {
        if (idx >= to_send) break;

        int32_t amt = 0x3FFF8; // FFF8?
        if (idx+amt >= to_send) {
            amt = to_send - idx;
        }
        xrsp_send_to_topic_raw(host, topic, data + idx, amt);

        idx += amt;
    }
    os_mutex_unlock(&host->usb_mutex);
}

static void xrsp_send_to_topic_raw(struct ql_xrsp_host *host, uint8_t topic, const uint8_t* data, int32_t data_size)
{
    //QUEST_LINK_INFO("Send to topic raw %s", xrsp_topic_str(topic));
    //hex_dump(data, data_size);

    if (!host) return;
    //if (data_size <= 0) return;

    int32_t real_len = data_size;
    int32_t align_up_bytes = (((4+data_size) >> 2) << 2) - data_size;
    if (align_up_bytes == 4) {
        align_up_bytes = 0;
    }

    //QUEST_LINK_INFO("align up %x, %x, %x", align_up_bytes, data_size, real_len);

    // TODO place this in a fixed buffer?
    uint8_t* msg = (uint8_t*)malloc(data_size + align_up_bytes + sizeof(xrsp_topic_header) + 0x400);
    uint8_t* msg_payload = msg + sizeof(xrsp_topic_header);
    int32_t msg_size = data_size + align_up_bytes + sizeof(xrsp_topic_header);

    // Sometimes we can end up with 0x4 bytes leftover, so we have to pad a bit extra
    int32_t to_fill_check = 0x400 - ((msg_size + 0x400) & 0x3FF);
    if (to_fill_check >= 0 && to_fill_check < 8) {
        align_up_bytes += to_fill_check;

        msg_size = data_size + align_up_bytes + sizeof(xrsp_topic_header);
    }

    xrsp_topic_header* header = (xrsp_topic_header*)msg;
    header->version_maybe = 0;
    header->has_alignment_padding = align_up_bytes ? 1 : 0;
    header->packet_version_is_internal = 1;
    header->packet_version_number = 0;
    header->topic = topic;
    header->unk_14_15 = 0;

    header->num_words = ((data_size + align_up_bytes) >> 2) + 1;
    header->sequence_num = host->increment;
    header->pad = 0;

    memcpy(msg_payload, data, data_size);

    if (align_up_bytes)
    {
        if (align_up_bytes > 1)
        {
            memset(msg_payload+data_size, 0xDE, align_up_bytes-1);
        }
        msg_payload[data_size + align_up_bytes - 1] = align_up_bytes;
    }

    uint8_t* msg_end = msg + msg_size;
    memset(msg_end, 0, 0x400);

    int32_t to_fill = 0x400 - ((msg_size + 0x400) & 0x3FF) - 8;
    int32_t final_size = (msg_size + 8 + to_fill);
    //QUEST_LINK_INFO("final_size=%x, to_fill=%x, msg_size=%x", final_size, to_fill, msg_size);
    if (to_fill < 0x3f8 && to_fill >= 0) { // && final_size <= 0x10000
        xrsp_topic_header* fill_header = (xrsp_topic_header*)msg_end;
        fill_header->version_maybe = 0;
        fill_header->has_alignment_padding = 0;
        fill_header->packet_version_is_internal = 1;
        fill_header->packet_version_number = 0;
        fill_header->topic = 0;
        fill_header->unk_14_15 = 0;

        fill_header->num_words = (to_fill >> 2) + 1;
        fill_header->sequence_num = host->increment;
        fill_header->pad = 0;
        msg_size += to_fill + sizeof(xrsp_topic_header);
    }

    xrsp_send_usb(host, msg, msg_size);
    host->increment += 1;

    free(msg);
}

static void xrsp_reset_echo(struct ql_xrsp_host *host)
{
    host->echo_idx = 1;
    host->ns_offset = 0;
    host->ns_offset_from_target = 0;
    host->last_xmt = 0;

    host->echo_req_sent_ns = 0; // client ns
    host->echo_req_recv_ns = 0; // server ns
    host->echo_resp_sent_ns = 0; // server ns
    host->echo_resp_recv_ns = 0; // server ns

    host->frame_sent_ns = 0;
    host->add_test = 0;
    host->sent_mesh = false;
    host->is_inactive = false;

    ql_xrsp_segpkt_destroy(&host->pose_ctx);
    ql_xrsp_ipc_segpkt_destroy(&host->ipc_ctx);

    ql_xrsp_segpkt_init(&host->pose_ctx, 1, ql_xrsp_handle_pose);
    ql_xrsp_ipc_segpkt_init(&host->ipc_ctx, ql_xrsp_handle_ipc);

    if (!host->sys) return;

    struct ql_hmd* hmd = host->sys->hmd;
    if (hmd) {
        hmd->pose_ns = os_monotonic_get_ns();
    }
}

int64_t xrsp_ts_ns_from_target(struct ql_xrsp_host *host, int64_t ts)
{
    int64_t option_1 = (ts) - host->ns_offset;
    int64_t option_2 = (ts) + host->ns_offset_from_target;
    return option_1; // HACK: really need to figure out how to calculate ns_offset
    //return (option_1+option_2)>>1;
}

int64_t xrsp_ts_ns_to_target(struct ql_xrsp_host *host, int64_t ts)
{
    int64_t option_1 = (ts) + host->ns_offset;
    int64_t option_2 = (ts) - host->ns_offset_from_target;
    return option_1; // HACK: really need to figure out how to calculate ns_offset
    //return (option_1+option_2)>>1;
}

int64_t xrsp_target_ts_ns(struct ql_xrsp_host *host)
{
    return xrsp_ts_ns_to_target(host, xrsp_ts_ns(host));
}

int64_t xrsp_ts_ns(struct ql_xrsp_host *host)
{
    return os_monotonic_get_ns();
}

static void xrsp_send_ping(struct ql_xrsp_host *host)
{
    if (xrsp_ts_ns(host) - host->echo_req_sent_ns < 16000000) // 16ms
    {
        return;
    }

    host->echo_req_sent_ns = xrsp_ts_ns(host);

    //QUEST_LINK_INFO("Ping sent: xmt=%zx offs=%zx", host->echo_req_sent_ns, host->ns_offset);

    int32_t request_echo_ping_len = 0;
    uint8_t* request_echo_ping = ql_xrsp_craft_echo(ECHO_PING, host->echo_idx, 0, 0, host->echo_req_sent_ns, host->ns_offset, &request_echo_ping_len);

    xrsp_send_to_topic(host, TOPIC_HOSTINFO_ADV, request_echo_ping, request_echo_ping_len);
    free(request_echo_ping);

    host->echo_idx += 1;
}

// TODO: capnproto struct for these?
static void xrsp_init_session_bye(struct ql_xrsp_host *host)
{
    const uint8_t response_bye_payload[] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int32_t response_bye_len = 0;
    uint8_t* response_bye = ql_xrsp_craft_capnp(BUILTIN_BYE, 0x3E6, 1, response_bye_payload, sizeof(response_bye_payload), &response_bye_len);

    QUEST_LINK_INFO("BYE send");

    xrsp_send_to_topic(host, TOPIC_HOSTINFO_ADV, response_bye, response_bye_len);
    free(response_bye);
}

// TODO: capnproto struct for these?
static void xrsp_init_session(struct ql_xrsp_host *host, struct ql_xrsp_hostinfo_pkt* pkt)
{
    //xrsp_read_usb(host);
    const uint8_t response_ok_payload[] = {0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x03, 0x00, 0x02, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int32_t response_ok_len = 0;
    uint8_t* response_ok = ql_xrsp_craft_capnp(BUILTIN_OK, 0x2C8, 1, response_ok_payload, sizeof(response_ok_payload), &response_ok_len);

    QUEST_LINK_INFO("OK send");

    xrsp_send_to_topic(host, TOPIC_HOSTINFO_ADV, response_ok, response_ok_len);
    free(response_ok);
}

// TODO: capnproto struct for these?
static void xrsp_send_codegen_1(struct ql_xrsp_host *host, struct ql_xrsp_hostinfo_pkt* pkt)
{
    const uint8_t response_codegen_payload[] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int32_t response_codegen_len = 0;
    uint8_t* response_codegen = ql_xrsp_craft_capnp(BUILTIN_CODE_GENERATION, 0xC8, 1, response_codegen_payload, sizeof(response_codegen_payload), &response_codegen_len);

    QUEST_LINK_INFO("Codegen send");

    xrsp_send_to_topic(host, TOPIC_HOSTINFO_ADV, response_codegen, response_codegen_len);
    free(response_codegen);

    //xrsp_read_usb(host); // old
}

// TODO: capnproto struct for these?
static void xrsp_send_pairing_1(struct ql_xrsp_host *host, struct ql_xrsp_hostinfo_pkt* pkt)
{
    const uint8_t response_pairing_payload[] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    int32_t response_pairing_len = 0;
    uint8_t* response_pairing = ql_xrsp_craft_capnp(BUILTIN_PAIRING, 0xC8, 1, response_pairing_payload, sizeof(response_pairing_payload), &response_pairing_len);

    QUEST_LINK_INFO("Pairing send");

    xrsp_send_to_topic(host, TOPIC_HOSTINFO_ADV, response_pairing, response_pairing_len);
    free(response_pairing);

    //xrsp_read_usb(host); // old
}

// TODO: capnproto struct for these?
static void xrsp_trigger_bye(struct ql_xrsp_host *host, struct ql_xrsp_hostinfo_pkt* pkt)
{
    const uint8_t request_video_idk[] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    xrsp_send_to_topic_capnp_wrapped(host, TOPIC_VIDEO, 0, request_video_idk, sizeof(request_video_idk));

    //xrsp_init_session_bye(host);
}

// TODO: capnproto struct for these?
static void xrsp_finish_pairing_1(struct ql_xrsp_host *host, struct ql_xrsp_hostinfo_pkt* pkt)
{
    const uint8_t request_video_idk[] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    QUEST_LINK_INFO("Echo send");
    xrsp_send_ping(host);

    QUEST_LINK_INFO("Video idk cmd send");
    xrsp_send_to_topic_capnp_wrapped(host, TOPIC_VIDEO, 0, request_video_idk, sizeof(request_video_idk));

    QUEST_LINK_INFO("Waiting for user to accept...");
}

// TODO: capnproto struct for these?
static void xrsp_init_session_2(struct ql_xrsp_host *host, struct ql_xrsp_hostinfo_pkt* pkt)
{
    xrsp_reset_echo(host);
    xrsp_read_usb(host);

    struct ql_hmd* hmd = host->sys->hmd;

    uint8_t fps = (uint8_t)hmd->fps;
    uint8_t session_type = 0x03;
    uint8_t error_code = 0x01;

    // 0x0 = AVC/H264, 0x1 = HEVC/H265 TODO TODO get this from the video encoder!
#ifdef XRT_HAVE_VT
    uint8_t encoding_type = 0x1;
#else
    uint8_t encoding_type = 0x0;
#endif
    uint8_t response_ok_2_payload[] = {0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, session_type, 0x00, error_code, 0x00, 0x1F, 0x00, encoding_type, 0x00, (uint8_t)(host->num_slices & 0xF), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, fps, 0x00, /* invalid certs?*/0x00, /* invalid certs?*/0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x1B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x2A, 0x00, 0x00, 0x00, 0x55, 0x53, 0x42, 0x33, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x03, 0x00, 0x02, 0x00, 0x00, 0x00};
    const uint8_t* ok_2_payload = response_ok_2_payload;
    size_t ok_2_payload_len = sizeof(response_ok_2_payload);
    uint8_t ok_2_with_formats[sizeof(response_ok_2_payload) + 16];
    const char* format_option = debug_get_option_camera_ok_formats();

    if ((format_option == NULL || *format_option == '\0') && host->passthrough) format_option = "2";
    if (format_option != NULL && *format_option != '\0') {
        uint16_t formats[4];
        size_t count = 0;
        for (const char* it = format_option; *it != '\0' && count < 4;) {
            char* end = NULL;
            unsigned long value = strtoul(it, &end, 0);
            if (end == it || value > 0xffff) break;
            formats[count++] = (uint16_t)value;
            it = *end == ',' ? end + 1 : end;
            if (*end != ',' && *end != '\0') break;
        }
        const size_t len = ql_invite_ok_add_camera_formats(response_ok_2_payload, sizeof(response_ok_2_payload),
                                                           formats, count, ok_2_with_formats,
                                                           sizeof(ok_2_with_formats));
        if (len != 0) {
            ok_2_payload = ok_2_with_formats;
            ok_2_payload_len = len;
            QUEST_LINK_INFO("CAMERA_OK accepting %zu camera format(s) from QL_CAMERA_OK_FORMATS=%s in InviteOk pointer 3",
                            count, format_option);
        } else {
            QUEST_LINK_ERROR("CAMERA_OK ignoring QL_CAMERA_OK_FORMATS=%s; sending the unmodified InviteOk",
                             format_option);
        }
    }
    int32_t response_ok_2_len = 0;
    uint8_t* response_ok_2 = ql_xrsp_craft_capnp(BUILTIN_OK, 0x2C8, 1, ok_2_payload, ok_2_payload_len, &response_ok_2_len);

    QUEST_LINK_INFO("Done?");

    QUEST_LINK_INFO("OK send #2");
    xrsp_send_to_topic(host, TOPIC_HOSTINFO_ADV, response_ok_2, response_ok_2_len);
    free(response_ok_2);

    QUEST_LINK_INFO("OK read #2");
}

// TODO: capnproto struct for these?
static void xrsp_send_codegen_2(struct ql_xrsp_host *host, struct ql_xrsp_hostinfo_pkt* pkt)
{
    const uint8_t response_codegen_payload[] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int32_t response_codegen_len = 0;
    uint8_t* response_codegen = ql_xrsp_craft_capnp(BUILTIN_CODE_GENERATION, 0xC8, 1, response_codegen_payload, sizeof(response_codegen_payload), &response_codegen_len);

    QUEST_LINK_INFO("Codegen send #2");
    xrsp_send_to_topic(host, TOPIC_HOSTINFO_ADV, response_codegen, response_codegen_len);
    free(response_codegen);

    QUEST_LINK_INFO("Codegen read #2");
}

// TODO: capnproto struct for these?
static void xrsp_send_pairing_2(struct ql_xrsp_host *host, struct ql_xrsp_hostinfo_pkt* pkt)
{
    const uint8_t response_pairing_payload[] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int32_t response_pairing_len = 0;
    uint8_t* response_pairing = ql_xrsp_craft_capnp(BUILTIN_PAIRING, 0xC8, 1, response_pairing_payload, sizeof(response_pairing_payload), &response_pairing_len);

    QUEST_LINK_INFO("Pairing send #2");
    xrsp_send_to_topic(host, TOPIC_HOSTINFO_ADV, response_pairing, response_pairing_len);
    free(response_pairing);

    QUEST_LINK_INFO("Pairing read #2");
}

typedef struct cmd_pkt_idk
{
    uint64_t a;
    uint32_t cmd_idx;

    uint32_t b;
    uint32_t c;
    uint32_t d;
    uint32_t e;
    uint32_t f;
} cmd_pkt_idk;

typedef struct body_pkt_idk
{
    uint32_t a;
    uint32_t b;
} body_pkt_idk;

static void xrsp_camera_probe_tick(struct ql_xrsp_host *host, bool shutting_down = false)
{
    if (!xrsp_camera_enabled(host)) return;
    const bool portal = !debug_get_bool_option_camera_probe();
    const bool paired = host->usb_valid && host->pairing_state == PAIRINGSTATE_PAIRED;
    const bool ready = !shutting_down && !__atomic_load_n(&host->camera_stop_requested, __ATOMIC_ACQUIRE) &&
                       paired && host->ready_to_send_frames &&
                       xrsp_ts_ns(host) - host->paired_ns >= 5000000000LL;

    const int64_t override_s = std::min<int64_t>(debug_get_num_option_camera_probe_seconds(), 3600);
    const int64_t duration_ns = override_s > 0 ? override_s * 1000000000LL
                                : debug_get_num_option_camera_drop_test_window() > 0 ? 15000000000LL
                                                                                   : 10000000000LL;
    const auto action =
        portal ? ql_camera_portal_step(&host->camera_probe, os_monotonic_get_ns(), ready,
                                       __atomic_load_n(&g_passthrough_requested, __ATOMIC_ACQUIRE),
                                       ql_xrsp_camera_decoder_stalled(host))
               : ql_camera_probe_step(&host->camera_probe, os_monotonic_get_ns(), ready,
                                      ql_xrsp_camera_decoder_stalled(host), duration_ns);
    __atomic_store_n(&host->camera_streaming, host->camera_probe.active, __ATOMIC_RELEASE);
    if (action == QL_CAMERA_PROBE_NONE) return;
    if (!paired) {
        QUEST_LINK_INFO("CAMERA_PROBE ended with session loss; no command sent to disconnected session");
        return;
    }

    const bool starting = action == QL_CAMERA_PROBE_START || action == QL_CAMERA_PROBE_RESTART_START;
    const uint32_t command = starting ? COMMAND_ENABLE_CAMERA_STREAM : COMMAND_DISABLE_CAMERA_STREAM;
    cmd_pkt_idk packet = {(uint64_t)xrsp_ts_ns(host), command, 0, 0, 0, 0, 0};
    static_assert(sizeof(packet) == 32, "XRSP legacy command wire size");
    if (action == QL_CAMERA_PROBE_RESTART_STOP) {
        QUEST_LINK_INFO("CAMERA_RECOVERY decoder stalled; restarting camera stream (restart %d of %d): stop (0x%x)",
                        host->camera_probe.restarts, QL_CAMERA_PROBE_MAX_RESTARTS, command);
    } else if (action == QL_CAMERA_PROBE_RESTART_START) {
        QUEST_LINK_INFO("CAMERA_RECOVERY restarting camera stream: start (0x%x) after %.0f ms pause", command,
                        QL_CAMERA_PROBE_RESTART_PAUSE_NS / 1e6);
    } else if (portal) {
        QUEST_LINK_INFO("PASSTHROUGH portal %s: camera streaming %s (0x%x)", starting ? "opened" : "closed",
                        starting ? "start" : "stop", command);
        if (!starting) ql_xrsp_camera_clear_frames();
    } else {
        QUEST_LINK_INFO("CAMERA_PROBE requesting %s (legacy command=0x%x, automatic stop after %.0f seconds)",
                        starting ? "start" : "stop", command, duration_ns / 1e9);
    }
    xrsp_send_to_topic(host, TOPIC_COMMAND, (const uint8_t*)&packet, sizeof(packet));
    if (action == QL_CAMERA_PROBE_START) {
        memcpy(host->camera_probe_topic_baseline, host->link_probe_topic_packets,
               sizeof(host->camera_probe_topic_baseline));
    }
    if (action == QL_CAMERA_PROBE_STOP && !portal) {
        QUEST_LINK_INFO("CAMERA_PROBE finished: received camera packets=%llu bytes=%llu (no images saved)",
                        (unsigned long long)host->link_probe_packets[2],
                        (unsigned long long)host->link_probe_bytes[2]);
        ql_xrsp_camera_log_summary(host); // keeps reassembly state for trailing packets
        for (int topic = 0; topic < 64; topic++) {
            const uint64_t during = host->link_probe_topic_packets[topic] -
                                    host->camera_probe_topic_baseline[topic];
            if (during == 0) continue;
            QUEST_LINK_INFO("CAMERA_PROBE window topic %s (0x%x): packets=%llu (before start=%llu)",
                            xrsp_topic_str(topic), topic, (unsigned long long)during,
                            (unsigned long long)host->camera_probe_topic_baseline[topic]);
        }
    }
}

static void xrsp_finish_pairing_2(struct ql_xrsp_host *host, struct ql_xrsp_hostinfo_pkt* pkt)
{
    // TODO: are those hex values just timestamps
    struct cmd_pkt_idk send_cmd_chemx_toggle = {0x0005EC94E91B9D4F, COMMAND_TOGGLE_CHEMX, 0, 0, 0, 0, 0};
    struct cmd_pkt_idk send_cmd_asw_toggle = {0x0005EC94E91B9D83, COMMAND_TOGGLE_ASW, 0, 0, 0, 0, 0};
    struct cmd_pkt_idk send_cmd_asw_disable = {0x0005EC94E91B9D83, COMMAND_TOGGLE_ASW, 0, 1, 0, 0, 0};
    struct cmd_pkt_idk send_cmd_dropframestate_toggle = {0x0005EC94E91B9D83, COMMAND_DROP_FRAMES_STATE, 0, 0, 0, 0, 0};
    struct cmd_pkt_idk send_cmd_dropframestate_disable = {0x0005EC94E91B9D83, COMMAND_DROP_FRAMES_STATE, 0, 1, 0, 0, 0};

    QUEST_LINK_INFO("Echo send");
    xrsp_send_ping(host);

    //xrsp_send_mesh(host);

    QUEST_LINK_INFO("Audio Control cmd send");
    xrsp_send_audio_control(host, 1, 1, 0, 0.0, 0.0);

    xrsp_send_to_topic(host, TOPIC_COMMAND, (uint8_t*)&send_cmd_chemx_toggle, sizeof(send_cmd_chemx_toggle)); // link sharpening
    xrsp_send_to_topic(host, TOPIC_COMMAND, (uint8_t*)&send_cmd_asw_toggle, sizeof(send_cmd_asw_toggle));
    //xrsp_send_to_topic(host, TOPIC_COMMAND, (uint8_t*)&send_cmd_asw_disable, sizeof(send_cmd_asw_disable));
    //xrsp_send_to_topic(host, TOPIC_COMMAND, (uint8_t*)&send_cmd_dropframestate_toggle, sizeof(send_cmd_dropframestate_toggle));
    xrsp_send_to_topic(host, TOPIC_COMMAND, (uint8_t*)&send_cmd_dropframestate_disable, sizeof(send_cmd_dropframestate_disable));

    xrsp_send_input_control(host, 1, 1, 0, 0.0, 0.0); // Hands enable
    xrsp_send_input_control(host, 2, 1, 0, 0.0, 0.0); // Body enable


    // Packages?
    // com.oculus.systemdriver
    // com.facebook.spatial_persistence_service
    // com.oculus.bodyapiservice
    // com.oculus.qplservice
    // com.oculus.presence
    // com.oculus.os.dialoghost
    // com.oculus.vrguardianservice?

    // Client: com.oculus.vrshell:com.oculus.vrshell:Overlay:2352, Server: com.oculus.os.dialoghost:com.oculus.os.dialoghost (DialogHostService)

    xrsp_ripc_ensure_service_started(host, host->client_id, "com.oculus.systemdriver", "com.oculus.vrruntimeservice.VrRuntimeService");
    xrsp_ripc_connect_to_remote_server(host, RIPC_FAKE_CLIENT_1, "com.oculus.systemdriver", "com.oculus.vrruntimeservice", "RuntimeServiceServer");


    xrsp_ripc_ensure_service_started(host, host->client_id+1, "com.oculus.bodyapiservice", "com.oculus.bodyapiservice.BodyApiService");
    xrsp_ripc_connect_to_remote_server(host, RIPC_FAKE_CLIENT_2, "com.oculus.bodyapiservice", "com.oculus.bodyapiservice", "BodyApiServiceServer");

    xrsp_ripc_ensure_service_started(host, host->client_id+2, "com.oculus.bodyapiservice", "com.oculus.eyetrackingservice.EyeTrackingService");
    xrsp_ripc_connect_to_remote_server(host, RIPC_FAKE_CLIENT_3, "com.oculus.bodyapiservice", "com.oculus.eyetrackingservice", "EyeTrackingServiceServer");

    //xrsp_ripc_ensure_service_started(host, host->client_id+4, "com.oculus.vrshell", "com.oculus.panelapp.dogfood.DogfoodPanelService");
    //xrsp_ripc_ensure_service_started(host, host->client_id+4, "com.oculus.vrshell", "com.oculus.panelapp.debug.ShellDebugMultiInstanceService");

    //xrsp_ripc_ensure_service_started(host, host->client_id+3, "com.oculus.os.dialoghost", "com.oculus.os.dialoghost.DialogHostService");
    //xrsp_ripc_connect_to_remote_server(host, RIPC_FAKE_CLIENT_4, "com.oculus.os.dialoghost", "com.oculus.os.dialoghost", "DialogHostService");

    //if (!host->sent_mesh)
    {
        xrsp_send_mesh(host);
    }
}

static void xrsp_handle_echo(struct ql_xrsp_host *host, struct ql_xrsp_hostinfo_pkt* pkt)
{
    ql_xrsp_echo_payload* payload = (ql_xrsp_echo_payload*)pkt->payload;
    if ((pkt->result & 1) == 1) // PONG
    {
        host->echo_req_recv_ns = payload->recv; // server recv ns
        host->echo_resp_sent_ns = payload->xmt; // server tx ns
        host->echo_resp_recv_ns = pkt->recv_ns; // client rx ns
        host->echo_req_sent_ns = xrsp_ts_ns(host);

        int64_t calc_ns_offset = ((host->echo_req_recv_ns-host->echo_req_sent_ns) + (host->echo_resp_sent_ns-pkt->recv_ns)) >> 1; // 2

        if (!host->ns_offset) {
            host->ns_offset = calc_ns_offset;
        }
        else {
            //host->ns_offset = calc_ns_offset;
            host->ns_offset += calc_ns_offset;
            host->ns_offset /= 2;
        }

        //QUEST_LINK_INFO("Ping offs: %zx %zx %zd/%zd", host->ns_offset, -host->ns_offset_from_target, host->ns_offset-host->ns_offset_from_target, host->ns_offset_from_target-host->ns_offset);

        //QUEST_LINK_INFO("Pong get: org=%zx recv=%zx xmt=%zx offs=%zx", payload->org, payload->recv, payload->xmt, payload->offset);

        /*if (payload->offset) {
            host->ns_offset_from_target = payload->offset;
        }*/

        if (host->pairing_state == PAIRINGSTATE_PAIRED) {
            xrsp_send_ping(host);
        }
    }
    else //PING
    {
        host->last_xmt = payload->xmt;

        if (payload->offset) {
            host->ns_offset_from_target = payload->offset;
            host->ns_offset -= host->ns_offset_from_target;
            host->ns_offset /= 2;
        }

        //QUEST_LINK_INFO("Ping get: org=%zx recv=%zx xmt=%zx offs=%zx", payload->org, payload->recv, payload->xmt, payload->offset);

        int32_t request_echo_ping_len = 0;
        int64_t send_xmt = xrsp_ts_ns(host);
        uint8_t* request_echo_ping = ql_xrsp_craft_echo(ECHO_PONG, pkt->unk_4, host->last_xmt, pkt->recv_ns, send_xmt, host->ns_offset, &request_echo_ping_len);

        //QUEST_LINK_INFO("Pong sent: org=%zx recv=%zx xmt=%zx offs=%zx", host->last_xmt, pkt->recv_ns, send_xmt, host->ns_offset);

        xrsp_send_to_topic(host, TOPIC_HOSTINFO_ADV, request_echo_ping, request_echo_ping_len);
        free(request_echo_ping);

        if (host->pairing_state == PAIRINGSTATE_PAIRED) {
            xrsp_send_ping(host);
        }
    }
}

static void xrsp_log_invite_u16_lists(capnp::AnyStruct::Reader root)
{
    auto pointers = root.getPointerSection();
    QUEST_LINK_INFO("LINK_PROBE invite root: data=%zu bytes pointers=%u",
                    root.getDataSection().size(), pointers.size());
    for (uint32_t slot = 0; slot < pointers.size(); slot++) {
        auto ptr = pointers[slot];
        if (ptr.getPointerType() != capnp::PointerType::LIST) continue;
        if (ptr.getAs<capnp::AnyList>().getElementSize() != capnp::ElementSize::TWO_BYTES) continue;
        auto list = ptr.getAs<capnp::List<uint16_t>>();
        char values[256] = {0};
        size_t used = 0;
        for (uint32_t i = 0; i < list.size() && i < 32 && used < sizeof(values); i++) {
            int n = snprintf(values + used, sizeof(values) - used, "%s0x%x", i ? "," : "", list[i]);
            if (n < 0) break;
            used += (size_t)n;
        }
        QUEST_LINK_INFO("LINK_PROBE invite ptr[%u] UInt16 count=%u values=[%s]%s",
                        slot, list.size(), values, list.size() > 32 ? " (truncated)" : "");
    }
}

static void xrsp_handle_invite(struct ql_xrsp_host *host, struct ql_xrsp_hostinfo_pkt* pkt)
{
    ql_xrsp_hostinfo_capnp_payload* payload = (ql_xrsp_hostinfo_capnp_payload*)pkt->payload;
    uint8_t* capnp_data = pkt->payload + sizeof(*payload);

    try
    {
        kj::ArrayPtr<const capnp::word> dataptr[1] = {kj::arrayPtr((capnp::word*)capnp_data, payload->len_u64s)};
        capnp::SegmentArrayMessageReader message(kj::arrayPtr(dataptr, 1));

        PayloadHostInfo::Reader info = message.getRoot<PayloadHostInfo>();

        if (debug_get_bool_option_link_probe() || debug_get_bool_option_camera_probe()) {
            xrsp_log_invite_u16_lists(message.getRoot<capnp::AnyStruct>());
        }

        HeadsetConfig::Reader config = info.getConfig();
        HeadsetDescription::Reader description = config.getDescription();
        HeadsetLens::Reader lensLeft = description.getLeftLens();
        HeadsetLens::Reader lensRight = description.getRightLens();

        struct ql_hmd* hmd = host->sys->hmd;

        // TODO mutex

        os_mutex_lock(&host->pose_mutex);
        hmd->device_type = description.getDeviceType();

        if (hmd->device_type == DEVICE_TYPE_QUEST_2) {
            hmd->fps = 120;
        }
        else if (hmd->device_type == DEVICE_TYPE_QUEST_PRO) {
            hmd->fps = 90;
        }
        else if (hmd->device_type == DEVICE_TYPE_QUEST_3) {
            hmd->fps = 90;
        }
        else {
            hmd->fps = 72;
        }

        float scale = 0.75;
        if (host->usb_slow_cable) {
            if (hmd->device_type == DEVICE_TYPE_QUEST_2) {
                hmd->fps = 90;
            }
        }

        int fps_override = debug_get_num_option_force_fps();
        if (fps_override > 0) {
            hmd->fps = fps_override;
        }

        float scale_override = debug_get_float_option_force_scale();
        if (scale_override > 0) {
            scale = scale_override;
        }

        // Quest 2:
        // 58mm (0.057928182) angle_left -> -52deg
        // 65mm (0.065298356) angle_left -> -49deg
        // 68mm (0.068259589) angle_left -> -43deg

        // Pull FOV information
        hmd->base.hmd->distortion.fov[0].angle_up = lensLeft.getAngleUp() * M_PI / 180;
        hmd->base.hmd->distortion.fov[0].angle_down = -lensLeft.getAngleDown() * M_PI / 180;
        hmd->base.hmd->distortion.fov[0].angle_left = -lensLeft.getAngleLeft() * M_PI / 180;
        hmd->base.hmd->distortion.fov[0].angle_right = lensLeft.getAngleRight() * M_PI / 180;

        hmd->base.hmd->distortion.fov[1].angle_up = lensRight.getAngleUp() * M_PI / 180;
        hmd->base.hmd->distortion.fov[1].angle_down = -lensRight.getAngleDown() * M_PI / 180;
        hmd->base.hmd->distortion.fov[1].angle_left = -lensRight.getAngleLeft() * M_PI / 180;
        hmd->base.hmd->distortion.fov[1].angle_right = lensRight.getAngleRight() * M_PI / 180;

        hmd->fov_angle_left = lensLeft.getAngleLeft();

        int w = (int)((float)description.getResolutionWidth() * scale);
        int h = (int)((float)description.getResolutionHeight() * scale);
        int w_override = debug_get_num_option_force_w();
        if (w_override > 0) {
            w = w_override;
        }
        int h_override = debug_get_num_option_force_h();
        if (h_override > 0) {
            h = h_override;
        }

        QUEST_LINK_INFO("HMD FPS is %d, scale is %f, w=%d, h=%d", hmd->fps, scale, w, h);
        ql_hmd_set_per_eye_resolution(hmd, w, h, /*description.getRefreshRateHz()*/ hmd->fps);

        os_mutex_unlock(&host->pose_mutex);
    }
    catch(...) {

    }
}

static void xrsp_handle_hostinfo_adv(struct ql_xrsp_host *host)
{
    int ret;
    struct ql_xrsp_topic_pkt* pkt = &host->working_pkt;

    struct ql_xrsp_hostinfo_pkt hostinfo;
    ret = ql_xrsp_hostinfo_pkt_create(&hostinfo, pkt, host);
    if (ret < 0) {
        // TODO
    }

    if (hostinfo.message_type == BUILTIN_ECHO) {
        xrsp_handle_echo(host, &hostinfo);
        return;
    }

    // Pull lens and distortion info
    if (hostinfo.message_type == BUILTIN_INVITE) {
        xrsp_handle_invite(host, &hostinfo);
    }

    if (host->pairing_state == PAIRINGSTATE_WAIT_FIRST)
    {
        if (hostinfo.message_type == BUILTIN_INVITE) {
            xrsp_init_session(host, &hostinfo);
        }
        else if (hostinfo.message_type == BUILTIN_ACK) {
            xrsp_send_codegen_1(host, &hostinfo);
        }
        else if (hostinfo.message_type == BUILTIN_CODE_GENERATION_ACK) {
            xrsp_send_pairing_1(host, &hostinfo);
        }
        else if (hostinfo.message_type == BUILTIN_PAIRING_ACK) {
            xrsp_finish_pairing_1(host, &hostinfo);

            host->pairing_state = PAIRINGSTATE_WAIT_SECOND;
        }
    }
    else if (host->pairing_state == PAIRINGSTATE_WAIT_SECOND || host->pairing_state == PAIRINGSTATE_PAIRING)
    {
        if (hostinfo.message_type == BUILTIN_INVITE) {
            host->pairing_state = PAIRINGSTATE_PAIRING;
            xrsp_init_session_2(host, &hostinfo);
        }
        else if (hostinfo.message_type == BUILTIN_ACK) {
            xrsp_send_codegen_2(host, &hostinfo);
        }
        else if (hostinfo.message_type == BUILTIN_CODE_GENERATION_ACK) {
            xrsp_send_pairing_2(host, &hostinfo);
        }
        else if (hostinfo.message_type == BUILTIN_PAIRING_ACK) {
            xrsp_finish_pairing_2(host, &hostinfo);

            host->pairing_state = PAIRINGSTATE_PAIRED;

            host->paired_ns = xrsp_ts_ns(host);
            host->audio_tone_start_ns = 0; // QL_AUDIO_TEST_TONE plays once per pairing
            host->audio_logged = false;
            host->audio_next_ns = 0;
        }
    }
}

static void xrsp_handle_pkt(struct ql_xrsp_host *host)
{
    struct ql_xrsp_topic_pkt* pkt = &host->working_pkt;
    xrsp_camera_probe_tick(host);

    if (debug_get_bool_option_link_probe() || debug_get_bool_option_camera_probe()) {
        const int index = pkt->topic == TOPIC_AUDIO ? 0 :
                          pkt->topic == TOPIC_AUDIO_CONTROL ? 1 :
                          pkt->topic == TOPIC_CAMERA_STREAM ? 2 : -1;
        bool first_packet = false;
        if (++host->link_probe_topic_packets[pkt->topic] == 1) {
            QUEST_LINK_INFO("LINK_PROBE first rx on topic %s (0x%x)", xrsp_topic_str(pkt->topic), pkt->topic);
        }
        host->link_probe_topic_bytes[pkt->topic] += pkt->payload_valid;
        if (index >= 0) {
            first_packet = ++host->link_probe_packets[index] == 1;
            host->link_probe_bytes[index] += pkt->payload_valid;
        }
        const int64_t now = os_monotonic_get_ns();
        if (first_packet || now - host->link_probe_last_report_ns >= 5000000000LL) {
            host->link_probe_last_report_ns = now;
            QUEST_LINK_INFO("LINK_PROBE rx totals audio=%llu/%lluB control=%llu/%lluB camera=%llu/%lluB paired=%d",
                            (unsigned long long)host->link_probe_packets[0],
                            (unsigned long long)host->link_probe_bytes[0],
                            (unsigned long long)host->link_probe_packets[1],
                            (unsigned long long)host->link_probe_bytes[1],
                            (unsigned long long)host->link_probe_packets[2],
                            (unsigned long long)host->link_probe_bytes[2],
                            host->pairing_state == PAIRINGSTATE_PAIRED);
        }
    }

    if (pkt->topic != TOPIC_CAMERA_STREAM || !xrsp_camera_enabled(host)) {
        ql_xrsp_topic_pkt_dump(pkt);
    }

    if (pkt->topic == TOPIC_HOSTINFO_ADV)
    {
        xrsp_handle_hostinfo_adv(host);
    }
    else if (pkt->topic == TOPIC_POSE)
    {
        ql_xrsp_segpkt_consume(&host->pose_ctx, host, pkt);
    }
    else if (pkt->topic == TOPIC_HANDS)
    {
        ql_xrsp_handle_hands(host, pkt);
    }
    else if (pkt->topic == TOPIC_SKELETON)
    {
        ql_xrsp_handle_skeleton(host, pkt);
    }
    else if (pkt->topic == TOPIC_BODY)
    {
        ql_xrsp_handle_body(host, pkt);
    }
    else if (pkt->topic == TOPIC_LOGGING)
    {
        ql_xrsp_handle_logging(host, pkt);
    }
    else if (pkt->topic == TOPIC_RUNTIME_IPC)
    {
        ql_xrsp_ipc_segpkt_consume(&host->ipc_ctx, host, pkt);
    }
    else if (pkt->topic == TOPIC_CAMERA_STREAM && xrsp_camera_enabled(host))
    {
        __atomic_store_n(&host->camera_last_rx_ns, os_monotonic_get_ns(), __ATOMIC_RELEASE);
        ql_xrsp_camera_consume(host, pkt);
    }
    else if (pkt->topic == TOPIC_AUDIO && debug_get_bool_option_audio_probe())
    {

        static uint64_t audio_probe_count = 0;
        audio_probe_count++;
        if (audio_probe_count <= 5 || (audio_probe_count % 480) == 0) {
            uint32_t shown = pkt->payload_valid < 16 ? pkt->payload_valid : 16;
            char hex[3 * 16 + 1] = {0};
            for (uint32_t i = 0; i < shown; i++) {
                snprintf(hex + i * 3, sizeof(hex) - i * 3, "%02x%s", pkt->payload[i], i + 1 == shown ? "" : " ");
            }
            QUEST_LINK_INFO("TOPIC_AUDIO probe count=%llu seq=%u words=%u payload=%u valid=%u first=[%s]",
                            (unsigned long long)audio_probe_count,
                            pkt->sequence_num,
                            pkt->num_words,
                            pkt->payload_size,
                            pkt->payload_valid,
                            hex);
        }
    }

    if ((pkt->topic == TOPIC_POSE || pkt->topic == TOPIC_SKELETON || pkt->topic == TOPIC_LOGGING) && host->pairing_state != PAIRINGSTATE_PAIRED)
    {
        // A session handoff (dashboard <-> game) can leave a few stray
        // telemetry packets from the just-ended session still in flight
        // while the new session's handshake is getting underway -- that's
        // normal, not a stuck connection. Only force a disruptive full USB
        // reset (which itself causes a burst of transient pipe errors and
        // forces the headset to show the pairing-acceptance prompt again)
        // once we've genuinely been stuck unpaired for a while.
        int64_t since_init_ns = os_monotonic_get_ns() - host->last_init_ns;
        if (since_init_ns > 2000LL * U_TIME_1MS_IN_NS) {
            xrsp_trigger_bye(host, NULL);
            ql_xrsp_usb_init(host, true);
        }
    }

    if (host->pairing_state == PAIRINGSTATE_PAIRED && xrsp_ts_ns(host) - host->echo_req_sent_ns > 1000000000) {
        xrsp_send_ping(host);
    }
}

static constexpr int kUsbBacklogReads = 64;

static constexpr int64_t kUsbLargeHoldNs = 100000000LL;

static constexpr int64_t kUsbLargeFastNs = 1000000LL;

static void xrsp_handle_pkt_timed(struct ql_xrsp_host *host)
{
    if (!debug_get_bool_option_link_probe()) {
        xrsp_handle_pkt(host);
        return;
    }
    const int kind = host->working_pkt.topic == TOPIC_CAMERA_STREAM ? 1 : 0;
    const int64_t start = os_monotonic_get_ns();
    xrsp_handle_pkt(host);
    const int64_t spent = os_monotonic_get_ns() - start;
    host->link_read.handle_ns[kind] += spent;
    if (spent > host->link_read.handle_max_ns[kind]) host->link_read.handle_max_ns[kind] = spent;
}

static void xrsp_link_read_report(struct ql_xrsp_host *host, int64_t now)
{
    auto &r = host->link_read;
    if (r.window_start_ns == 0) {
        r = {};
        r.window_start_ns = now;
        r.pose_baseline = host->link_probe_topic_packets[TOPIC_POSE];
        return;
    }
    const int64_t span = now - r.window_start_ns;
    if (span < 2000000000LL) return;
    const double s = span / 1e9;
    const double pct = 100.0 / span;
    QUEST_LINK_INFO("LINK_READ %.1fs: reads=%llu (with data %llu, %.0f%%; large %llu) %.2f MB/s, %.0f us/read; busy usb=%.0f%% "
                    "handle other=%.0f%% camera=%.0f%%; max handle other=%.2f ms camera=%.2f ms; pose rx %.0f/s",
                    s, (unsigned long long)r.reads, (unsigned long long)r.data_reads,
                    r.reads ? 100.0 * r.data_reads / r.reads : 0.0, (unsigned long long)r.large_reads, r.bytes / s / 1e6,
                    r.reads ? r.usb_ns / 1e3 / r.reads : 0.0, r.usb_ns * pct, r.handle_ns[0] * pct, r.handle_ns[1] * pct, r.handle_max_ns[0] / 1e6,
                    r.handle_max_ns[1] / 1e6, (host->link_probe_topic_packets[TOPIC_POSE] - r.pose_baseline) / s);
    r = {};
    r.window_start_ns = now;
    r.pose_baseline = host->link_probe_topic_packets[TOPIC_POSE];
}

static bool xrsp_read_usb(struct ql_xrsp_host *host)
{
    int ret;

    if (!host->usb_valid) return false;

    const int large_read =
        (int)std::min<int64_t>(std::max<int64_t>(host->passthrough &&
                                                       getenv("QL_USB_READ_SIZE") == NULL
                                                   ? 0x4000
                                                   : debug_get_num_option_usb_read_size(),
                                               0x400),
                               0x100000) &
        ~0x3ff;
    std::vector<unsigned char> buffer(large_read);
    unsigned char *data = buffer.data();

    while (true)
    {
        int32_t data_consumed = 0;
        const int amt_to_read =
            host->usb_read_streak >= kUsbBacklogReads || os_monotonic_get_ns() < host->usb_large_until_ns
                ? large_read
                : 0x400;

        int read_len = 0;
        const bool probe = debug_get_bool_option_link_probe();
        const int64_t usb_start = os_monotonic_get_ns();
        int r = libusb_bulk_transfer(host->dev, host->ep_in, data, amt_to_read, &read_len, 1);
        const int64_t usb_end = os_monotonic_get_ns();
        if (probe) {
            host->link_read.usb_ns += usb_end - usb_start;
            host->link_read.reads++;
            if (read_len > 0) {
                host->link_read.data_reads++;
                host->link_read.bytes += read_len;
            }
            xrsp_link_read_report(host, usb_end);
        }
        const bool partial_timeout = r == LIBUSB_ERROR_TIMEOUT && read_len > 0;
        if (read_len > 0 && !partial_timeout) {
            host->usb_read_streak++;
            if (amt_to_read > 0x400 && read_len == amt_to_read && usb_end - usb_start < kUsbLargeFastNs) {
                host->usb_large_until_ns = usb_end + kUsbLargeHoldNs;
            }
        } else {
            host->usb_read_streak = 0;
        }
        if (probe && amt_to_read > 0x400) host->link_read.large_reads++;
        if ((r != 0 && !partial_timeout) || !read_len) {
            if (r != LIBUSB_ERROR_TIMEOUT) {
                QUEST_LINK_ERROR("libusb error: %s", libusb_strerror((libusb_error)r));
            }

            if (r == LIBUSB_ERROR_NO_DEVICE) {
                ql_xrsp_usb_init(host, true);
            }
            break;
        }

        if (read_len) {
            host->last_read_ns = xrsp_ts_ns(host);
        }

        //QUEST_LINK_INFO("Read %x bytes", read_len);
        //hex_dump(data, read_len);

        if (!host->have_working_pkt) {
            ret = ql_xrsp_topic_pkt_create(&host->working_pkt, data, read_len, host->last_read_ns);
            if (ret < 0) {
                // TODO
                data_consumed += 0x8;
                host->have_working_pkt = false;
            }
            else {
                data_consumed += ret;
                host->have_working_pkt = true;
            }
        }
        else if (host->working_pkt.missing_bytes == 0) {
            try {
                xrsp_handle_pkt_timed(host);
            }
            catch(...) {
                QUEST_LINK_ERROR("Exception while parsing packet...");
            }


            QUEST_LINK_INFO("Is remaining data possible?");

            int32_t remaining_data = read_len - data_consumed;
        }
        else {
            ret = ql_xrsp_topic_pkt_append(&host->working_pkt, data, read_len);
            if (ret < 0) {
                // TODO
                data_consumed += 0x8;
                host->have_working_pkt = false;
            }
            else {
                data_consumed += ret;
            }
        }

        while (host->have_working_pkt) {
            if (host->working_pkt.missing_bytes == 0) {
                xrsp_handle_pkt_timed(host);
                ql_xrsp_topic_pkt_destroy(&host->working_pkt);
                host->have_working_pkt = false;
            }

            int32_t remaining_data = read_len - data_consumed;
            if (remaining_data <= 0) {
                break;
            }

            if (remaining_data > 0 && remaining_data < 8) {
                hex_dump(&data[data_consumed], read_len-data_consumed);
                ql_xrsp_topic_pkt_destroy(&host->working_pkt);
                host->have_working_pkt = false;
            }
            else if (remaining_data > 0) {
                ret = ql_xrsp_topic_pkt_create(&host->working_pkt, &data[data_consumed], remaining_data, host->last_read_ns);
                if (ret < 0) {
                    // TODO
                    data_consumed += 0x8;
                    host->have_working_pkt = false;
                }
                else {
                    data_consumed += ret;
                    host->have_working_pkt = true;
                }
            }
        }
        if (partial_timeout) break; // nothing more queued right now
    }

    return true;
}

static void xrsp_send_mesh(struct ql_xrsp_host *host)
{
    struct ql_hmd* hmd = host->sys->hmd;

    ::capnp::MallocMessageBuilder message;
    PayloadRectifyMesh::Builder msg = message.initRoot<PayloadRectifyMesh>();

    // TODO how are the resolutions determined?
    msg.setMeshId(QL_MESH_FOVEATED);
    msg.setInputResX(hmd->encode_width); // 3680
    msg.setInputResY(hmd->encode_height); // 1920
    msg.setOutputResX(hmd->encode_width); // 4128
    msg.setOutputResY(hmd->encode_height); // 2096
    msg.setUnk2p1(0);

    ::capnp::List<MeshVtx>::Builder vertices = msg.initVertices(hmd->quest_vtx_count);
    ::capnp::List<uint16_t>::Builder indices = msg.initIndices(hmd->quest_index_count);

    for (int i = 0; i < hmd->quest_vtx_count; i++)
    {
        vertices[i].setU1(hmd->quest_vertices[(i*4)+0]);
        vertices[i].setV1(hmd->quest_vertices[(i*4)+1]);
        vertices[i].setU2(hmd->quest_vertices[(i*4)+2]);
        vertices[i].setV2(hmd->quest_vertices[(i*4)+3]);
    }

    //msg.setIndices(kj::arrayPtr(hmd->quest_indices, hmd->quest_index_count));

    for (int i = 0; i < hmd->quest_index_count; i++)
    {
        indices.set(i, hmd->quest_indices[i]);
    }

    kj::ArrayPtr<const kj::ArrayPtr<const capnp::word>> out = message.getSegmentsForOutput();

    xrsp_send_to_topic_capnp_segments(host, TOPIC_MESH, 2, out);

    host->sent_mesh = true;
}

// TODO: figure out the params
static void xrsp_send_audio_control(struct ql_xrsp_host *host, uint16_t a, uint16_t b, uint32_t c, float d, float e)
{
    if (debug_get_bool_option_link_probe()) {
        QUEST_LINK_INFO("LINK_PROBE tx audio-control source=%u flags=0x%x kind=%u value0=%g value1=%g",
                        (unsigned)a, (unsigned)b, (unsigned)c, (double)d, (double)e);
    }
    ::capnp::MallocMessageBuilder message;
    PayloadAudioControl::Builder msg = message.initRoot<PayloadAudioControl>();

    msg.setDataUnk0(a);
    msg.setDataUnk1(b);
    msg.setDataUnk2(c);
    msg.setDataUnk3(d);
    msg.setDataUnk4(e);

    kj::ArrayPtr<const kj::ArrayPtr<const capnp::word>> out = message.getSegmentsForOutput();
    xrsp_send_to_topic_capnp_segments(host, TOPIC_AUDIO_CONTROL, 0, out);
}

static void xrsp_send_audio(struct ql_xrsp_host *host, const float *interleaved, uint32_t frames, int64_t host_ns)
{
    if (host->pairing_state != PAIRINGSTATE_PAIRED || frames == 0) return;
    ::capnp::MallocMessageBuilder message;
    PayloadAudio::Builder msg = message.initRoot<PayloadAudio>();
    msg.setDataUnk0((uint64_t)xrsp_ts_ns_to_target(host, host_ns));
    const size_t bytes = (size_t)frames * 2 * sizeof(float);
    auto data = msg.initData(bytes);
    memcpy(data.begin(), interleaved, bytes);
    kj::ArrayPtr<const kj::ArrayPtr<const capnp::word>> out = message.getSegmentsForOutput();
    xrsp_send_to_topic_capnp_segments(host, TOPIC_AUDIO, 0, out);
}

namespace {
constexpr uint32_t kAudioRate = 48000, kAudioChunk = 480, kAudioRingFrames = kAudioRate / 5;
constexpr uint32_t kAudioPrimeFrames = kAudioRate * 40 / 1000, kAudioMaxFrames = kAudioRate / 10;
constexpr int64_t kAudioChunkNs = 10000000;
std::mutex g_audio_mutex;
QlAudioBuffer<kAudioRingFrames> g_audio_buffer;
QlAudioBuffer<kAudioRingFrames> g_addon_audio_buffer;
bool g_audio_paired = false; // protected by g_audio_mutex, together with the samples
} // namespace

extern "C" __attribute__((visibility("default"))) bool
ql_audio_submit(const float *interleaved, uint32_t frames)
{
    std::lock_guard<std::mutex> lock(g_audio_mutex);
    return g_audio_paired && g_audio_buffer.push(interleaved, frames);
}

extern "C" __attribute__((visibility("default"))) bool
ql_audio_addon_submit(const float *interleaved, uint32_t frames)
{
    std::lock_guard<std::mutex> lock(g_audio_mutex);
    return g_audio_paired && g_addon_audio_buffer.push(interleaved, frames);
}

static void xrsp_audio_drain(struct ql_xrsp_host *host)
{
    const bool paired = host->pairing_state == PAIRINGSTATE_PAIRED && host->ready_to_send_frames;
    const int64_t now = xrsp_ts_ns(host);
    float chunk[kAudioChunk * 2];
    for (;;) {
        {
            std::lock_guard<std::mutex> lock(g_audio_mutex);
            g_audio_paired = paired;
            if (!paired) {
                g_audio_buffer.clear();
                g_addon_audio_buffer.clear();
                host->audio_next_ns = 0;
                return;
            }
            if (g_audio_buffer.size() > kAudioMaxFrames) {
                g_audio_buffer.discard(g_audio_buffer.size() - kAudioPrimeFrames);
                host->audio_next_ns = now; // retained samples belong to the new timeline
                host->audio_trims++;
            }
            if (g_addon_audio_buffer.size() > kAudioMaxFrames)
                g_addon_audio_buffer.discard(g_addon_audio_buffer.size() - kAudioPrimeFrames);
            if (host->audio_next_ns == 0) { // (re)starting: wait for 40 ms in hand
                if (std::max(g_audio_buffer.size(), g_addon_audio_buffer.size()) < kAudioPrimeFrames) return;
                host->audio_next_ns = now;
            }
            if (now < host->audio_next_ns) return;
            if (g_audio_buffer.size() < kAudioChunk && g_addon_audio_buffer.size() < kAudioChunk) { // ran dry
                host->audio_next_ns = 0;
                host->audio_underruns++;
                return;
            }
            std::fill(std::begin(chunk), std::end(chunk), 0.0f);
            if (g_audio_buffer.size() >= kAudioChunk) g_audio_buffer.pop(chunk, kAudioChunk);
            if (g_addon_audio_buffer.size() >= kAudioChunk) {
                float addon[kAudioChunk * 2];
                g_addon_audio_buffer.pop(addon, kAudioChunk);
                QlMixStereo(chunk, addon, kAudioChunk);
            }
        }
        if (!host->audio_logged) {
            host->audio_logged = true;
            host->audio_stats_ns = now;
            QUEST_LINK_INFO("Mac audio: streaming to the headset (48 kHz stereo float32, 10 ms every 10 ms, 40 ms buffer)");
        }
        xrsp_send_audio(host, chunk, kAudioChunk, host->audio_next_ns);
        host->audio_next_ns += kAudioChunkNs;
        // Restarts and trims, at most once a minute and only when there were any.
        if (now - host->audio_stats_ns > 60LL * 1000000000 && (host->audio_underruns || host->audio_trims)) {
            QUEST_LINK_INFO("Mac audio: last minute %u restarts (ran dry), %u trims (backlog)",
                            host->audio_underruns, host->audio_trims);
            host->audio_underruns = host->audio_trims = 0;
            host->audio_stats_ns = now;
        }
    }
}

// QL_AUDIO_TEST_TONE: 2 s of 440 Hz at amplitude 0.1 (about -20 dBFS) with 20 ms fades,
// in 10 ms messages paced by the clock (write thread).
static void xrsp_audio_test_tone_tick(struct ql_xrsp_host *host)
{
    constexpr uint32_t kRate = 48000, kChunk = 480, kTotal = 2 * kRate, kFade = kRate / 50;
    if (!debug_get_bool_option_audio_test_tone() || !host->ready_to_send_frames) return;
    const int64_t now = xrsp_ts_ns(host);
    if (host->audio_tone_start_ns == 0) {
        host->audio_tone_start_ns = now;
        host->audio_tone_frames = 0;
        QUEST_LINK_INFO("AUDIO_TEST_TONE: sending 2 s of 440 Hz (48 kHz stereo float32, 10 ms messages)");
    }
    if (host->audio_tone_frames >= kTotal) return;
    const uint64_t due = (uint64_t)((now - host->audio_tone_start_ns) / 1000) * kRate / 1000000 + kRate / 50;
    float samples[kChunk * 2];
    while (host->audio_tone_frames < kTotal && host->audio_tone_frames < due) {
        for (uint32_t i = 0; i < kChunk; i++) {
            const uint64_t n = host->audio_tone_frames + i;
            float gain = 0.1f;
            if (n < kFade) gain *= (float)n / kFade;
            if (n + kFade > kTotal) gain *= (float)(kTotal - std::min<uint64_t>(n, kTotal)) / kFade;
            const float v = n < kTotal ? gain * sinf(2.0f * (float)M_PI * 440.0f * (float)n / kRate) : 0.0f;
            samples[i * 2] = samples[i * 2 + 1] = v;
        }
        xrsp_send_audio(host, samples, kChunk, host->audio_tone_start_ns + (int64_t)(host->audio_tone_frames * 1000000000ULL / kRate));
        host->audio_tone_frames += kChunk;
    }
    if (host->audio_tone_frames >= kTotal) QUEST_LINK_INFO("AUDIO_TEST_TONE: done (%llu frames)", (unsigned long long)host->audio_tone_frames);
}

// TODO: figure out the params
static void xrsp_send_input_control(struct ql_xrsp_host *host, uint16_t a, uint16_t b, uint32_t c, float d, float e)
{
    ::capnp::MallocMessageBuilder message;
    PayloadAudioControl::Builder msg = message.initRoot<PayloadAudioControl>();

    msg.setDataUnk0(a);
    msg.setDataUnk1(b);
    msg.setDataUnk2(c);
    msg.setDataUnk3(d);
    msg.setDataUnk4(e);

    kj::ArrayPtr<const kj::ArrayPtr<const capnp::word>> out = message.getSegmentsForOutput();
    xrsp_send_to_topic_capnp_segments(host, TOPIC_INPUT_CONTROL, 0, out);
}

// TODO: get this to work lol (is it possible for it to work...?)
static void xrsp_send_buffered_haptic(struct ql_xrsp_host *host, int64_t ts, ovr_haptic_target_t controller_id)
{
    if (host->pairing_state != PAIRINGSTATE_PAIRED || !host->ready_to_send_frames) {
        return;
    }
    ::capnp::MallocMessageBuilder message;
    PayloadHaptics::Builder msg = message.initRoot<PayloadHaptics>();

    msg.setTimestamp(ts); // TODO: idk what this timestamp is?
    msg.setInputType(controller_id);
    msg.setHapticType(OVR_HAPTIC_BUFFERED);
    msg.setDataUnk1p2(0x1919); // unk, usually 0
    msg.setDataUnk1p3(0x1919); // unk, usually 0
    msg.setAmplitude(1.0); // TODO idk if this is set for buffered
    msg.setPoseTimestamp(ts); // Timestamp identical to poseTimestamp in Slice, sometimes 0 if hapticType is simple?

    uint8_t* test_data = (uint8_t*)malloc(0x20);
    memset(test_data, 0xFF, 0x20);
    for (int i = 0; i < 0x20; i += 1) {
        test_data[i] = 0xFF;
    }

    // TODO where is this maximum defined? It seems hardcoded in XRSP tho.
    msg.setData(kj::arrayPtr(test_data, 0x19));

    kj::ArrayPtr<const kj::ArrayPtr<const capnp::word>> out = message.getSegmentsForOutput();
    xrsp_send_to_topic_capnp_segments(host, TOPIC_HAPTIC, 0, out);
    free(test_data);
}

void xrsp_send_simple_haptic(struct ql_xrsp_host *host, int64_t ts, ovr_haptic_target_t controller_id, float amplitude)
{
    if (host->pairing_state != PAIRINGSTATE_PAIRED || !host->ready_to_send_frames) {
        return;
    }
    ::capnp::MallocMessageBuilder message;
    PayloadHaptics::Builder msg = message.initRoot<PayloadHaptics>();

    msg.setTimestamp(ts);
    msg.setInputType(controller_id);
    msg.setHapticType(OVR_HAPTIC_SIMPLE);
    msg.setDataUnk1p2(0); // unk, usually 0
    msg.setDataUnk1p3(0); // unk, usually 0
    msg.setAmplitude(amplitude);
    msg.setPoseTimestamp(0);

    kj::ArrayPtr<const kj::ArrayPtr<const capnp::word>> out = message.getSegmentsForOutput();
    xrsp_send_to_topic_capnp_segments(host, TOPIC_HAPTIC, 0, out);
}

static void xrsp_send_video(struct ql_xrsp_host *host, int index, int slice_idx, int frame_idx, int64_t frame_started_ns, const uint8_t* csd_dat, size_t csd_len,
                            const uint8_t* video_dat, size_t video_len, int blit_y_pos)
{
    int64_t sending_pose_ns = host->stream_pose_ns[QL_IDX_SLICE(slice_idx, index)];
    int read_index = QL_IDX_SLICE(slice_idx, index);

    // Pause frame sending
    if (host->pairing_state != PAIRINGSTATE_PAIRED || !host->ready_to_send_frames)
    {
        host->tx_started_ns[read_index] = 0;
        host->tx_done_ns[read_index] = 0;
        host->tx_duration_ns[read_index] = 0;
        return;
    }

    ::capnp::MallocMessageBuilder message;
    PayloadSlice::Builder msg = message.initRoot<PayloadSlice>();

    struct ql_hmd* hmd = host->sys->hmd;

    struct xrt_pose sending_pose;
    U_ZERO(&sending_pose);

    static uint64_t last_roundtrip = 0;
    uint64_t roundtrip_ns = xrsp_ts_ns(host) - last_roundtrip;
    last_roundtrip = xrsp_ts_ns(host);

    uint64_t ts_before = xrsp_ts_ns(host);
    host->tx_started_ns[read_index] = ts_before;

    int bits = 0;
    if (csd_len > 0)
        bits |= 1;
    if (slice_idx == host->num_slices-1)
        bits |= 2;

    // Each slice owns the same frame snapshot. Reading slice 0 here could pick
    // a different frame when independent encoder workers finish out of order.
    sending_pose = host->stream_poses[read_index];

#if 0
    // If we haven't gotten a pose for a whole second, reset
    if (xrsp_ts_ns(host) - sending_pose_ns > 1000000000) {
        //host->ready_to_send_frames = false;
        libusb_reset_device(host->dev);
    }
#endif

    msg.setFrameIdx(frame_idx);
    msg.setUnk0p1(0);
    msg.setRectifyMeshId(QL_MESH_FOVEATED); // QL_MESH_NONE

    // TODO mutex

    // This pose belongs to these encoded pixels, allowing Quest timewarp to correct only subsequent motion.
    msg.setPoseQuatX(sending_pose.orientation.x);
    msg.setPoseQuatY(sending_pose.orientation.y);
    msg.setPoseQuatZ(sending_pose.orientation.z);
    msg.setPoseQuatW(sending_pose.orientation.w);
    msg.setPoseX(sending_pose.position.x);
    msg.setPoseY(sending_pose.position.y);
    msg.setPoseZ(sending_pose.position.z);

    // TODO this might include render time?
    uint64_t pipeline_pred_delta_ma = host->encode_done_ns[QL_IDX_SLICE(slice_idx, index)] - host->encode_started_ns[QL_IDX_SLICE(0, index)];//2916100;
    //uint64_t pipeline_pred_delta_ma = 0;
    //QUEST_LINK_INFO("%llu", pipeline_pred_delta_ma);

    // TODO maybe pull a round-trip delta time?
    uint64_t duration_a = (uint64_t)(1000000000.0/hmd->fps) /*+ 9116997*/; // 9ms this might also include the slice 0 pipeline_pred_delta_ma, but we don't include render time yet?
    uint64_t duration_c = pipeline_pred_delta_ma; // 4ms
    uint64_t duration_b = duration_a+duration_c; // 14ms
    uint64_t base_ts = xrsp_ts_ns_to_target(host, host->encode_started_ns[QL_IDX_SLICE(0, index)]);
    uint64_t tx_start_ts = host->tx_started_ns[QL_IDX_SLICE(0, index)];

    // all timestamps are all the same between different slices, only pipeline_pred_delta_ma changes
    msg.setPoseTimestamp(xrsp_ts_ns_to_target(host, sending_pose_ns));//xrsp_target_ts_ns(host)+41540173 // Deadline //18278312488115 // xrsp_ts_ns(host)
    msg.setSliceNum(slice_idx);
    msg.setUnk6p1(bits);
    msg.setUnk6p2(0);
    msg.setUnk6p3(0);
    msg.setBlitYPos((hmd->encode_height / host->num_slices) * slice_idx);
    msg.setCropBlocks((hmd->encode_height/16) / host->num_slices); // 24 for slice count 5

    /*
    unk0p0 = 74,
  unk0p1 = 0,
  unk1p0 = 1000,
  poseQuatX = -0.50216717,
  poseQuatY = 0.10189699,
  poseQuatZ = -0.093296453,
  poseQuatW = -0.85366327,
  poseX = 0.010952883,
  poseY = 0.17921059,
  poseZ = 0.18543391,
  poseTimestamp = 18789777081583,
  sliceNum = 4,
  unk6p1 = 2,
  unk6p2 = 0,
  unk6p3 = 0,
  blitYPos = 1536,
  unk7p0 = 24,
  csdSize = 0,
  videoSize = 1387,
  unk8p1 = 0,
  timestamp09 = 18789735622294,
  unkA = 5472800,
  timestamp0B = 18789764254886,
  timestamp0C = 18789759255729,
  timestamp0D = 18789744739291,


    unkA = 2916100 ... 5472800,    2.92ms ... 5.47ms

  timestamp09   = 18789735622294, +0          0.00ms   +0          0.00ms       transmission start?? more likely, encoding start?
  timestamp0D   = 18789744739291, +9116997    9.11ms   +9116997    9.11ms       estimated GPU end
  timestamp0C   = 18789759255729, +14516438  14.51ms   +23633435   23.63ms      deadline?
  timestamp0B   = 18789764254886, +4999157    4.99ms   +28632592   28.62ms      unknown B
  poseTimestamp = 18789777081583, +12826697  12.82ms   +41459289   41.45ms      predicted pose

  ( unk0 = 0,
      timestampUs = 18789759065,
      data = "Frame 74 decoded, delta in prediction time: 0.000000ms" ),
    ( unk0 = 0,
      timestampUs = 18789759084,
      data = "Glitches: 1, Mispredicts: 44, Deadline: 12.06ms, Transmission:  3.02ms, Pipeline Prediction Delta M" ),
    ( unk0 = 0,
      timestampUs = 18789759180,
      data = "Rectify: Frame using rectify meshId = 1000" ) ] )
    */

    //
    msg.setUnk8p1(0);
    msg.setTimestamp09(xrsp_ts_ns_to_target(host, tx_start_ts)-pipeline_pred_delta_ma);// transmission start
    msg.setUnkA(pipeline_pred_delta_ma); // pipeline prediction delta MA?
    msg.setTimestamp0B(base_ts+duration_a+duration_b+duration_c); // unknown
    msg.setTimestamp0C(base_ts+duration_a+duration_b); // deadline
    msg.setTimestamp0D(base_ts+duration_a); // unknown
    //QUEST_LINK_INFO("%x", host->ns_offset);

    // left eye orientation? for foveated compression weirdness?
    msg.getQuat1().setX(0.0);
    msg.getQuat1().setY(0.0);
    msg.getQuat1().setZ(0.0);
    msg.getQuat1().setW(0.0);

    // right eye orientation? for foveated compression weirdness?
    msg.getQuat1().setX(0.0);
    msg.getQuat2().setY(0.0);
    msg.getQuat2().setZ(0.0);
    msg.getQuat2().setW(0.0);

    msg.setCsdSize(csd_len);
    msg.setVideoSize(video_len);

    kj::ArrayPtr<const kj::ArrayPtr<const capnp::word>> out = message.getSegmentsForOutput();

    // The first frame of every session *must* be a keyframe with a CSD.
    int should_send = 1;
    if (!csd_len && !host->sent_first_frame)
    {
        should_send = 0;
    }

    if (should_send)
    {
        xrsp_send_to_topic_capnp_segments(host, TOPIC_SLICE_0+slice_idx, 0, out);

        if (csd_len)
            xrsp_send_to_topic(host, TOPIC_SLICE_0+slice_idx, csd_dat, csd_len);

        xrsp_send_to_topic(host, TOPIC_SLICE_0+slice_idx, video_dat, video_len);

        if (!host->sent_first_frame && slice_idx == 0) {
            xrsp_ripc_void_bool_cmd(host, host->client_id, "EnableEyeTrackingForPCLink");
        }
        host->sent_first_frame = true;
    }

    uint64_t ts_after = xrsp_ts_ns(host);
    host->tx_done_ns[read_index] = ts_after;

    int64_t ts_diff = ts_after - ts_before;
    host->tx_duration_ns[read_index] = ts_diff;

    // TODO: ehhhhh
    //xrsp_ripc_void_bool_cmd(host, host->client_id, "EnableEyeTrackingForPCLink");
    //xrsp_ripc_void_bool_cmd(host, host->client_id, "EnableFaceTrackingForPCLink");
    //xrsp_ripc_void_bool_cmd(host, host->client_id, "SendSwitchToHandsNotif");
    //xrsp_ripc_void_bool_cmd(host, host->client_id, "SystemButtonEventRequest");
}

static void *
ql_xrsp_read_thread(void *ptr)
{
    DRV_TRACE_MARKER();

    struct ql_xrsp_host *host = (struct ql_xrsp_host *)ptr;

    os_thread_helper_lock(&host->read_thread);
    while (os_thread_helper_is_running_locked(&host->read_thread)) {
        os_thread_helper_unlock(&host->read_thread);

        if (xrsp_ts_ns(host) - host->last_read_ns > 1000000000 && host->pairing_state == PAIRINGSTATE_WAIT_FIRST && !host->usb_valid)
        {
            ql_xrsp_usb_init(host, false);
            host->last_read_ns = xrsp_ts_ns(host);
        }

        xrsp_camera_probe_tick(host);
        xrsp_read_usb(host);

        os_thread_helper_lock(&host->read_thread);
        if (os_thread_helper_is_running_locked(&host->read_thread)) {
            os_nanosleep(U_TIME_1MS_IN_NS / 10);
        }
    }
    os_thread_helper_unlock(&host->read_thread);

    xrsp_camera_probe_tick(host, true);
    QUEST_LINK_DEBUG("Exiting packet reading thread");

    return NULL;
}

static void *
ql_xrsp_write_thread(void *ptr)
{
    DRV_TRACE_MARKER();

    struct ql_xrsp_host *host = (struct ql_xrsp_host *)ptr;

    os_thread_helper_lock(&host->write_thread);
    while (os_thread_helper_is_running_locked(&host->write_thread)) {
        os_thread_helper_unlock(&host->write_thread);



        int64_t present_ns = 0x7FFFFFFFFFFFFFFF;
        int to_present = -1;
        for (int i = 0; i < QL_SWAPCHAIN_DEPTH; i++)
        {
            bool all_slices_present = true;
            for (int j = 0; j < QL_NUM_SLICES; j++)
            {
                int full_idx = QL_IDX_SLICE(j, i);
                os_mutex_lock(&host->stream_mutex[full_idx]);
                //QUEST_LINK_INFO("%x %zx %zx", host->needs_flush[i], host->stream_started_ns[i], present_ns);
                if (!host->needs_flush[full_idx]) {
                    all_slices_present = false;
                }
                os_mutex_unlock(&host->stream_mutex[full_idx]);
            }

            int first_idx = QL_IDX_SLICE(0, i);
            os_mutex_lock(&host->stream_mutex[first_idx]);
            //QUEST_LINK_INFO("%x %zx %zx", host->needs_flush[i], host->stream_started_ns[i], present_ns);
            if (all_slices_present && host->stream_started_ns[first_idx] < present_ns) {
                present_ns = host->stream_started_ns[first_idx];
                to_present = i;
            }
            os_mutex_unlock(&host->stream_mutex[first_idx]);
        }

        // TODO: merge frames together if needed
        if (to_present >= 0) { //  && (xrsp_ts_ns(host) - host->frame_sent_ns) >= 13890000

            for (int slice = 0; slice < QL_NUM_SLICES; slice++)
            {
                int to_present_idx = QL_IDX_SLICE(slice, to_present);
                os_mutex_lock(&host->stream_mutex[to_present_idx]);
                //QUEST_LINK_INFO("Flush: %x %x", slice, to_present);

                if (host->csd_stream_len[to_present_idx] || host->idr_stream_len[to_present_idx])
                    xrsp_send_video(host, to_present, slice, host->frame_idx, present_ns, (const uint8_t*)host->csd_stream[to_present_idx], host->csd_stream_len[to_present_idx], (const uint8_t*)host->idr_stream[to_present_idx], host->idr_stream_len[to_present_idx], 0);

                if (!slice)
                    host->frame_sent_ns = xrsp_ts_ns(host);

                host->csd_stream_len[to_present_idx] = 0;
                host->idr_stream_len[to_present_idx] = 0;
                host->needs_flush[to_present_idx] = false;

                //QUEST_LINK_INFO("Flush: %x", host->stream_read_idx);
                os_mutex_unlock(&host->stream_mutex[to_present_idx]);
            }
            host->frame_idx++;
        }



        //QUEST_LINK_INFO("%zx", xrsp_ts_ns(host) - host->paired_ns);
        if (xrsp_ts_ns(host) - host->paired_ns > 1000000000 && host->pairing_state == PAIRINGSTATE_PAIRED && !host->ready_to_send_frames) // && xrsp_ts_ns(host) - host->frame_sent_ns >= 16000000
        {
            host->ready_to_send_frames = true;
            host->sent_first_frame = false;

            for (int i = 0; i < QL_NUM_SLICES*QL_SWAPCHAIN_DEPTH; i++)
            {
                host->csd_stream_len[i] = 0;
                host->idr_stream_len[i] = 0;
                host->needs_flush[i] = false;
            }
        }

        //QUEST_LINK_INFO("%zx", xrsp_ts_ns(host) - host->last_read_ns);
        if (xrsp_ts_ns(host) - host->last_read_ns > 1000000000 && host->pairing_state == PAIRINGSTATE_WAIT_FIRST && host->usb_valid)
        {
            xrsp_trigger_bye(host, NULL);
            //xrsp_init_session_bye(host);
            //ql_xrsp_usb_init(host, true);
            host->last_read_ns = xrsp_ts_ns(host);
        }

        xrsp_audio_test_tone_tick(host);
        xrsp_audio_drain(host);

        if (host->sys) {
            struct ql_hmd* hmd = host->sys->hmd;

            if (hmd && xrsp_ts_ns(host) - hmd->pose_ns > 1000000000) {
                host->is_inactive = true;
            }
        }

        os_thread_helper_lock(&host->write_thread);

        if (os_thread_helper_is_running_locked(&host->write_thread)) {
            os_nanosleep(U_TIME_1MS_IN_NS / 10);
        }
    }
    os_thread_helper_unlock(&host->write_thread);

    {
        std::lock_guard<std::mutex> lock(g_audio_mutex);
        g_audio_paired = false;
        g_audio_buffer.clear();
    }
    QUEST_LINK_DEBUG("Exiting packet writing thread");

    return NULL;
}
