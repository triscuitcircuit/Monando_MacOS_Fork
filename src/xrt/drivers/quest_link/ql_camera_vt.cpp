// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  quest_link camera HEVC decode test with VideoToolbox (diagnostic only)
 * @ingroup drv_quest_link
 */

#include "ql_camera_vt.h"

#include <CoreMedia/CoreMedia.h>
#include <CoreVideo/CoreVideo.h>
#include <VideoToolbox/VideoToolbox.h>
#include <dispatch/dispatch.h>

#include <atomic>
#include <mutex>
#include <vector>
#include <deque>

#include "os/os_time.h"

#include "ql_camera_hevc.h"
#include "ql_system.h"

namespace {


constexpr int kMaxInFlight = 30;


constexpr int kMaxQueued = 8;

} // namespace


namespace {
std::mutex g_frames_mutex;
struct passthrough_frames
{
    ql_camera_vt *owner = nullptr;
    struct pending_frame { CVPixelBufferRef image; uint64_t id; };
    std::deque<pending_frame> pending[2];
    CVPixelBufferRef pair[2] = {};
    uint64_t pair_seq = 0;
    uint64_t pair_capture_ns = 0;
    int64_t last_pair_ns = 0;
} g_frames;

void
release_frames_locked()
{
    for (int i = 0; i < 2; i++) {
        for (const auto& frame : g_frames.pending[i]) CVPixelBufferRelease(frame.image);
        g_frames.pending[i].clear();
        if (g_frames.pair[i] != nullptr) CVPixelBufferRelease(g_frames.pair[i]);
        g_frames.pair[i] = nullptr;
    }
    g_frames.pair_capture_ns = 0;
    g_frames.last_pair_ns = 0;
}

struct vt_stream
{
    int id = 0;
    ql_camera_vt *owner = nullptr;
    bool keep_frames = false;
    CMVideoFormatDescriptionRef format = nullptr;
    VTDecompressionSessionRef session = nullptr;
    std::vector<uint8_t> vps, sps, pps;
    std::atomic<bool> waiting_for_irap{true};
    bool skip_rasl = false;
    std::atomic<int> in_flight{0};

    std::mutex mutex;
    uint64_t submitted = 0;
    uint64_t decoded = 0;
    uint64_t decode_errors = 0;
    uint64_t submit_errors = 0;
    uint64_t dropped_waiting = 0;
    uint64_t dropped_backlog = 0;
    uint64_t dropped_rasl = 0;
    uint64_t malformed = 0;
    uint64_t irap_frames = 0;
    uint64_t sessions_created = 0;
    int64_t decode_ns_total = 0;
    int64_t decode_ns_max = 0;
    bool first_output_logged = false;
    uint64_t waiting_frames = 0;
    int64_t stall_start_ns = 0;
    uint64_t gaps = 0;
    uint64_t resumes = 0;
};


constexpr uint64_t kStallFrames = 5;

void
begin_wait_locked(vt_stream *s, int64_t now)
{
    s->waiting_for_irap = true;
    if (s->stall_start_ns == 0) s->stall_start_ns = now;
}

void
fourcc(OSType type, char out[5])
{
    out[0] = (char)(type >> 24);
    out[1] = (char)(type >> 16);
    out[2] = (char)(type >> 8);
    out[3] = (char)type;
    out[4] = '\0';
}

void
publish_frame(vt_stream *s, CVImageBufferRef image, uint64_t frame_id)
{
    std::lock_guard<std::mutex> lock(g_frames_mutex);
    if (g_frames.owner != s->owner || frame_id <= g_frames.pair_capture_ns) return;
    auto& pending = g_frames.pending[s->id];
    for (const auto& frame : pending) if (frame.id == frame_id) return;
    pending.push_back({CVPixelBufferRetain(image), frame_id});

    if (pending.size() > 4) { CVPixelBufferRelease(pending.front().image); pending.pop_front(); }
    CVPixelBufferRef matched[2]{};
    matched[s->id] = image;
    for (const auto& frame : g_frames.pending[1 - s->id])
        if (frame.id == frame_id) { matched[1 - s->id] = frame.image; break; }
    if (matched[0] && matched[1]) {
        for (int i = 0; i < 2; i++) {
            if (g_frames.pair[i] != nullptr) CVPixelBufferRelease(g_frames.pair[i]);
            g_frames.pair[i] = CVPixelBufferRetain(matched[i]);
            auto& frames = g_frames.pending[i];
            for (auto it = frames.begin(); it != frames.end();) {
                if (it->id <= frame_id) { CVPixelBufferRelease(it->image); it = frames.erase(it); }
                else ++it;
            }
        }
        g_frames.pair_capture_ns = frame_id;
        g_frames.last_pair_ns = os_monotonic_get_ns();
        g_frames.pair_seq++;
    }
}

void
on_decoded(void *stream_refcon,
           void *frame_refcon,
           OSStatus status,
           VTDecodeInfoFlags /*flags*/,
           CVImageBufferRef image,
           CMTime pts,
           CMTime /*duration*/)
{
    vt_stream *s = (vt_stream *)stream_refcon;
    const int64_t elapsed = os_monotonic_get_ns() - (int64_t)(uintptr_t)frame_refcon;
    s->in_flight.fetch_sub(1);
    if (s->keep_frames && status == noErr && image != nullptr && CMTIME_IS_NUMERIC(pts)) {
        publish_frame(s, image, (uint64_t)pts.value);
    }

    std::lock_guard<std::mutex> lock(s->mutex);
    if (status != noErr || image == nullptr) {
        s->decode_errors++;
        s->gaps++;
        begin_wait_locked(s, os_monotonic_get_ns());
        if (s->decode_errors <= 3) {
            QUEST_LINK_ERROR("CAMERA_DECODE stream %d: decode failed, OSStatus %d", s->id, (int)status);
        }
        return;
    }
    s->decoded++;
    s->decode_ns_total += elapsed;
    if (elapsed > s->decode_ns_max) s->decode_ns_max = elapsed;
    if (!s->first_output_logged) {
        s->first_output_logged = true;
        char format[5];
        fourcc(CVPixelBufferGetPixelFormatType(image), format);
        QUEST_LINK_INFO("CAMERA_DECODE stream %d: first decoded frame %zux%zu pixel format '%s' planes=%zu (%.2f ms)",
                        s->id, CVPixelBufferGetWidth(image), CVPixelBufferGetHeight(image), format,
                        CVPixelBufferGetPlaneCount(image), elapsed / 1e6);
    }
}

void
destroy_session(vt_stream *s)
{
    if (s->session != nullptr) {
        VTDecompressionSessionWaitForAsynchronousFrames(s->session);
        VTDecompressionSessionInvalidate(s->session);
        CFRelease(s->session);
        s->session = nullptr;
    }
    if (s->format != nullptr) {
        CFRelease(s->format);
        s->format = nullptr;
    }
}

bool
create_session(vt_stream *s)
{
    destroy_session(s);

    const uint8_t *sets[3] = {s->vps.data(), s->sps.data(), s->pps.data()};
    const size_t sizes[3] = {s->vps.size(), s->sps.size(), s->pps.size()};
    OSStatus status = CMVideoFormatDescriptionCreateFromHEVCParameterSets(kCFAllocatorDefault, 3, sets, sizes, 4,
                                                                          nullptr, &s->format);
    if (status != noErr) {
        QUEST_LINK_ERROR("CAMERA_DECODE stream %d: format description from VPS/SPS/PPS failed, OSStatus %d", s->id,
                         (int)status);
        s->format = nullptr;
        return false;
    }
    const CMVideoDimensions dims = CMVideoFormatDescriptionGetDimensions(s->format);

    VTDecompressionOutputCallbackRecord callback = {on_decoded, s};
    CFDictionaryRef destination = nullptr;
    if (s->keep_frames) {
        const int32_t bgra = kCVPixelFormatType_32BGRA;
        CFNumberRef format = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &bgra);
        CFDictionaryRef iosurface = CFDictionaryCreate(kCFAllocatorDefault, nullptr, nullptr, 0,
                                                       &kCFTypeDictionaryKeyCallBacks,
                                                       &kCFTypeDictionaryValueCallBacks);
        const void *keys[] = {kCVPixelBufferPixelFormatTypeKey, kCVPixelBufferIOSurfacePropertiesKey,
                              kCVPixelBufferMetalCompatibilityKey};
        const void *values[] = {format, iosurface, kCFBooleanTrue};
        destination = CFDictionaryCreate(kCFAllocatorDefault, keys, values, 3, &kCFTypeDictionaryKeyCallBacks,
                                         &kCFTypeDictionaryValueCallBacks);
        CFRelease(format);
        CFRelease(iosurface);
    }
    status = VTDecompressionSessionCreate(kCFAllocatorDefault, s->format, nullptr, destination, &callback,
                                          &s->session);
    if (destination != nullptr) CFRelease(destination);
    if (status != noErr) {
        QUEST_LINK_ERROR("CAMERA_DECODE stream %d: VTDecompressionSessionCreate failed, OSStatus %d", s->id,
                         (int)status);
        s->session = nullptr;
        return false;
    }

    CFBooleanRef hardware = nullptr;
    VTSessionCopyProperty(s->session, kVTDecompressionPropertyKey_UsingHardwareAcceleratedVideoDecoder,
                          kCFAllocatorDefault, &hardware);
    {
        std::lock_guard<std::mutex> lock(s->mutex);
        s->sessions_created++;
    }
    QUEST_LINK_INFO("CAMERA_DECODE stream %d: session created for %dx%d (VPS %zu B, SPS %zu B, PPS %zu B), "
                    "hardware decoder=%s",
                    s->id, dims.width, dims.height, s->vps.size(), s->sps.size(), s->pps.size(),
                    hardware == nullptr ? "unknown" : (CFBooleanGetValue(hardware) ? "yes" : "no"));
    if (hardware != nullptr) CFRelease(hardware);
    return true;
}

} // namespace

struct ql_camera_vt
{
    vt_stream streams[2];
    dispatch_queue_t queue = nullptr;
    std::atomic<int> queued{0};
};

namespace {

struct vt_job
{
    ql_camera_vt *vt;
    int stream;
    bool gap;
    uint64_t frame_id;
    std::vector<uint8_t> data;
};

void
mark_gap(vt_stream *s)
{
    std::lock_guard<std::mutex> lock(s->mutex);
    s->gaps++;
    begin_wait_locked(s, os_monotonic_get_ns());
}

void
decode(vt_stream *s, const uint8_t *data, size_t len, uint64_t frame_id)
{

    ql_hevc_access_unit unit;
    if (!ql_hevc_from_chunks(data, len, &unit) || unit.vcl_count == 0) {
        std::lock_guard<std::mutex> lock(s->mutex);
        s->malformed++;
        return;
    }

    if (!unit.vps.empty() && !unit.sps.empty() && !unit.pps.empty() &&
        (unit.vps != s->vps || unit.sps != s->sps || unit.pps != s->pps || s->session == nullptr)) {
        s->vps = unit.vps;
        s->sps = unit.sps;
        s->pps = unit.pps;
        create_session(s);
        std::lock_guard<std::mutex> lock(s->mutex);
        begin_wait_locked(s, os_monotonic_get_ns());
    }

    {
        std::lock_guard<std::mutex> lock(s->mutex);
        if (unit.irap) s->irap_frames++;
    }
    if (s->session == nullptr || (s->waiting_for_irap && !unit.irap)) {
        std::lock_guard<std::mutex> lock(s->mutex);
        s->dropped_waiting++;
        s->waiting_frames++;
        if (s->stall_start_ns == 0) s->stall_start_ns = os_monotonic_get_ns();
        if (s->waiting_frames == kStallFrames) {
            QUEST_LINK_ERROR("CAMERA_DECODE stream %d: stalled, %llu frames without a usable keyframe (%s)", s->id,
                             (unsigned long long)s->waiting_frames,
                             s->session == nullptr ? "no parameter sets yet" : "after a gap");
        }
        return;
    }
    if (s->in_flight.load() >= kMaxInFlight) {
        std::lock_guard<std::mutex> lock(s->mutex);
        s->dropped_backlog++;
        s->gaps++;
        begin_wait_locked(s, os_monotonic_get_ns());
        return;
    }
    if (unit.irap) {
        s->skip_rasl = s->waiting_for_irap && unit.cra_or_bla;
    } else if (unit.rasl && s->skip_rasl) {
        std::lock_guard<std::mutex> lock(s->mutex);
        s->dropped_rasl++;
        return;
    }
    if (s->waiting_for_irap) {
        std::lock_guard<std::mutex> lock(s->mutex);
        if (s->stall_start_ns != 0 && (s->gaps > 0 || s->resumes > 0 || s->waiting_frames > 0)) {
            s->resumes++;
            QUEST_LINK_INFO("CAMERA_RECOVERY stream %d: keyframe received %.0f ms after the wait began, "
                            "%llu frames skipped; decoding resumes",
                            s->id, (os_monotonic_get_ns() - s->stall_start_ns) / 1e6,
                            (unsigned long long)s->waiting_frames);
        }
        s->stall_start_ns = 0;
        s->waiting_frames = 0;
        s->waiting_for_irap = false;
    }

    CMBlockBufferRef block = nullptr;
    OSStatus status = CMBlockBufferCreateWithMemoryBlock(kCFAllocatorDefault, nullptr, unit.sample.size(),
                                                         kCFAllocatorDefault, nullptr, 0, unit.sample.size(),
                                                         kCMBlockBufferAssureMemoryNowFlag, &block);
    if (status == noErr) {
        status = CMBlockBufferReplaceDataBytes(unit.sample.data(), block, 0, unit.sample.size());
    }
    CMSampleBufferRef sample = nullptr;
    const size_t sample_size = unit.sample.size();
    const CMSampleTimingInfo timing = {kCMTimeInvalid, CMTimeMake((int64_t)frame_id, 1), kCMTimeInvalid};
    if (status == noErr) {
        status = CMSampleBufferCreateReady(kCFAllocatorDefault, block, s->format, 1, 1, &timing, 1, &sample_size,
                                           &sample);
    }
    if (status == noErr) {
        s->in_flight.fetch_add(1);
        const int64_t now = os_monotonic_get_ns();
        status = VTDecompressionSessionDecodeFrame(s->session, sample, kVTDecodeFrame_EnableAsynchronousDecompression,
                                                   (void *)(uintptr_t)now, nullptr);
        if (status != noErr) s->in_flight.fetch_sub(1);
    }
    if (sample != nullptr) CFRelease(sample);
    if (block != nullptr) CFRelease(block);

    std::lock_guard<std::mutex> lock(s->mutex);
    if (status == noErr) {
        s->submitted++;
    } else {
        s->submit_errors++;
        s->gaps++;
        begin_wait_locked(s, os_monotonic_get_ns());
        if (s->submit_errors <= 3) {
            QUEST_LINK_ERROR("CAMERA_DECODE stream %d: submit failed, OSStatus %d", s->id, (int)status);
        }
    }
}

void
run_job(void *context)
{
    vt_job *job = (vt_job *)context;
    vt_stream *s = &job->vt->streams[job->stream];
    if (job->gap) {
        mark_gap(s);
    } else {
        decode(s, job->data.data(), job->data.size(), job->frame_id);
        job->vt->queued.fetch_sub(1);
    }
    delete job;
}

void
drain(void *) {}

} // namespace

struct ql_camera_vt *
ql_camera_vt_create(bool keep_frames)
{
    ql_camera_vt *vt = new ql_camera_vt();
    for (int i = 0; i < 2; i++) {
        vt->streams[i].id = i;
        vt->streams[i].owner = vt;
        vt->streams[i].keep_frames = keep_frames;
    }
    vt->queue = dispatch_queue_create("quest_link.camera_decode", DISPATCH_QUEUE_SERIAL);
    if (keep_frames) {
        std::lock_guard<std::mutex> lock(g_frames_mutex);
        release_frames_locked();
        g_frames.owner = vt;
    }
    return vt;
}

void
ql_camera_vt_submit(struct ql_camera_vt *vt, int stream, const uint8_t *data, size_t len, uint64_t frame_id)
{
    if (vt == nullptr || stream < 0 || stream > 1) return;
    if (vt->queued.load() >= kMaxQueued) {
        dispatch_async_f(vt->queue, new vt_job{vt, stream, true, 0, {}}, run_job);
        return;
    }
    vt->queued.fetch_add(1);
    dispatch_async_f(vt->queue, new vt_job{vt, stream, false, frame_id, std::vector<uint8_t>(data, data + len)},
                     run_job);
}

void
ql_camera_vt_mark_gap(struct ql_camera_vt *vt, int stream)
{
    if (vt == nullptr || stream < 0 || stream > 1) return;
    dispatch_async_f(vt->queue, new vt_job{vt, stream, true, 0, {}}, run_job);
}

bool
ql_camera_vt_stalled(struct ql_camera_vt *vt)
{
    if (vt == nullptr) return false;
    for (vt_stream &s : vt->streams) {
        std::lock_guard<std::mutex> lock(s.mutex);
        if (s.waiting_for_irap && s.waiting_frames >= kStallFrames) return true;
    }
    std::lock_guard<std::mutex> lock(g_frames_mutex);
    if (g_frames.owner == vt && g_frames.last_pair_ns > 0 &&
        os_monotonic_get_ns() - g_frames.last_pair_ns > 2'000'000'000LL) return true;
    return false;
}

void
ql_camera_vt_log_summary(struct ql_camera_vt *vt)
{
    if (vt == nullptr) return;
    dispatch_sync_f(vt->queue, nullptr, drain);
    for (vt_stream &s : vt->streams) {
        std::lock_guard<std::mutex> lock(s.mutex);
        QUEST_LINK_INFO("CAMERA_DECODE stream %d summary: submitted=%llu decoded=%llu decode_errors=%llu "
                        "submit_errors=%llu dropped_waiting_irap=%llu dropped_backlog=%llu dropped_rasl=%llu malformed=%llu "
                        "irap_frames=%llu sessions=%llu gaps=%llu resumes=%llu decode avg %.2f ms max %.2f ms in_flight=%d",
                        s.id, (unsigned long long)s.submitted, (unsigned long long)s.decoded,
                        (unsigned long long)s.decode_errors, (unsigned long long)s.submit_errors,
                        (unsigned long long)s.dropped_waiting, (unsigned long long)s.dropped_backlog,
                        (unsigned long long)s.dropped_rasl,
                        (unsigned long long)s.malformed, (unsigned long long)s.irap_frames,
                        (unsigned long long)s.sessions_created, (unsigned long long)s.gaps,
                        (unsigned long long)s.resumes,
                        s.decoded ? s.decode_ns_total / (double)s.decoded / 1e6 : 0.0, s.decode_ns_max / 1e6,
                        s.in_flight.load());
    }
}

void
ql_camera_vt_destroy(struct ql_camera_vt *vt)
{
    if (vt == nullptr) return;
    dispatch_sync_f(vt->queue, nullptr, drain);
    for (vt_stream &s : vt->streams) destroy_session(&s);
    {
        std::lock_guard<std::mutex> lock(g_frames_mutex);
        if (g_frames.owner == vt) {
            g_frames.owner = nullptr;
            release_frames_locked();
        }
    }
    dispatch_release(vt->queue);
    delete vt;
}

void
ql_camera_vt_clear_frames(void)
{
    std::lock_guard<std::mutex> lock(g_frames_mutex);
    release_frames_locked();
}

extern "C" __attribute__((visibility("default"))) bool
ql_passthrough_copy_latest_ex(uint64_t *io_seq, void **out_left, void **out_right, int64_t *out_capture_ns)
{
    if (io_seq == nullptr || out_left == nullptr || out_right == nullptr) return false;
    std::lock_guard<std::mutex> lock(g_frames_mutex);
    if (g_frames.pair_seq == *io_seq || g_frames.pair[0] == nullptr || g_frames.pair[1] == nullptr) return false;
    *io_seq = g_frames.pair_seq;
    *out_left = CVPixelBufferRetain(g_frames.pair[0]);
    *out_right = CVPixelBufferRetain(g_frames.pair[1]);
    if (out_capture_ns != nullptr) *out_capture_ns = (int64_t)g_frames.pair_capture_ns;
    return true;
}

extern "C" __attribute__((visibility("default"))) bool
ql_passthrough_copy_latest(uint64_t *io_seq, void **out_left, void **out_right)
{
    return ql_passthrough_copy_latest_ex(io_seq, out_left, out_right, nullptr);
}
