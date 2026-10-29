// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  quest_link XRSP camera stream metadata (diagnostic only)
 * @ingroup drv_quest_link
 */

#include "ql_xrsp_camera.h"

#include <stdio.h>

#include "os/os_time.h"
#include "util/u_debug.h"
#include "util/u_json.h"

#include "ql_camera_stream.h"
#include "ql_passthrough_camera.h"
#include <algorithm>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>
#ifdef __APPLE__
#include "ql_camera_vt.h"
#endif
#include "ql_system.h"
#include "ql_utils.h"

#include <capnp/any.h>
#include <capnp/message.h>
#include "protos/CameraStream.capnp.h"

DEBUG_GET_ONCE_BOOL_OPTION(camera_decode, "QL_CAMERA_DECODE", false)
DEBUG_GET_ONCE_BOOL_OPTION(camera_probe_logs, "QL_CAMERA_PROBE", false)
DEBUG_GET_ONCE_NUM_OPTION(camera_drop_test, "QL_CAMERA_DROP_TEST", 0)
DEBUG_GET_ONCE_BOOL_OPTION(float_probe, "QL_FLOAT_PROBE", false)
DEBUG_GET_ONCE_NUM_OPTION(float_probe_seconds, "QL_FLOAT_PROBE_SECONDS", 90)

static void
store_float_frame(const capnp::List<CameraMetadata>::Reader &metas, const kj::ArrayPtr<const kj::byte> *data,
                  uint32_t image_count, double timestamp_s);
extern "C" void
ql_xrsp_camera_clear_float(void);

namespace {

constexpr uint64_t kDetailedMessages = 4;
constexpr int64_t kSummaryPeriodNs = 2000000000LL;

struct camera_stats
{
    ql_camera_reassembler reassembler;
    uint64_t messages = 0;
    uint64_t frames = 0;
    uint64_t meta_messages = 0;
    uint64_t other_messages = 0;
    uint64_t parse_failures = 0;
    uint64_t color_frames_described = 0;
    uint64_t color_frames = 0;
    uint64_t counter_last[2] = {};
    bool counter_seen[2] = {};
    uint64_t counter_steps[2][3] = {};
    uint64_t image_bytes = 0;
    uint64_t period_frames = 0;
    uint64_t period_image_bytes = 0;
    int64_t period_start_ns = 0;
    double first_timestamp = 0;
    double last_timestamp = 0;
    bool passthrough = false;
    int timing_logs = 0;
    const ql_xrsp_host *host = nullptr;
    double last_color_timestamp = 0;
    int64_t float_probe_start_ns = 0, float_probe_last_ns = 0;
    bool float_probe_done = false;
#ifdef __APPLE__
    ql_camera_vt *vt = nullptr;

    ~camera_stats()
    {
        ql_camera_vt_destroy(vt);
    }
#endif
};

void
describe_bitstream(const capnp::List<uint8_t>::Reader &data, char *out, size_t out_len)
{
    size_t start = 0;
    if (data.size() >= 4 && data[0] == 0 && data[1] == 0 && data[2] == 0 && data[3] == 1) {
        start = 4;
    } else if (data.size() >= 3 && data[0] == 0 && data[1] == 0 && data[2] == 1) {
        start = 3;
    }
    if (start == 0 || data.size() <= start) {
        snprintf(out, out_len, "no Annex B start code");
        return;
    }
    const uint8_t nal = data[start];
    snprintf(out, out_len, "Annex B start=%zu, nal type hevc=%u avc=%u", start, (nal >> 1) & 0x3f, nal & 0x1f);
}

void
hex_prefix(const uint8_t *data, size_t len, size_t max, char *out, size_t out_len)
{
    size_t used = 0;
    out[0] = '\0';
    for (size_t i = 0; i < len && i < max && used + 3 < out_len; i++) {
        used += (size_t)snprintf(out + used, out_len - used, "%s%02x", i ? " " : "", data[i]);
    }
}


void
describe_length_prefixed(const uint8_t *data, size_t len, char *out, size_t out_len)
{
    for (int big_endian = 1; big_endian >= 0; big_endian--) {
        size_t at = 0;
        size_t units = 0;
        size_t used = (size_t)snprintf(out, out_len, "%s u32 length prefix:", big_endian ? "BE" : "LE");
        while (at + 4 <= len && units < 64) {
            const uint32_t n = big_endian ? ((uint32_t)data[at] << 24 | (uint32_t)data[at + 1] << 16 |
                                             (uint32_t)data[at + 2] << 8 | data[at + 3])
                                          : ((uint32_t)data[at + 3] << 24 | (uint32_t)data[at + 2] << 16 |
                                             (uint32_t)data[at + 1] << 8 | data[at]);
            if (n == 0 || n > len - at - 4) break;
            if (units < 8 && used + 24 < out_len) {
                used += (size_t)snprintf(out + used, out_len - used, " [hevc type %u, %u B]",
                                         (data[at + 4] >> 1) & 0x3f, n);
            }
            at += 4 + n;
            units++;
        }
        if (units > 0 && at == len) {
            if (used + 32 < out_len) snprintf(out + used, out_len - used, " -> %zu units tile exactly", units);
            return;
        }
    }
    snprintf(out, out_len, "not 4-byte length-prefixed (BE or LE)");
}

void
describe_start_codes(const uint8_t *data, size_t len, char *out, size_t out_len)
{
    size_t used = (size_t)snprintf(out, out_len, "start codes:");
    size_t found = 0;
    for (size_t i = 0; i + 3 < len && found < 8; i++) {
        if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1) {
            if (used + 32 < out_len) {
                used += (size_t)snprintf(out + used, out_len - used, " @%zu(type %u)", i,
                                         (data[i + 3] >> 1) & 0x3f);
            }
            found++;
            i += 2;
        }
    }
    if (found == 0) snprintf(out + used, out_len - used, " none");
}

void
log_bitstream_structure(const char *what, const uint8_t *data, size_t len)
{
    char hex[64], prefixed[320], starts[200];
    hex_prefix(data, len, 16, hex, sizeof(hex));
    describe_length_prefixed(data, len, prefixed, sizeof(prefixed));
    describe_start_codes(data, len, starts, sizeof(starts));
    QUEST_LINK_INFO("CAMERA_BITSTREAM %s: %zu B, first bytes [%s]", what, len, hex);
    QUEST_LINK_INFO("CAMERA_BITSTREAM %s: %s", what, prefixed);
    QUEST_LINK_INFO("CAMERA_BITSTREAM %s: %s", what, starts);
}

const char *
json_type(const cJSON *item)
{
    if (cJSON_IsObject(item)) return "object";
    if (cJSON_IsArray(item)) return "array";
    if (cJSON_IsString(item)) return "string";
    if (cJSON_IsNumber(item)) return "number";
    if (cJSON_IsBool(item)) return "bool";
    if (cJSON_IsNull(item)) return "null";
    return "invalid";
}


void
log_json_structure(const cJSON *item, const char *path, int depth, int *lines)
{
    if (*lines >= 120) return;
    (*lines)++;
    if (cJSON_IsObject(item) || cJSON_IsArray(item)) {
        QUEST_LINK_INFO("CAMERA_JSON %s: %s[%d]", path, json_type(item), cJSON_GetArraySize(item));
    } else {
        QUEST_LINK_INFO("CAMERA_JSON %s: %s", path, json_type(item));
        return;
    }
    if (depth >= 8) return;
    if (cJSON_IsArray(item)) {
        const cJSON *first = cJSON_GetArrayItem(item, 0);
        if (first == NULL) return;
        char child[256];
        snprintf(child, sizeof(child), "%s[0]", path);
        log_json_structure(first, child, depth + 1, lines);
        return;
    }
    for (const cJSON *it = item->child; it != NULL; it = it->next) {
        char child[256];
        snprintf(child, sizeof(child), "%s.%s", path, it->string ? it->string : "?");
        log_json_structure(it, child, depth + 1, lines);
    }
}

// Color camera intrinsics for the passthrough portal, by camera image id.
std::mutex g_calibration_mutex;
ql_passthrough_camera g_calibration[2] = {};
bool g_calibration_valid[2] = {};

void
copy_string(char *out, size_t out_len, const cJSON *item)
{
    out[0] = '\0';
    if (cJSON_IsString(item) && item->valuestring != NULL) snprintf(out, out_len, "%s", item->valuestring);
}

int
copy_numbers(double *out, int capacity, const cJSON *array)
{
    int count = 0;
    for (const cJSON *it = cJSON_IsArray(array) ? array->child : NULL; it != NULL && count < capacity; it = it->next) {
        if (!cJSON_IsNumber(it)) return 0;
        out[count++] = it->valuedouble;
    }
    return count;
}

void
store_color_calibration(const cJSON *root, bool log)
{
    const cJSON *cameras = cJSON_GetObjectItemCaseSensitive(root, "CameraCalibration");
    if (!cJSON_IsArray(cameras)) return;
    struct candidate
    {
        std::string id;
        ql_passthrough_camera camera;
    };
    std::vector<candidate> color;
    for (const cJSON *it = cameras->child; it != NULL; it = it->next) {
        candidate c{};
        c.camera.struct_size = sizeof(ql_passthrough_camera);
        const cJSON *id = cJSON_GetObjectItemCaseSensitive(it, "Id");
        const cJSON *type = cJSON_GetObjectItemCaseSensitive(it, "SensorType");
        const cJSON *projection = cJSON_GetObjectItemCaseSensitive(it, "Projection");
        const cJSON *distortion = cJSON_GetObjectItemCaseSensitive(it, "Distortion");
        double size[2] = {};
        copy_numbers(size, 2, cJSON_GetObjectItemCaseSensitive(it, "ImageSize"));
        c.camera.width = (int32_t)size[0];
        c.camera.height = (int32_t)size[1];
        copy_string(c.camera.projection_model, sizeof(c.camera.projection_model),
                    cJSON_GetObjectItemCaseSensitive(projection, "Model"));
        copy_string(c.camera.distortion_model, sizeof(c.camera.distortion_model),
                    cJSON_GetObjectItemCaseSensitive(distortion, "Model"));
        c.camera.projection_count =
            copy_numbers(c.camera.projection, 4, cJSON_GetObjectItemCaseSensitive(projection, "Coefficients"));
        c.camera.distortion_count =
            copy_numbers(c.camera.distortion, 12, cJSON_GetObjectItemCaseSensitive(distortion, "Coefficients"));
        copy_numbers(c.camera.device_from_camera, 16, cJSON_GetObjectItemCaseSensitive(it, "DeviceFromCamera"));
        c.id = cJSON_IsString(id) && id->valuestring ? id->valuestring : "";
        const char *sensor = cJSON_IsString(type) && type->valuestring ? type->valuestring : "";
        const bool four_three = c.camera.height > 0 && std::abs(c.camera.width * 3 - c.camera.height * 4) <
                                                           c.camera.width / 50;
        const bool is_color = four_three && c.camera.width >= 1000;
        if (is_color && log) {

            const double *m = c.camera.device_from_camera;
            const bool row_major = m[12] == 0 && m[13] == 0 && m[14] == 0;
            auto at = [&](int r, int col) { return row_major ? m[r * 4 + col] : m[col * 4 + r]; };
            QUEST_LINK_INFO("PASSTHROUGH extrinsics Id=%s (%s-major): position mm (%.1f, %.1f, %.1f); camera x axis "
                            "(%.3f, %.3f, %.3f), y (down) (%.3f, %.3f, %.3f), z (forward) (%.3f, %.3f, %.3f)",
                            cJSON_IsString(id) && id->valuestring ? id->valuestring : "?", row_major ? "row" : "column",
                            at(0, 3) * 1000, at(1, 3) * 1000, at(2, 3) * 1000, at(0, 0), at(1, 0), at(2, 0), at(0, 1),
                            at(1, 1), at(2, 1), at(0, 2), at(1, 2), at(2, 2));
        }
        if (log) QUEST_LINK_INFO("PASSTHROUGH calibration camera Id=%s SensorType=%s %dx%d projection=%s(%d) "
                        "distortion=%s(%d)%s",
                        c.id.c_str(), sensor, c.camera.width, c.camera.height, c.camera.projection_model,
                        c.camera.projection_count, c.camera.distortion_model, c.camera.distortion_count,
                        is_color ? " <- color" : "");
        if (is_color) color.push_back(c);
    }
    if (color.size() != 2) {
        QUEST_LINK_ERROR("PASSTHROUGH expected 2 color cameras in the calibration, found %zu; no lens correction",
                         color.size());
        return;
    }
    std::lock_guard<std::mutex> lock(g_calibration_mutex);
    for (int eye = 0; eye < 2; eye++) {
        g_calibration[eye] = color[eye].camera;
        g_calibration_valid[eye] = true;
    }
    if (log)
        QUEST_LINK_INFO("PASSTHROUGH lens calibration: image 0 = %s, image 1 = %s", color[0].id.c_str(),
                        color[1].id.c_str());
}

void
log_metadata_json(const capnp::List<uint8_t>::Reader &list)
{
    const kj::ArrayPtr<const kj::byte> bytes = capnp::AnyList::Reader(list).getRawBytes();
    cJSON *root = cJSON_ParseWithLength((const char *)bytes.begin(), bytes.size());
    if (root == NULL) {
        QUEST_LINK_ERROR("CAMERA_JSON metadata blob (%zu B) did not parse as JSON", bytes.size());
        return;
    }
    static bool structure_logged = false;
    const bool first = !structure_logged;
    if (first) {
        structure_logged = true;
        int lines = 0;
        log_json_structure(root, "$", 0, &lines);
        if (lines >= 120) QUEST_LINK_INFO("CAMERA_JSON structure truncated at 120 lines");
    }
    store_color_calibration(root, first);
    cJSON_Delete(root);
}

void
log_frame(const PayloadCameraStream::Reader &root, uint64_t index)
{
    const CameraStruct1::Reader frame = root.getUnk1();
    QUEST_LINK_INFO("CAMERA_META frame #%llu: unk0=%llu struct1Unk0=%llu timestamp=%.6f struct1Unk2=%llu",
                    (unsigned long long)index, (unsigned long long)root.getUnk0(),
                    (unsigned long long)frame.getStruct1Unk0(), frame.getTimestampSecs(),
                    (unsigned long long)frame.getStruct1Unk2());

    const CameraStruct2::Reader set = frame.getStruct1Unk3();
    const auto metas = set.getMetas();
    QUEST_LINK_INFO("CAMERA_META   set: struct2Unk0p0=%u struct2Unk0p1=%u metas=%u", set.getStruct2Unk0p0(),
                    set.getStruct2Unk0p1(), metas.size());
    for (uint32_t i = 0; i < metas.size() && i < 8; i++) {
        const CameraMetadata::Reader m = metas[i];
        QUEST_LINK_INFO("CAMERA_META   meta[%u]: id=%d %ux%u stride=%u bufferSize=%u unk1=(%u,%u,%u,%u) unk2=%u", i,
                        m.getId(), m.getWidth(), m.getHeight(), m.getStride(), m.getBufferSize(),
                        m.getMetaUnk1p0(), m.getMetaUnk1p1(), m.getMetaUnk1p2(), m.getMetaUnk1p3(),
                        m.getMetaUnk2p0());
    }

    const auto images = frame.getStruct1Unk4();
    for (uint32_t i = 0; i < images.size() && i < 8; i++) {
        const CameraData::Reader image = images[i];
        char bitstream[96];
        describe_bitstream(image.getData(), bitstream, sizeof(bitstream));
        QUEST_LINK_INFO("CAMERA_META   data[%u]: dataUnk0=%llu bytes=%u (%s)", i,
                        (unsigned long long)image.getDataUnk0(), image.getData().size(), bitstream);
    }
}

void
probe_float_image(uint32_t index, const CameraMetadata::Reader &meta, kj::ArrayPtr<const kj::byte> bytes)
{
    const uint32_t width = meta.getWidth(), height = meta.getHeight(), stride = meta.getStride();
    if (width == 0 || height == 0 || stride < width * 4 || bytes.size() < (size_t)stride * (height - 1) + width * 4) {
        QUEST_LINK_INFO("FLOAT_PROBE image %u: unexpected layout %ux%u stride %u, %zu bytes", index, width, height,
                        stride, bytes.size());
        return;
    }
    std::vector<float> all;
    std::vector<float> cells[9];
    all.reserve((size_t)width * height);
    size_t nonfinite = 0, zeros = 0;
    for (uint32_t y = 0; y < height; y++) {
        for (uint32_t x = 0; x < width; x++) {
            float v;
            memcpy(&v, bytes.begin() + (size_t)y * stride + (size_t)x * 4, 4);
            if (!std::isfinite(v)) { nonfinite++; continue; }
            if (v == 0) zeros++;
            all.push_back(v);
            cells[(y * 3 / height) * 3 + x * 3 / width].push_back(v);
        }
    }
    auto pick = [](std::vector<float> &v, double q) -> double {
        if (v.empty()) return NAN;
        const size_t k = std::min(v.size() - 1, (size_t)(q * (v.size() - 1)));
        std::nth_element(v.begin(), v.begin() + k, v.end());
        return (double)v[k];
    };
    const double mn = all.empty() ? NAN : *std::min_element(all.begin(), all.end());
    const double mx = all.empty() ? NAN : *std::max_element(all.begin(), all.end());
    const double p5 = pick(all, 0.05), p50 = pick(all, 0.5), p95 = pick(all, 0.95);
    double grid[9];
    for (int i = 0; i < 9; i++) grid[i] = pick(cells[i], 0.5);
    QUEST_LINK_INFO("FLOAT_PROBE image %u (id %d): min %.4g p5 %.4g p50 %.4g p95 %.4g max %.4g | non-finite %zu zero %zu "
                    "| grid [%.3g %.3g %.3g / %.3g %.3g %.3g / %.3g %.3g %.3g]",
                    index, meta.getId(), mn, p5, p50, p95, mx, nonfinite, zeros, grid[0], grid[1], grid[2], grid[3],
                    grid[4], grid[5], grid[6], grid[7], grid[8]);
}

void
probe_float_frame(camera_stats &stats, const CameraStruct1::Reader &frame, const capnp::List<CameraMetadata>::Reader &metas,
                  const kj::ArrayPtr<const kj::byte> *data, uint32_t image_count)
{
    if (!debug_get_bool_option_float_probe() || stats.float_probe_done) return;
    const int64_t now = (int64_t)os_monotonic_get_ns();
    if (stats.float_probe_start_ns == 0) {
        stats.float_probe_start_ns = now;
        QUEST_LINK_INFO("FLOAT_PROBE started: statistics of the float images twice a second for %lld s "
                        "(no values or images are logged)",
                        (long long)debug_get_num_option_float_probe_seconds());
    }
    if (now - stats.float_probe_start_ns > debug_get_num_option_float_probe_seconds() * 1000000000LL) {
        stats.float_probe_done = true;
        QUEST_LINK_INFO("FLOAT_PROBE finished");
        return;
    }
    if (now - stats.float_probe_last_ns < 500000000LL) return;
    stats.float_probe_last_ns = now;
    QUEST_LINK_INFO("FLOAT_PROBE t=%.1f s: float ts %.6f, newest color ts %.6f (float - color %.1f ms), images %u",
                    (now - stats.float_probe_start_ns) / 1e9, frame.getTimestampSecs(), stats.last_color_timestamp,
                    (frame.getTimestampSecs() - stats.last_color_timestamp) * 1000, image_count);
    for (uint32_t i = 0; i < image_count && i < metas.size(); i++) probe_float_image(i, metas[i], data[i]);
}

void
handle_message(camera_stats &stats, const ql_camera_reassembler &r)
{
    stats.messages++;

    kj::ArrayPtr<const capnp::word> segments[ql_camera_reassembler::kMaxSegments];
    size_t at = 0;
    for (uint32_t i = 0; i < r.segment_count; i++) {
        segments[i] = kj::arrayPtr((const capnp::word *)(r.body.data() + at), r.segment_words[i]);
        at += (size_t)r.segment_words[i] * 8;
    }
    capnp::ReaderOptions options;

    options.traversalLimitInWords = (r.body.size() / 8) * 16 + 1024;

    try {
        capnp::SegmentArrayMessageReader message(kj::arrayPtr(segments, r.segment_count), options);
        const capnp::AnyStruct::Reader any = message.getRoot<capnp::AnyStruct>();
        const auto data = any.getDataSection();
        uint16_t discriminator = 0;
        if (data.size() >= 2) memcpy(&discriminator, data.begin(), 2);

        const bool detailed = stats.messages <= kDetailedMessages;
        if (detailed) {
            char sizes[128] = {0};
            size_t used = 0;
            for (uint32_t i = 0; i < r.segment_count && used < sizeof(sizes); i++) {
                int n = snprintf(sizes + used, sizeof(sizes) - used, "%s%u", i ? "," : "", r.segment_words[i]);
                if (n < 0) break;
                used += (size_t)n;
            }
            QUEST_LINK_INFO("CAMERA_META message #%llu: segments=%u words=[%s] root data=%zu bytes pointers=%u "
                            "discriminator=%u",
                            (unsigned long long)stats.messages, r.segment_count, sizes, data.size(),
                            any.getPointerSection().size(), discriminator);
        }

        if (discriminator == 0) {
            const PayloadCameraStream::Reader root = message.getRoot<PayloadCameraStream>();
            const CameraStruct1::Reader frame = root.getUnk1();
            const auto images = frame.getStruct1Unk4();
            const auto metas = frame.getStruct1Unk3().getMetas();
            const bool color = metas.size() > 0 && metas[0].getMetaUnk2p0() == 3;

            kj::ArrayPtr<const kj::byte> data[2];
            const uint32_t image_count = images.size() < 2 ? images.size() : 2;
            uint64_t bytes = 0;
            for (uint32_t i = 0; i < image_count; i++) {
                data[i] = capnp::AnyList::Reader(images[i].getData()).getRawBytes();
                bytes += data[i].size();
            }
#ifdef __APPLE__

            const bool passthrough = stats.passthrough;

            if (color && stats.host != nullptr && (stats.color_frames % 60) == 1 && stats.timing_logs < 5) {
                stats.timing_logs++;
                const double camera_s = frame.getTimestampSecs();
                const double pose_s = stats.host->last_pose_target_ts / 1e9;
                QUEST_LINK_INFO("PASSTHROUGH timing: camera ts %.6f s, latest pose ts %.6f s, camera - pose %.1f ms "
                                "(ns_offset %lld)",
                                camera_s, pose_s, (camera_s - pose_s) * 1000,
                                (long long)stats.host->ns_offset);
            }
            if (color && (debug_get_bool_option_camera_decode() || passthrough)) {
                if (stats.vt == nullptr) {
                    stats.vt = ql_camera_vt_create(passthrough);
                    QUEST_LINK_INFO("CAMERA_DECODE enabled: VideoToolbox HEVC decode, %s",
                                    passthrough ? "frames kept for the passthrough portal (never saved)"
                                                : "statistics only");
                }
                stats.color_frames++;
                if ((int64_t)stats.color_frames == debug_get_num_option_camera_drop_test()) {
                    QUEST_LINK_INFO("CAMERA_RECOVERY drop test: discarding color frame %llu as if lost",
                                    (unsigned long long)stats.color_frames);
                    ql_camera_vt_mark_gap(stats.vt, 0);
                    ql_camera_vt_mark_gap(stats.vt, 1);
                } else {
                    for (uint32_t i = 0; i < image_count && i < metas.size(); i++) {
                        ql_camera_vt_submit(stats.vt, metas[i].getId(), data[i].begin(), data[i].size(),
                                            (uint64_t)std::llround(frame.getTimestampSecs() * 1e9));
                    }
                }
            }
#endif
            if (color) stats.last_color_timestamp = frame.getTimestampSecs();
            else {
                probe_float_frame(stats, frame, metas, data, image_count);
                if (stats.passthrough) store_float_frame(metas, data, image_count, frame.getTimestampSecs());
            }
            {
                const int kind = color ? 0 : 1;
                const uint64_t counter = frame.getStruct1Unk2();
                if (stats.counter_seen[kind]) {
                    const uint64_t step = counter - stats.counter_last[kind];
                    stats.counter_steps[kind][step == 1 ? 0 : step == 2 ? 1 : 2]++;
                }
                stats.counter_seen[kind] = true;
                stats.counter_last[kind] = counter;
            }
            stats.frames++;
            stats.period_frames++;
            stats.image_bytes += bytes;
            stats.period_image_bytes += bytes;
            if (stats.frames == 1) stats.first_timestamp = frame.getTimestampSecs();
            stats.last_timestamp = frame.getTimestampSecs();
            if (color && stats.color_frames_described < 3) {
                stats.color_frames_described++;
                for (uint32_t i = 0; i < image_count; i++) {
                    char what[64];
                    snprintf(what, sizeof(what), "color frame %llu image %u",
                             (unsigned long long)stats.color_frames_described, i);
                    log_bitstream_structure(what, data[i].begin(), data[i].size());
                }
            }
            if (detailed) log_frame(root, stats.frames);
        } else if (discriminator == 1) {
            stats.meta_messages++;
            const PayloadCameraStreamMeta::Reader root = message.getRoot<PayloadCameraStreamMeta>();
            const CameraMetaData::Reader meta = root.getMetadata();
            if (detailed) {
                QUEST_LINK_INFO("CAMERA_META metadata message: dataUnk0=%llu bytes=%u",
                                (unsigned long long)meta.getDataUnk0(), meta.getData().size());
                const kj::ArrayPtr<const kj::byte> blob = capnp::AnyList::Reader(meta.getData()).getRawBytes();
                log_bitstream_structure("metadata blob", blob.begin(), blob.size());
            }
            log_metadata_json(meta.getData());
        } else {
            stats.other_messages++;
        }
    } catch (...) {
        stats.parse_failures++;
#ifdef __APPLE__
        ql_camera_vt_mark_gap(stats.vt, 0);
        ql_camera_vt_mark_gap(stats.vt, 1);
#endif
        if (stats.parse_failures <= 3) {
            QUEST_LINK_ERROR("CAMERA_META message #%llu failed to parse (%u segments, %zu bytes)",
                             (unsigned long long)stats.messages, r.segment_count, r.body.size());
        }
    }
}

void
maybe_log_summary(camera_stats &stats, bool force)
{
    const int64_t now = os_monotonic_get_ns();
    if (stats.period_start_ns == 0) {
        stats.period_start_ns = now;
        return;
    }
    const int64_t elapsed = now - stats.period_start_ns;
    if (!force && elapsed < kSummaryPeriodNs) return;
    if (!force && !debug_get_bool_option_camera_probe_logs()) {
        stats.period_start_ns = now; // keep counting, but stay quiet outside diagnostics
        stats.period_frames = 0;
        stats.period_image_bytes = 0;
        return;
    }
    const double secs = elapsed > 0 ? (double)elapsed / 1e9 : 1.0;
    QUEST_LINK_INFO("CAMERA_META summary: %.1f frames/s, %.2f MB/s image data; totals messages=%llu frames=%llu "
                    "meta=%llu other=%llu parse_failures=%llu framing_errors=%llu headset_time_span=%.3fs",
                    stats.period_frames / secs, stats.period_image_bytes / secs / 1e6,
                    (unsigned long long)stats.messages, (unsigned long long)stats.frames,
                    (unsigned long long)stats.meta_messages, (unsigned long long)stats.other_messages,
                    (unsigned long long)stats.parse_failures, (unsigned long long)stats.reassembler.invalid,
                    stats.last_timestamp - stats.first_timestamp);
    QUEST_LINK_INFO("CAMERA_META frame counter steps: color 1:%llu 2:%llu other:%llu; float 1:%llu 2:%llu other:%llu",
                    (unsigned long long)stats.counter_steps[0][0], (unsigned long long)stats.counter_steps[0][1],
                    (unsigned long long)stats.counter_steps[0][2], (unsigned long long)stats.counter_steps[1][0],
                    (unsigned long long)stats.counter_steps[1][1], (unsigned long long)stats.counter_steps[1][2]);
    stats.period_start_ns = now;
    stats.period_frames = 0;
    stats.period_image_bytes = 0;
}

} // namespace

extern "C" void
ql_xrsp_camera_consume(struct ql_xrsp_host *host, struct ql_xrsp_topic_pkt *pkt)
{
    if (host->camera_stats == NULL) {
        host->camera_stats = new camera_stats();
        ((camera_stats *)host->camera_stats)->passthrough = host->passthrough;
        ((camera_stats *)host->camera_stats)->host = host;
    }
    camera_stats &stats = *(camera_stats *)host->camera_stats;

    size_t offset = 0;
    const size_t len = pkt->payload_valid > 0 ? (size_t)pkt->payload_valid : 0;
    for (;;) {
        const auto result = stats.reassembler.feed(pkt->payload, len, &offset);
        if (result == ql_camera_reassembler::Result::Message) {
            handle_message(stats, stats.reassembler);
            stats.reassembler.reset();
            continue;
        }
        if (result == ql_camera_reassembler::Result::Invalid) {
#ifdef __APPLE__
            ql_camera_vt_mark_gap(stats.vt, 0);
            ql_camera_vt_mark_gap(stats.vt, 1);
#endif
            if (stats.reassembler.invalid <= 3) {
                QUEST_LINK_ERROR("CAMERA_META framing error in a %zu-byte packet; resyncing at the next packet", len);
            }
        }
        break;
    }
    maybe_log_summary(stats, false);
}

extern "C" bool
ql_xrsp_camera_decoder_stalled(struct ql_xrsp_host *host)
{
#ifdef __APPLE__
    if (host->camera_stats == NULL) return false;
    return ql_camera_vt_stalled(((camera_stats *)host->camera_stats)->vt);
#else
    (void)host;
    return false;
#endif
}

extern "C" void
ql_xrsp_camera_log_summary(struct ql_xrsp_host *host)
{
    if (host->camera_stats == NULL) return;
    camera_stats &stats = *(camera_stats *)host->camera_stats;
    maybe_log_summary(stats, true);
#ifdef __APPLE__
    ql_camera_vt_log_summary(stats.vt);
#endif
}

extern "C" void
ql_xrsp_camera_destroy(struct ql_xrsp_host *host)
{
    if (host->camera_stats == NULL) return;
    camera_stats *stats = (camera_stats *)host->camera_stats;
    if (stats->messages > 0) maybe_log_summary(*stats, true);
    delete stats;
    host->camera_stats = NULL;
}

void
ql_xrsp_camera_clear_frames(void)
{
#ifdef __APPLE__
    ql_camera_vt_clear_frames();
#endif
    ql_xrsp_camera_clear_float();
}

namespace {
std::mutex g_float_mutex;
std::vector<float> g_float_images[2];
int g_float_width = 0, g_float_height = 0;
int64_t g_float_capture_ns = 0;
uint64_t g_float_seq = 0;
} // namespace

static void
store_float_frame(const capnp::List<CameraMetadata>::Reader &metas, const kj::ArrayPtr<const kj::byte> *data,
                  uint32_t image_count, double timestamp_s)
{
    if (image_count < 2 || metas.size() < 2) return;
    const uint32_t width = metas[0].getWidth(), height = metas[0].getHeight(), stride = metas[0].getStride();
    if (width == 0 || height == 0 || width > 512 || height > 512 || stride < width * 4) return;
    std::lock_guard<std::mutex> lock(g_float_mutex);
    for (uint32_t i = 0; i < 2; i++) {
        if (metas[i].getWidth() != width || metas[i].getHeight() != height || metas[i].getStride() != stride ||
            data[i].size() < (size_t)stride * (height - 1) + width * 4) return;
    }
    for (uint32_t i = 0; i < 2; i++) {
        const int id = metas[i].getId();
        if (id < 0 || id > 1) return;
        auto &image = g_float_images[id];
        image.resize((size_t)width * height);
        for (uint32_t y = 0; y < height; y++) memcpy(&image[(size_t)y * width], data[i].begin() + (size_t)y * stride, width * 4);
    }
    g_float_width = (int)width;
    g_float_height = (int)height;
    g_float_capture_ns = (int64_t)std::llround(timestamp_s * 1e9);
    g_float_seq++;
}

extern "C" void
ql_xrsp_camera_clear_float(void)
{
    std::lock_guard<std::mutex> lock(g_float_mutex);
    for (auto &image : g_float_images) std::vector<float>().swap(image);
    g_float_capture_ns = 0;
}

extern "C" __attribute__((visibility("default"))) bool
ql_passthrough_copy_float(int eye, uint64_t *io_seq, float *out, int capacity, int *out_width, int *out_height,
                          int64_t *out_capture_ns)
{
    if (eye < 0 || eye > 1 || io_seq == NULL || out == NULL || out_width == NULL || out_height == NULL ||
        out_capture_ns == NULL) return false;
    std::lock_guard<std::mutex> lock(g_float_mutex);
    const auto &image = g_float_images[eye];
    if (image.empty() || *io_seq == g_float_seq || capacity < (int)image.size()) return false;
    std::copy(image.begin(), image.end(), out);
    *out_width = g_float_width;
    *out_height = g_float_height;
    *out_capture_ns = g_float_capture_ns;
    *io_seq = g_float_seq;
    return true;
}

extern "C" __attribute__((visibility("default"))) bool
ql_passthrough_copy_camera(int eye, struct ql_passthrough_camera *out)
{
    if (eye < 0 || eye > 1 || out == NULL || out->struct_size < sizeof(ql_passthrough_camera)) return false;
    std::lock_guard<std::mutex> lock(g_calibration_mutex);
    if (!g_calibration_valid[eye]) return false;
    *out = g_calibration[eye];
    return true;
}
