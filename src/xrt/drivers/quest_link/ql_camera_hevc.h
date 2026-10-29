// SPDX-License-Identifier: BSL-1.0
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <vector>

struct ql_hevc_access_unit
{
    std::vector<uint8_t> vps, sps, pps;
    std::vector<uint8_t> sample;
    size_t vcl_count = 0;
    bool irap = false;
    bool cra_or_bla = false;
    bool rasl = false;
};

enum ql_hevc_nal_type
{
    QL_HEVC_NAL_VPS = 32,
    QL_HEVC_NAL_SPS = 33,
    QL_HEVC_NAL_PPS = 34,
};

static inline void
ql_hevc_add_nal(ql_hevc_access_unit *out, const uint8_t *nal, size_t len)
{
    while (len > 0 && nal[len - 1] == 0) len--;
    if (len < 2) return;
    const unsigned type = (nal[0] >> 1) & 0x3f;
    if (type == QL_HEVC_NAL_VPS) {
        out->vps.assign(nal, nal + len);
    } else if (type == QL_HEVC_NAL_SPS) {
        out->sps.assign(nal, nal + len);
    } else if (type == QL_HEVC_NAL_PPS) {
        out->pps.assign(nal, nal + len);
    } else if (type < 32) {
        const uint8_t prefix[4] = {(uint8_t)(len >> 24), (uint8_t)(len >> 16), (uint8_t)(len >> 8), (uint8_t)len};
        out->sample.insert(out->sample.end(), prefix, prefix + 4);
        out->sample.insert(out->sample.end(), nal, nal + len);
        out->vcl_count++;
        if (type >= 16 && type <= 21) out->irap = true;
        if ((type >= 16 && type <= 18) || type == 21) out->cra_or_bla = true;
        if (type == 8 || type == 9) out->rasl = true;
    }
}

static inline void
ql_hevc_split_annexb(ql_hevc_access_unit *out, const uint8_t *data, size_t len)
{
    size_t nal_start = 0;
    bool in_nal = false;
    size_t i = 0;
    while (i + 3 <= len) {
        if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1) {
            if (in_nal) ql_hevc_add_nal(out, data + nal_start, i - nal_start);
            i += 3;
            nal_start = i;
            in_nal = true;
        } else {
            i++;
        }
    }
    if (in_nal && nal_start < len) ql_hevc_add_nal(out, data + nal_start, len - nal_start);
}

static inline bool
ql_hevc_from_chunks(const uint8_t *data, size_t len, ql_hevc_access_unit *out)
{
    *out = ql_hevc_access_unit();
    size_t at = 0;
    while (at < len) {
        if (len - at < 4) return false;
        uint32_t n;
        memcpy(&n, data + at, 4);
        at += 4;
        if (n == 0 || n > len - at) return false;
        ql_hevc_split_annexb(out, data + at, n);
        at += n;
    }
    return true;
}
