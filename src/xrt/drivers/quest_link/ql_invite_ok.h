// SPDX-License-Identifier: BSL-1.0
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static inline size_t
ql_invite_ok_add_camera_formats(const uint8_t *in, size_t in_len, const uint16_t *formats, size_t count,
                                uint8_t *out, size_t out_cap)
{
    if (in_len < 8 || in_len % 8 != 0 || count == 0 || count > 0x1fffffff) return 0;

    uint32_t root_lo, root_hi;
    memcpy(&root_lo, in, 4);
    memcpy(&root_hi, in + 4, 4);
    const size_t data_words = root_hi & 0xffff;
    const size_t ptr_words = root_hi >> 16;
    if (root_lo != 0 || ptr_words != 3) return 0;

    const size_t in_words = in_len / 8;
    const size_t ptr_end = 1 + data_words + ptr_words;
    if (ptr_end > in_words) return 0;

    const size_t list_words = (count * 2 + 7) / 8;
    const size_t out_len = (in_words + 1 + list_words) * 8;
    if (out_len > out_cap) return 0;
    memset(out, 0, out_len);

    memcpy(out, in, ptr_end * 8);
    root_hi += 1u << 16;
    memcpy(out + 4, &root_hi, 4);

    for (size_t slot = 1 + data_words; slot < ptr_end; slot++) {
        uint32_t lo;
        memcpy(&lo, in + slot * 8, 4);
        uint64_t whole;
        memcpy(&whole, in + slot * 8, 8);
        if (whole == 0) continue;
        const uint32_t kind = lo & 3;
        if (kind > 1) return 0;
        const int32_t offset = (int32_t)lo >> 2;
        if ((int64_t)slot + 1 + offset < (int64_t)ptr_end) return 0;
        lo = ((uint32_t)(offset + 1) << 2) | kind;
        memcpy(out + slot * 8, &lo, 4);
    }

    memcpy(out + (ptr_end + 1) * 8, in + ptr_end * 8, (in_words - ptr_end) * 8);

    const size_t new_slot = ptr_end;
    const size_t list_at = in_words + 1;
    const uint32_t list_lo = ((uint32_t)(list_at - new_slot - 1) << 2) | 1;
    const uint32_t list_hi = ((uint32_t)count << 3) | 3;
    memcpy(out + new_slot * 8, &list_lo, 4);
    memcpy(out + new_slot * 8 + 4, &list_hi, 4);
    for (size_t i = 0; i < count; i++) {
        memcpy(out + list_at * 8 + i * 2, &formats[i], 2);
    }
    return out_len;
}
