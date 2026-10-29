// SPDX-License-Identifier: BSL-1.0
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <vector>


struct ql_camera_reassembler
{
    static constexpr uint32_t kMaxSegments = 16;
    static constexpr size_t kMaxMessageBytes = 64u << 20;

    enum class Result
    {
        Incomplete,
        Message,
        Invalid,
    };

    std::vector<uint8_t> header;
    std::vector<uint8_t> body;
    uint32_t segment_count = 0;
    uint32_t segment_words[kMaxSegments] = {};
    size_t body_expected = 0;
    uint64_t invalid = 0;

    void
    reset()
    {
        header.clear();
        body.clear();
        segment_count = 0;
        body_expected = 0;
    }

    size_t
    header_bytes() const
    {
        return ((4 + 4 * (size_t)segment_count) + 7) & ~(size_t)7;
    }


    Result
    feed(const uint8_t *data, size_t len, size_t *offset)
    {
        while (*offset < len) {
            if (body_expected == 0) {
                const size_t want = header.size() < 4 ? 4 : header_bytes();
                const size_t take = want - header.size() < len - *offset ? want - header.size() : len - *offset;
                header.insert(header.end(), data + *offset, data + *offset + take);
                *offset += take;
                if (header.size() < want) return Result::Incomplete;

                if (segment_count == 0) {
                    uint32_t minus_one;
                    memcpy(&minus_one, header.data(), 4);
                    if (minus_one >= kMaxSegments) return drop_packet(len, offset);
                    segment_count = minus_one + 1;
                    continue;
                }

                size_t total = 0;
                for (uint32_t i = 0; i < segment_count; i++) {
                    memcpy(&segment_words[i], header.data() + 4 + 4 * i, 4);
                    total += (size_t)segment_words[i] * 8;
                }
                if (total == 0 || total > kMaxMessageBytes) return drop_packet(len, offset);
                body_expected = total;
                body.reserve(total);
                continue;
            }

            const size_t take = body_expected - body.size() < len - *offset ? body_expected - body.size() : len - *offset;
            body.insert(body.end(), data + *offset, data + *offset + take);
            *offset += take;
            if (body.size() == body_expected) return Result::Message;
        }
        return Result::Incomplete;
    }

private:
    Result
    drop_packet(size_t len, size_t *offset)
    {
        invalid++;
        reset();
        *offset = len;
        return Result::Invalid;
    }
};
