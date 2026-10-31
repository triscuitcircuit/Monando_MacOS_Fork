// SPDX-License-Identifier: BSL-1.0
#pragma once
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstring>

template <size_t Capacity> class QlAudioBuffer {
    static_assert(Capacity > 0);
    std::array<float, Capacity * 2> samples{};
    size_t read = 0, count = 0;
public:
    size_t size() const { return count; }
    void clear() { read = count = 0; }
    void discard(size_t frames) {
        frames = std::min(frames, count);
        read = (read + frames) % Capacity;
        count -= frames;
    }
    bool push(const float* source, size_t frames) {
        if (!frames) return true;
        if (!source) return false;
        if (frames >= Capacity) {
            source += (frames - Capacity) * 2;
            frames = Capacity;
            clear();
        }
        if (frames > Capacity - count) discard(frames - (Capacity - count));
        const size_t at = (read + count) % Capacity;
        const size_t first = std::min(frames, Capacity - at);
        std::memcpy(samples.data() + at * 2, source, first * 2 * sizeof(float));
        std::memcpy(samples.data(), source + first * 2, (frames - first) * 2 * sizeof(float));
        count += frames;
        return true;
    }
    bool pop(float* destination, size_t frames) {
        if (!frames) return true;
        if (!destination || frames > count) return false;
        const size_t first = std::min(frames, Capacity - read);
        std::memcpy(destination, samples.data() + read * 2, first * 2 * sizeof(float));
        std::memcpy(destination + first * 2, samples.data(), (frames - first) * 2 * sizeof(float));
        discard(frames);
        return true;
    }
};

inline void QlMixStereo(float* destination, const float* source, size_t frames) {
    for (size_t i = 0; i < frames * 2; ++i)
        destination[i] = std::clamp(destination[i] + source[i], -1.0f, 1.0f);
}
