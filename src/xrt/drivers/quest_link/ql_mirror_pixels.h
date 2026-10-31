#pragma once
// SPDX-License-Identifier: BSL-1.0
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace xrt::drivers::quest_link {

template <typename FoveationAxis>
inline float mirror_source_coordinate(const FoveationAxis& axis, float undistorted) {
    if (axis.scale >= 1.0) return undistorted;
    const double input = 2.0 * undistorted - 1.0;
    const double coordinate = axis.center + (axis.a / axis.scale) * std::atan((input - axis.b) / axis.a);
    return float(std::clamp((1.0 + coordinate) * 0.5, 0.0, 1.0));
}

struct MirrorMap {
    struct Sample { uint32_t low, high; float fraction; };
    uint32_t sourceEyeWidth = 0, sourceHeight = 0, width = 0, height = 0;
    std::array<std::vector<Sample>, 2> x, y;

    template <typename Mapping>
    bool reset(uint32_t inputWidth, uint32_t inputHeight, uint32_t outputWidth,
               uint32_t outputHeight, Mapping map) {
        if (!inputWidth || inputWidth % 2 || !inputHeight || !outputWidth || !outputHeight ||
            inputWidth > 8192 || inputHeight > 4096 || outputWidth > 4096 || outputHeight > 4096) return false;
        sourceEyeWidth = inputWidth / 2; sourceHeight = inputHeight;
        width = outputWidth; height = outputHeight;
        auto sample = [](float coordinate, uint32_t size) {
            if (!std::isfinite(coordinate)) coordinate = 0;
            float pixel = std::clamp(coordinate * size - 0.5f, 0.0f, float(size - 1));
            auto low = uint32_t(pixel);
            return Sample{low, std::min(low + 1, size - 1), pixel - low};
        };
        for (uint32_t eye = 0; eye < 2; ++eye) {
            x[eye].resize(width); y[eye].resize(height);
            for (uint32_t i = 0; i < width; ++i) {
                const auto uv = map(eye, (i + 0.5f) / width, 0.5f);
                x[eye][i] = sample(uv[0], sourceEyeWidth);
            }
            for (uint32_t i = 0; i < height; ++i) {
                const auto uv = map(eye, 0.5f, (i + 0.5f) / height);
                y[eye][i] = sample(uv[1], sourceHeight);
            }
        }
        return true;
    }

    bool convert(const uint8_t *source, size_t sourceBytes, std::vector<uint8_t>& output) const {
        if (!source || !width || !height || !sourceEyeWidth || !sourceHeight ||
            sourceBytes < size_t(sourceEyeWidth) * 2 * sourceHeight * 4) return false;
        output.resize(size_t(width) * height * 8);
        const size_t stride = size_t(sourceEyeWidth) * 2 * 4;
        for (uint32_t eye = 0; eye < 2; ++eye) {
            if (x[eye].size() != width || y[eye].size() != height) return false;
            uint8_t *dest = output.data() + size_t(eye) * width * height * 4;
            const uint8_t *eyeSource = source + size_t(eye) * sourceEyeWidth * 4;
            for (uint32_t row = 0; row < height; ++row) {
                const auto sy = y[eye][row];
                for (uint32_t column = 0; column < width; ++column) {
                    const auto sx = x[eye][column];
                    const uint8_t *a = eyeSource + size_t(sy.low) * stride + size_t(sx.low) * 4;
                    const uint8_t *b = eyeSource + size_t(sy.low) * stride + size_t(sx.high) * 4;
                    const uint8_t *c = eyeSource + size_t(sy.high) * stride + size_t(sx.low) * 4;
                    const uint8_t *d = eyeSource + size_t(sy.high) * stride + size_t(sx.high) * 4;
                    for (unsigned channel = 0; channel < 4; ++channel) {
                        float top = a[channel] + (float(b[channel]) - a[channel]) * sx.fraction;
                        float bottom = c[channel] + (float(d[channel]) - c[channel]) * sx.fraction;
                        *dest++ = uint8_t(std::clamp(std::lround(top + (bottom - top) * sy.fraction), 0L, 255L));
                    }
                }
            }
        }
        return true;
    }
};
}
