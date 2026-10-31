#include "../src/xrt/drivers/quest_link/ql_mirror_pixels.h"
#include <cassert>
#include <cstdio>
using xrt::drivers::quest_link::MirrorMap;
int main() {
    struct Axis { double center, scale, a, b; };
    const Axis axis{0.05, 0.75, 1.4, -0.08};
    for (float source : {0.15f, 0.3f, 0.5f, 0.7f, 0.85f}) {
        const double undistorted = (1 + axis.a * std::tan(axis.scale / axis.a * (2 * source - 1 - axis.center)) + axis.b) / 2;
        const float restored = xrt::drivers::quest_link::mirror_source_coordinate(axis, float(undistorted));
        assert(std::abs(restored - source) < 0.00001f);
    }
    MirrorMap map;
    auto identity = [](uint32_t, float u, float v) { return std::array<float, 2>{u, v}; };
    assert(!map.reset(5, 4, 2, 4, identity));
    assert(map.reset(8, 4, 4, 4, identity));
    std::vector<uint8_t> source(8 * 4 * 4);
    for (unsigned y = 0; y < 4; ++y) for (unsigned x = 0; x < 8; ++x) {
        auto offset = (y * 8 + x) * 4;
        source[offset] = x < 4 ? 20 + x : 120 + x - 4;
        source[offset + 1] = 30 + y;
        source[offset + 3] = 255;
    }
    std::vector<uint8_t> output;
    assert(!map.convert(source.data(), source.size() - 1, output));
    assert(map.convert(source.data(), source.size(), output));
    for (unsigned eye = 0; eye < 2; ++eye) for (unsigned y = 0; y < 4; ++y) for (unsigned x = 0; x < 4; ++x) {
        auto offset = ((eye * 4 + y) * 4 + x) * 4;
        assert(output[offset] == (eye ? 120 : 20) + x);
        assert(output[offset + 1] == 30 + y && output[offset + 3] == 255);
    }
    auto reversed = [](uint32_t eye, float u, float v) { return std::array<float, 2>{eye ? 1-u : u, v}; };
    assert(map.reset(8, 4, 4, 4, reversed) && map.convert(source.data(), source.size(), output));
    assert(output[4 * 4 * 4] == 123 && output[4 * 4 * 4 + 12] == 120);
    puts("PASS: stereo split, coordinates, sampling, and bounds");
}
