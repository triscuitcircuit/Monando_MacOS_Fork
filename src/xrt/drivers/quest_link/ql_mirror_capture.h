#pragma once
// SPDX-License-Identifier: BSL-1.0
#include "ql_mirror_pixels.h"
#include <vulkan/vulkan_core.h>
#include <chrono>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <vector>
struct vk_bundle;
struct ql_hmd;

namespace xrt::drivers::quest_link {
std::string mirror_socket_path();
class MirrorCapture {
public:
    MirrorCapture(vk_bundle *vk, VkCommandPool pool, VkFormat format, uint32_t width, uint32_t height,
                  uint32_t count, const VkImage *images, const ql_hmd *hmd);
    ~MirrorCapture();
    bool ready();
    VkCommandBuffer command(uint32_t index) const;
    void submit(uint32_t index, uint64_t frame);
    bool available() const { return valid; }
#ifdef QL_MIRROR_TESTING
    int testTargetFPS() const { return targetFPS.load(std::memory_order_relaxed); }
#endif
private:
    struct Slot { VkBuffer buffer = VK_NULL_HANDLE; VkDeviceMemory memory = VK_NULL_HANDLE;
                  VkCommandBuffer command = VK_NULL_HANDLE; void *mapped = nullptr; };
    struct Frame { uint64_t number; std::vector<uint8_t> pixels; };
    void run();
    void cleanup();
    vk_bundle *vk;
    VkCommandPool pool;
    uint32_t format = 0;
    size_t sourceBytes = 0;
    MirrorMap map;
    std::vector<Slot> slots;
    std::mutex mutex;
    std::condition_variable wake;
    std::unique_ptr<Frame> pending;
    bool valid = false, stop = false, busy = false, viewer = false;
    std::atomic<int> targetFPS{10};
    std::chrono::steady_clock::time_point lastProbe{}, lastCapture{};
    std::thread worker;
};
}
