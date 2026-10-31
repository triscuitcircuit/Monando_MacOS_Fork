// SPDX-License-Identifier: BSL-1.0
#include "../src/xrt/drivers/quest_link/ql_mirror_capture.h"
#include "../src/xrt/drivers/quest_link/ql_types.h"
#include "vk/vk_helpers.h"
#include <fstream>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>
using xrt::drivers::quest_link::MirrorCapture;
extern "C" bool vk_get_memory_type(vk_bundle *vk, uint32_t bits, VkMemoryPropertyFlags flags, uint32_t *out) {
    for (uint32_t i = 0; i < vk->device_memory_props.memoryTypeCount; ++i)
        if ((bits & (1u << i)) && (vk->device_memory_props.memoryTypes[i].propertyFlags & flags) == flags) { *out = i; return true; }
    return false;
}
static bool readAll(int fd, void *data, size_t size) {
    auto p = static_cast<uint8_t *>(data);
    while (size) { ssize_t n = read(fd, p, size); if (n <= 0) return false; p += n; size -= size_t(n); }
    return true;
}
struct Header { uint32_t magic, version, width, height, format, eyeBytes; uint64_t frame; };
int main() {
    alarm(20);
    std::string name = "/tmp/ql-mirror-gpu-" + std::to_string(getpid()) + ".sock";
    std::string fpsFile = "/tmp/ql-mirror-gpu-" + std::to_string(getpid()) + ".fps";
    unsetenv("QL_MIRROR_SOCKET");
    assert(xrt::drivers::quest_link::mirror_socket_path().empty());
    setenv("QL_MIRROR_SOCKET", "relative.sock", 1);
    assert(xrt::drivers::quest_link::mirror_socket_path().empty());
    setenv("QL_MIRROR_SOCKET", name.c_str(), 1);
    setenv("QL_MIRROR_FPS_FILE", fpsFile.c_str(), 1);
    { std::ofstream output(fpsFile); output << "30"; assert(output.good()); }
    int listener = socket(AF_UNIX, SOCK_STREAM, 0); assert(listener >= 0);
    sockaddr_un address{}; address.sun_family = AF_UNIX; std::strcpy(address.sun_path, name.c_str());
    assert(bind(listener, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0 && listen(listener, 1) == 0);
    Header received{}; std::vector<uint8_t> pixels;
    std::thread reader([&] {
        int fd = accept(listener, nullptr, nullptr); assert(fd >= 0);
        assert(readAll(fd, &received, sizeof(received)));
        pixels.resize(size_t(received.eyeBytes) * 2);
        assert(readAll(fd, pixels.data(), pixels.size())); close(fd);
    });
    VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO}; app.apiVersion = VK_API_VERSION_1_1;
    VkInstanceCreateInfo instanceInfo{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO}; instanceInfo.pApplicationInfo = &app;
    VkInstance instance; assert(vkCreateInstance(&instanceInfo, nullptr, &instance) == VK_SUCCESS);
    uint32_t count = 1; VkPhysicalDevice physical; assert(vkEnumeratePhysicalDevices(instance, &count, &physical) == VK_SUCCESS && count);
    float priority = 1;
    VkDeviceQueueCreateInfo queueInfo{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO}; queueInfo.queueCount = 1; queueInfo.pQueuePriorities = &priority;
    VkDeviceCreateInfo deviceInfo{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO}; deviceInfo.queueCreateInfoCount = 1; deviceInfo.pQueueCreateInfos = &queueInfo;
    VkDevice device; assert(vkCreateDevice(physical, &deviceInfo, nullptr, &device) == VK_SUCCESS);
    VkQueue queue; vkGetDeviceQueue(device, 0, 0, &queue);
    vk_bundle bundle{}; bundle.device = device; vkGetPhysicalDeviceMemoryProperties(physical, &bundle.device_memory_props);
#define LOAD(name) bundle.name = reinterpret_cast<PFN_##name>(vkGetDeviceProcAddr(device, #name)); assert(bundle.name)
    LOAD(vkCreateBuffer); LOAD(vkGetBufferMemoryRequirements); LOAD(vkAllocateMemory); LOAD(vkBindBufferMemory);
    LOAD(vkMapMemory); LOAD(vkUnmapMemory); LOAD(vkFreeMemory); LOAD(vkDestroyBuffer); LOAD(vkAllocateCommandBuffers);
    LOAD(vkFreeCommandBuffers); LOAD(vkBeginCommandBuffer); LOAD(vkEndCommandBuffer); LOAD(vkCmdPipelineBarrier); LOAD(vkCmdCopyImageToBuffer);
#undef LOAD
    VkCommandPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    VkCommandPool pool; assert(vkCreateCommandPool(device, &poolInfo, nullptr, &pool) == VK_SUCCESS);
    VkImageCreateInfo imageInfo{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO}; imageInfo.imageType = VK_IMAGE_TYPE_2D;
    imageInfo.format = VK_FORMAT_R8G8B8A8_UNORM; imageInfo.extent = {16, 8, 1}; imageInfo.mipLevels = 1;
    imageInfo.arrayLayers = 1; imageInfo.samples = VK_SAMPLE_COUNT_1_BIT; imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    VkImage image; assert(vkCreateImage(device, &imageInfo, nullptr, &image) == VK_SUCCESS);
    VkMemoryRequirements req; vkGetImageMemoryRequirements(device, image, &req); uint32_t memoryType;
    assert(vk_get_memory_type(&bundle, req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, &memoryType));
    VkMemoryAllocateInfo allocation{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO}; allocation.allocationSize = req.size; allocation.memoryTypeIndex = memoryType;
    VkDeviceMemory imageMemory; assert(vkAllocateMemory(device, &allocation, nullptr, &imageMemory) == VK_SUCCESS);
    assert(vkBindImageMemory(device, image, imageMemory, 0) == VK_SUCCESS);
    VkBufferCreateInfo stagingInfo{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO}; stagingInfo.size = 16 * 8 * 4; stagingInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    VkBuffer staging; assert(vkCreateBuffer(device, &stagingInfo, nullptr, &staging) == VK_SUCCESS);
    vkGetBufferMemoryRequirements(device, staging, &req);
    assert(vk_get_memory_type(&bundle, req.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &memoryType));
    allocation.allocationSize = req.size; allocation.memoryTypeIndex = memoryType;
    VkDeviceMemory stagingMemory; assert(vkAllocateMemory(device, &allocation, nullptr, &stagingMemory) == VK_SUCCESS);
    assert(vkBindBufferMemory(device, staging, stagingMemory, 0) == VK_SUCCESS);
    uint8_t *data; assert(vkMapMemory(device, stagingMemory, 0, 16 * 8 * 4, 0, reinterpret_cast<void**>(&data)) == VK_SUCCESS);
    for (int y = 0; y < 8; ++y) for (int x = 0; x < 16; ++x) {
        auto p = data + (y * 16 + x) * 4; p[0] = x < 8 ? 255 : 0; p[1] = 0; p[2] = x < 8 ? 0 : 255; p[3] = 255;
    }
    vkUnmapMemory(device, stagingMemory);
    VkCommandBufferAllocateInfo cmdInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO}; cmdInfo.commandPool = pool; cmdInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY; cmdInfo.commandBufferCount = 1;
    VkCommandBuffer upload; assert(vkAllocateCommandBuffers(device, &cmdInfo, &upload) == VK_SUCCESS);
    VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO}; assert(vkBeginCommandBuffer(upload, &begin) == VK_SUCCESS);
    VkImageMemoryBarrier barrier{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER}; barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED; barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    barrier.srcQueueFamilyIndex = barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; barrier.image = image; barrier.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT,0,1,0,1}; barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    vkCmdPipelineBarrier(upload, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0,nullptr,0,nullptr,1,&barrier);
    VkBufferImageCopy copy{}; copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT,0,0,1}; copy.imageExtent = {16,8,1};
    vkCmdCopyBufferToImage(upload, staging, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
    barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; barrier.newLayout = VK_IMAGE_LAYOUT_GENERAL;
    barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    vkCmdPipelineBarrier(upload, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0,0,nullptr,0,nullptr,1,&barrier);
    assert(vkEndCommandBuffer(upload) == VK_SUCCESS);
    VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO}; submit.commandBufferCount = 1; submit.pCommandBuffers = &upload;
    assert(vkQueueSubmit(queue, 1, &submit, VK_NULL_HANDLE) == VK_SUCCESS && vkQueueWaitIdle(queue) == VK_SUCCESS);
    ql_hmd hmd{}; for (auto &p : hmd.foveation_parameters) { p.x.scale = 1; p.y.scale = 1; }
    {
        MirrorCapture capture(&bundle, pool, VK_FORMAT_R8G8B8A8_UNORM, 16, 8, 1, &image, &hmd);
        assert(capture.available());
        bool ready = false;
        for (int i = 0; i < 100 && !ready; ++i) {
            ready = capture.ready();
            if (!ready) std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        assert(ready);
        VkCommandBuffer readback = capture.command(0); submit.pCommandBuffers = &readback;
        assert(vkQueueSubmit(queue, 1, &submit, VK_NULL_HANDLE) == VK_SUCCESS && vkQueueWaitIdle(queue) == VK_SUCCESS);
        capture.submit(0, 42);
        reader.join();
        assert(capture.testTargetFPS() == 30);
        assert(received.magic == 0x56524d50 && received.version == 1 && received.width == 8 && received.height == 8 && received.format == 2 && received.frame == 42);
        assert(pixels.size() == 8 * 8 * 8 && pixels[0] == 255 && pixels[2] == 0 && pixels[8 * 8 * 4] == 0 && pixels[8 * 8 * 4 + 2] == 255);
        auto checkRate = [&](const char* setting, int expected) {
            { std::ofstream output(fpsFile, std::ios::trunc); output << setting; assert(output.good()); }
            std::this_thread::sleep_for(std::chrono::milliseconds(1100));
            std::thread nextReader([&] {
                int fd = accept(listener, nullptr, nullptr); assert(fd >= 0);
                Header next{}; assert(readAll(fd, &next, sizeof(next)));
                std::vector<uint8_t> payload(size_t(next.eyeBytes) * 2);
                assert(readAll(fd, payload.data(), payload.size())); close(fd);
            });
            bool canCapture = false;
            for (int i = 0; i < 100 && !canCapture; ++i) {
                canCapture = capture.ready();
                if (!canCapture) std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            assert(canCapture);
            VkCommandBuffer command = capture.command(0); submit.pCommandBuffers = &command;
            assert(vkQueueSubmit(queue, 1, &submit, VK_NULL_HANDLE) == VK_SUCCESS && vkQueueWaitIdle(queue) == VK_SUCCESS);
            capture.submit(0, 43);
            nextReader.join();
            assert(capture.testTargetFPS() == expected);
        };
        checkRate("20", 20);
        checkRate("invalid", 10);
    }
    vkFreeCommandBuffers(device, pool, 1, &upload); vkDestroyBuffer(device, staging, nullptr); vkFreeMemory(device, stagingMemory, nullptr);
    vkDestroyImage(device, image, nullptr); vkFreeMemory(device, imageMemory, nullptr); vkDestroyCommandPool(device, pool, nullptr);
    vkDestroyDevice(device, nullptr); vkDestroyInstance(instance, nullptr); close(listener); unlink(name.c_str());
    unlink(fpsFile.c_str());
    alarm(0);
    puts("PASS: compositor GPU readback and stereo transport");
}
