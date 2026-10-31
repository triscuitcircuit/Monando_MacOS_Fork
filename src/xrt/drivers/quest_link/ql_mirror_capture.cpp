// SPDX-License-Identifier: BSL-1.0
#include "ql_mirror_capture.h"
#include "ql_types.h"
#include "vk/vk_helpers.h"
#include <fcntl.h>
#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace xrt::drivers::quest_link {
namespace {
struct Header {
    uint32_t magic = 0x56524d50, version = 1, width = 0, height = 0, format = 0, eyeBytes = 0;
    uint64_t frame = 0;
};
static_assert(sizeof(Header) == 32, "VR mirror wire format");
int viewerFPS() {
    const char* path = std::getenv("QL_MIRROR_FPS_FILE");
    if (!path || path[0] != '/') return 10;
    int fd = open(path, O_RDONLY | O_NOFOLLOW);
    if (fd < 0) return 10;
    struct stat st{};
    char data[16] = {};
    const bool valid = fstat(fd, &st) == 0 && S_ISREG(st.st_mode) && st.st_uid == getuid() && st.st_size > 0 && st.st_size < sizeof(data);
    const ssize_t size = valid ? read(fd, data, sizeof(data) - 1) : -1;
    close(fd);
    if (size <= 0) return 10;
    char* end = nullptr;
    long value = std::strtol(data, &end, 10);
    if (end == data || (*end != '\0' && !(*end == '\n' && end[1] == '\0'))) return 10;
    return value == 10 || value == 20 || value == 30 ? int(value) : 10;
}
bool sendAll(int fd, const void *bytes, size_t length) {
    auto data = static_cast<const uint8_t *>(bytes);
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (length && std::chrono::steady_clock::now() < deadline) {
        ssize_t n = send(fd, data, length, 0);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) return false;
        data += n; length -= size_t(n);
    }
    return length == 0;
}
int connectViewer() {
    const auto name = mirror_socket_path();
    if (name.empty()) return -1;
    struct stat st{};
    if (lstat(name.c_str(), &st) || !S_ISSOCK(st.st_mode) || st.st_uid != getuid()) return -1;
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    timeval timeout{0, 250000};
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    int enabled = 1;
    setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &enabled, sizeof(enabled));
    sockaddr_un address{}; address.sun_family = AF_UNIX;
    if (name.size() >= sizeof(address.sun_path)) { close(fd); return -1; }
    std::memcpy(address.sun_path, name.c_str(), name.size() + 1);
    uid_t uid; gid_t gid;
    if (connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) ||
        getpeereid(fd, &uid, &gid) || uid != getuid()) { close(fd); return -1; }
    return fd;
}
}

std::string mirror_socket_path() {
    const char* path = std::getenv("QL_MIRROR_SOCKET");
    if (!path || path[0] != '/' || std::strlen(path) >= sizeof(sockaddr_un::sun_path)) return {};
    return path;
}

MirrorCapture::MirrorCapture(vk_bundle *device, VkCommandPool commandPool, VkFormat pixelFormat,
                             uint32_t width, uint32_t height, uint32_t count,
                             const VkImage *images, const ql_hmd *hmd) : vk(device), pool(commandPool) {
    if (!vk || !hmd || !images || !count || width % 2 || !width || !height || width > 8192 || height > 4096) return;
    if (pixelFormat == VK_FORMAT_B8G8R8A8_SRGB || pixelFormat == VK_FORMAT_B8G8R8A8_UNORM) format = 1;
    else if (pixelFormat == VK_FORMAT_R8G8B8A8_SRGB || pixelFormat == VK_FORMAT_R8G8B8A8_UNORM) format = 2;
    else return;
    uint32_t eyeWidth = width / 2;
    uint32_t outputWidth = std::min(eyeWidth, uint32_t(960));
    uint32_t outputHeight = std::max(2u, uint32_t((uint64_t(height) * outputWidth / eyeWidth) & ~uint64_t(1)));
    outputWidth &= ~1u;
    if (!map.reset(width, height, outputWidth, outputHeight, [hmd](uint32_t eye, float u, float v) {
        const auto& p = hmd->foveation_parameters[eye];
        return std::array<float, 2>{mirror_source_coordinate(p.x, u), mirror_source_coordinate(p.y, v)};
    })) return;
    sourceBytes = size_t(width) * height * 4;
    slots.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        auto& slot = slots[i];
        VkBufferCreateInfo buffer{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        buffer.size = sourceBytes; buffer.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
        buffer.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        if (vk->vkCreateBuffer(vk->device, &buffer, nullptr, &slot.buffer) != VK_SUCCESS) { cleanup(); return; }
        VkMemoryRequirements requirements{};
        vk->vkGetBufferMemoryRequirements(vk->device, slot.buffer, &requirements);
        uint32_t memoryType = 0;
        if (!vk_get_memory_type(vk, requirements.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &memoryType)) { cleanup(); return; }
        VkMemoryAllocateInfo allocation{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        allocation.allocationSize = requirements.size; allocation.memoryTypeIndex = memoryType;
        if (vk->vkAllocateMemory(vk->device, &allocation, nullptr, &slot.memory) != VK_SUCCESS ||
            vk->vkBindBufferMemory(vk->device, slot.buffer, slot.memory, 0) != VK_SUCCESS ||
            vk->vkMapMemory(vk->device, slot.memory, 0, sourceBytes, 0, &slot.mapped) != VK_SUCCESS) { cleanup(); return; }
        VkCommandBufferAllocateInfo cmdInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        cmdInfo.commandPool = pool; cmdInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY; cmdInfo.commandBufferCount = 1;
        if (vk->vkAllocateCommandBuffers(vk->device, &cmdInfo, &slot.command) != VK_SUCCESS) { cleanup(); return; }
        VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        if (vk->vkBeginCommandBuffer(slot.command, &begin) != VK_SUCCESS) { cleanup(); return; }
        VkImageMemoryBarrier image{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        image.srcAccessMask = VK_ACCESS_SHADER_READ_BIT; image.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        image.oldLayout = VK_IMAGE_LAYOUT_GENERAL; image.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        image.srcQueueFamilyIndex = image.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        image.image = images[i]; image.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        vk->vkCmdPipelineBarrier(slot.command, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
                                 0, 0, nullptr, 0, nullptr, 1, &image);
        VkBufferImageCopy region{}; region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
        region.imageExtent = {width, height, 1};
        vk->vkCmdCopyImageToBuffer(slot.command, images[i], VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, slot.buffer, 1, &region);
        image.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT; image.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        image.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL; image.newLayout = VK_IMAGE_LAYOUT_GENERAL;
        VkBufferMemoryBarrier host{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        host.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; host.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        host.srcQueueFamilyIndex = host.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        host.buffer = slot.buffer; host.offset = 0; host.size = VK_WHOLE_SIZE;
        vk->vkCmdPipelineBarrier(slot.command, VK_PIPELINE_STAGE_TRANSFER_BIT,
                                 VK_PIPELINE_STAGE_ALL_COMMANDS_BIT | VK_PIPELINE_STAGE_HOST_BIT,
                                 0, 0, nullptr, 1, &host, 1, &image);
        if (vk->vkEndCommandBuffer(slot.command) != VK_SUCCESS) { cleanup(); return; }
    }
    try { worker = std::thread([this] { run(); }); }
    catch (...) { cleanup(); return; }
    valid = true;
}
MirrorCapture::~MirrorCapture() {
    { std::lock_guard<std::mutex> lock(mutex); stop = true; pending.reset(); }
    wake.notify_one();
    if (worker.joinable()) worker.join();
    cleanup();
}
void MirrorCapture::cleanup() {
    for (auto& slot : slots) {
        if (slot.command) vk->vkFreeCommandBuffers(vk->device, pool, 1, &slot.command);
        if (slot.mapped) vk->vkUnmapMemory(vk->device, slot.memory);
        if (slot.buffer) vk->vkDestroyBuffer(vk->device, slot.buffer, nullptr);
        if (slot.memory) vk->vkFreeMemory(vk->device, slot.memory, nullptr);
        slot = {};
    }
    valid = false;
}
bool MirrorCapture::ready() {
    if (!valid) return false;
    const auto now = std::chrono::steady_clock::now();
    if (now - lastProbe >= std::chrono::milliseconds(250)) {
        struct stat st{}; const auto name = mirror_socket_path();
        viewer = !lstat(name.c_str(), &st) && S_ISSOCK(st.st_mode) && st.st_uid == getuid();
        lastProbe = now;
    }
    const int fps = targetFPS.load(std::memory_order_relaxed);
    if (!viewer || now - lastCapture < std::chrono::microseconds(1000000 / fps)) return false;
    std::unique_lock<std::mutex> lock(mutex, std::try_to_lock);
    if (!lock || pending || busy) return false;
    lastCapture = now; return true;
}
VkCommandBuffer MirrorCapture::command(uint32_t index) const {
    return valid && index < slots.size() ? slots[index].command : VK_NULL_HANDLE;
}
void MirrorCapture::submit(uint32_t index, uint64_t frame) {
    if (!valid || index >= slots.size()) return;
    std::unique_lock<std::mutex> lock(mutex, std::try_to_lock);
    if (!lock || pending || busy) return;
    try {
        auto next = std::make_unique<Frame>(); next->number = frame;
        const auto bytes = static_cast<const uint8_t *>(slots[index].mapped);
        next->pixels.assign(bytes, bytes + sourceBytes);
        pending = std::move(next); wake.notify_one();
    } catch (...) {}
}
void MirrorCapture::run() {
    auto lastSettingsRead = std::chrono::steady_clock::time_point{};
    for (;;) {
        std::unique_ptr<Frame> frame;
        {
            std::unique_lock<std::mutex> lock(mutex);
            wake.wait(lock, [this] { return stop || pending; });
            if (stop) return;
            frame = std::move(pending); busy = true;
        }
        try {
            const auto now = std::chrono::steady_clock::now();
            if (now - lastSettingsRead >= std::chrono::seconds(1)) {
                targetFPS.store(viewerFPS(), std::memory_order_relaxed);
                lastSettingsRead = now;
            }
            std::vector<uint8_t> eyes;
            if (map.convert(frame->pixels.data(), frame->pixels.size(), eyes)) {
                const int fd = connectViewer();
                if (fd >= 0) {
                    Header header; header.width = map.width; header.height = map.height; header.format = format;
                    header.eyeBytes = map.width * map.height * 4; header.frame = frame->number;
                    if (sendAll(fd, &header, sizeof(header))) sendAll(fd, eyes.data(), eyes.size());
                    close(fd);
                }
            }
        } catch (...) {}
        { std::lock_guard<std::mutex> lock(mutex); busy = false; }
    }
}
}
