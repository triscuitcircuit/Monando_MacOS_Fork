/*
 * WiVRn VR streaming
 * Copyright (C) 2022  Guillaume Meunier <guillaume.meunier@centraliens.net>
 * Copyright (C) 2022  Patrick Nicolas <patricknicolas@laposte.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "vk/vk_helpers.h"
#include "vk/vk_cmd.h"
#include "vk/vk_cmd_pool.h"
#include <vector>
#include <vulkan/vulkan_core.h>

class YuvConverter
{
public:
	// Per-slot conversion targets: slot i renders/copies only into its own
	// image and host buffer, so pixels for swapchain image i stay paired
	// with that image's pose/timestamps in the encoder.
	struct image_bundle
	{
		VkDeviceMemory image_memory = VK_NULL_HANDLE;
		VkImage image = VK_NULL_HANDLE;
		VkImageView view = VK_NULL_HANDLE;
		VkDeviceMemory buffer_memory = VK_NULL_HANDLE;
		VkBuffer buffer = VK_NULL_HANDLE;
		void * mapped_memory = nullptr;
		VkFramebuffer frame_buffer = VK_NULL_HANDLE;
		VkExtent2D extent;
		VkExtent2D extent_clipped;
		int stride;
	};

	struct plane_shared
	{
		VkShaderModule frag = VK_NULL_HANDLE;
		VkRenderPass render_pass = VK_NULL_HANDLE;
		VkPipeline pipeline = VK_NULL_HANDLE;
		VkExtent2D extent;
		int stride = 0;
	};

	struct yuv_ubo {
	    float uvs[4];
	};

	vk_bundle & vk;
	plane_shared y_plane;
	plane_shared uv_plane;
	std::vector<image_bundle> y_slots;
	std::vector<image_bundle> uv_slots;

	void * y_mapped(int i) const { return y_slots.at(i).mapped_memory; }
	void * uv_mapped(int i) const { return uv_slots.at(i).mapped_memory; }
	int y_stride() const { return y_plane.stride; }
	int uv_stride() const { return uv_plane.stride; }
	VkSampler sampler = VK_NULL_HANDLE;
	VkDescriptorSetLayout descriptor_set_layout = VK_NULL_HANDLE;
	VkDescriptorPool descriptor_pool = VK_NULL_HANDLE;

	struct vk_cmd_pool comp_pool;
	VkCommandPool cmd_pool;

	VkBuffer ubo_buf;
	VkDeviceMemory ubo_mem;
	yuv_ubo ubo_uvs;

	VkShaderModule vert = VK_NULL_HANDLE;

	VkPipelineLayout pipeline_layout = VK_NULL_HANDLE;

	std::vector<VkCommandBuffer> command_buffers;
	std::vector<VkDescriptorSet> descriptor_sets;

	int slice_idx;
	int num_slices;

	YuvConverter(vk_bundle * vk, VkExtent3D extent, int offset_x, int offset_y, int input_width, int input_height, int slice_idx, int num_slices);
	~YuvConverter();

	void SetImages(int num_images, VkImage * images, VkImageView * views);
};
