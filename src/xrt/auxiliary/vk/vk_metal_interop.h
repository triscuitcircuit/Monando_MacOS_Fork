// SPDX-License-Identifier: BSL-1.0
#ifndef NEWT_VK_METAL_INTEROP_H
#define NEWT_VK_METAL_INTEROP_H
#include <dlfcn.h>
#include <mach-o/dyld.h>
#include "vk/vk_helpers.h"

/* Resolve against the implementation owning this device, never RTLD_DEFAULT:
 * the application and compositor can embed different MoltenVK versions. */
static inline void *
vk_metal_proc(struct vk_bundle *vk, const char *name)
{
	void *proc = NULL;
	Dl_info owner = {0};
	void *create_image = (void *)vk->vkGetDeviceProcAddr(vk->device, "vkCreateImage");
	if (!dladdr(create_image, &owner) || owner.dli_fname == NULL) return NULL;
	// Godot statically embeds MoltenVK in its executable. An executable cannot
	// be dlopened as a dylib, but its own exported symbols can be queried.
	if (owner.dli_fbase == (const void *)_dyld_get_image_header(0)) {
		return dlsym(RTLD_MAIN_ONLY, name);
	}
	void *library = dlopen(owner.dli_fname, RTLD_LAZY | RTLD_NOLOAD);
	if (library == NULL) return NULL;
	proc = dlsym(library, name);
	dlclose(library);
	return proc;
}
typedef void (*newt_get_metal_texture)(VkImage, void **);
typedef VkResult (*newt_set_metal_texture)(VkImage, void *);

#endif
