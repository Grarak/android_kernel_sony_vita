/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef PSP2SGX_GEM_H
#define PSP2SGX_GEM_H

#include <drm/drm_gem_shmem_helper.h>

struct psp2sgx_vm;

struct psp2sgx_bo {
	struct drm_gem_shmem_object base;
	u32 heap;			/* PSP2SGX_BO_HEAP_* */
	struct mutex lock;		/* maps */
	struct list_head maps;
};

/* Where a buffer object sits in one address space */
struct psp2sgx_bo_map {
	struct list_head link;
	struct psp2sgx_vm *vm;
	struct drm_mm_node node;
	unsigned int refs;		/* open handles in the file, plus jobs */
};

static inline struct psp2sgx_bo *to_psp2sgx_bo(struct drm_gem_object *obj)
{
	return container_of(to_drm_gem_shmem_obj(obj), struct psp2sgx_bo, base);
}

struct drm_gem_object *psp2sgx_gem_create_object(struct drm_device *drm,
						 size_t size);
int psp2sgx_bo_map_get(struct psp2sgx_bo *bo, struct psp2sgx_vm *vm, u32 *va);
void psp2sgx_bo_map_put(struct psp2sgx_bo *bo, struct psp2sgx_vm *vm);

int psp2sgx_ioctl_bo_create(struct drm_device *drm, void *data,
			    struct drm_file *file);
int psp2sgx_ioctl_bo_info(struct drm_device *drm, void *data,
			  struct drm_file *file);
int psp2sgx_ioctl_bo_wait(struct drm_device *drm, void *data,
			  struct drm_file *file);

#endif
