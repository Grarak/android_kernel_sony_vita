/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef PSP2SGX_GEM_H
#define PSP2SGX_GEM_H

#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_mm.h>

struct dma_buf;
struct drm_printer;
struct psp2sgx_device;
struct psp2sgx_vm;

/*
 * What every buffer object has, whichever memory backs it: the GEM object
 * is a psp2sgx_shmem_bo (system memory) or a psp2sgx_cdram_bo (CDRAM,
 * psp2sgx_cdram.c).
 */
struct psp2sgx_bo {
	struct drm_gem_object *gem;
	u32 heap;			/* PSP2SGX_BO_HEAP_* */
	bool counted;			/* in the device's system memory totals */
	struct mutex lock;		/* maps */
	struct list_head maps;
};

struct psp2sgx_shmem_bo {
	struct drm_gem_shmem_object base;
	struct psp2sgx_bo bo;
};

/* A physically contiguous part of the CDRAM region */
struct psp2sgx_cdram_bo {
	struct drm_gem_object base;
	struct psp2sgx_bo bo;
	struct drm_mm_node node;	/* node.start is the physical address */
};

/* Where a buffer object sits in one address space */
struct psp2sgx_bo_map {
	struct list_head link;
	struct psp2sgx_vm *vm;
	struct drm_mm_node node;
	unsigned int refs;		/* open handles in the file, plus jobs */
};

extern const struct drm_gem_object_funcs psp2sgx_cdram_funcs;

static inline bool psp2sgx_gem_is_cdram(const struct drm_gem_object *obj)
{
	return obj->funcs == &psp2sgx_cdram_funcs;
}

static inline struct psp2sgx_cdram_bo *
to_psp2sgx_cdram_bo(struct drm_gem_object *obj)
{
	return container_of(obj, struct psp2sgx_cdram_bo, base);
}

static inline struct psp2sgx_bo *to_psp2sgx_bo(struct drm_gem_object *obj)
{
	if (psp2sgx_gem_is_cdram(obj))
		return &to_psp2sgx_cdram_bo(obj)->bo;
	return &container_of(to_drm_gem_shmem_obj(obj), struct psp2sgx_shmem_bo,
			     base)->bo;
}

/* psp2sgx_gem.c */
void psp2sgx_bo_init(struct psp2sgx_bo *bo, struct drm_gem_object *gem);
void psp2sgx_bo_fini(struct psp2sgx_bo *bo);
int psp2sgx_gem_open(struct drm_gem_object *obj, struct drm_file *file);
void psp2sgx_gem_close(struct drm_gem_object *obj, struct drm_file *file);

struct drm_gem_object *psp2sgx_gem_create_object(struct drm_device *drm,
						 size_t size);
int psp2sgx_bo_map_get(struct psp2sgx_bo *bo, struct psp2sgx_vm *vm, u32 *va);
void psp2sgx_bo_map_put(struct psp2sgx_bo *bo, struct psp2sgx_vm *vm);

struct drm_gem_object *psp2sgx_gem_prime_import(struct drm_device *drm,
						struct dma_buf *dma_buf);
void psp2sgx_show_fdinfo(struct drm_printer *p, struct drm_file *file);

int psp2sgx_ioctl_bo_create(struct drm_device *drm, void *data,
			    struct drm_file *file);
int psp2sgx_ioctl_bo_info(struct drm_device *drm, void *data,
			  struct drm_file *file);
int psp2sgx_ioctl_bo_wait(struct drm_device *drm, void *data,
			  struct drm_file *file);

/* psp2sgx_cdram.c */
int psp2sgx_cdram_init(struct psp2sgx_device *sgx);
struct drm_gem_object *psp2sgx_cdram_create(struct psp2sgx_device *sgx,
					    size_t size);
bool psp2sgx_cdram_is_dmabuf(struct drm_device *drm, struct dma_buf *dma_buf);
void psp2sgx_cdram_debugfs_init(struct psp2sgx_device *sgx);

#endif
