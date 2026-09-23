// SPDX-License-Identifier: GPL-2.0-only
/*
 * psp2sgx: buffer objects.
 *
 * Buffer objects are shmem-backed and mapped write-combined for the CPU:
 * the GPU reads memory through its own caches and the system-level cache,
 * never snooping the CPU's, so userspace writes go straight to memory.  A
 * buffer object gets a GPU address in each address space (open file) that
 * holds a handle to it, in the range its heap names.
 */
#include <linux/dma-resv.h>
#include <linux/slab.h>

#include <drm/drm_file.h>
#include <drm/drm_utils.h>
#include <drm/psp2sgx_drm.h>

#include "psp2sgx_drv.h"
#include "psp2sgx_gem.h"

static const struct {
	u32 start, end;
} heap_range[] = {
	[PSP2SGX_BO_HEAP_GENERAL] = { PSP2SGX_GENERAL_BASE, PSP2SGX_GENERAL_END },
	[PSP2SGX_BO_HEAP_CODE_VERTEX] = { PSP2SGX_CODE_VERTEX_BASE,
					  PSP2SGX_CODE_VERTEX_BASE + PSP2SGX_CODE_WINDOW },
	[PSP2SGX_BO_HEAP_CODE_FRAGMENT] = { PSP2SGX_CODE_FRAGMENT_BASE,
					    PSP2SGX_CODE_FRAGMENT_BASE + PSP2SGX_CODE_WINDOW },
	[PSP2SGX_BO_HEAP_PDS] = { PSP2SGX_PDS_BASE, PSP2SGX_PDS_END },
};

static struct psp2sgx_bo_map *find_map(struct psp2sgx_bo *bo,
				       struct psp2sgx_vm *vm)
{
	struct psp2sgx_bo_map *m;

	list_for_each_entry(m, &bo->maps, link)
		if (m->vm == vm)
			return m;
	return NULL;
}

int psp2sgx_bo_map_get(struct psp2sgx_bo *bo, struct psp2sgx_vm *vm, u32 *va)
{
	struct drm_gem_object *obj = &bo->base.base;
	struct psp2sgx_bo_map *m;
	struct sg_table *sgt;
	int err;

	mutex_lock(&bo->lock);
	m = find_map(bo, vm);
	if (m) {
		m->refs++;
		goto out;
	}
	sgt = drm_gem_shmem_get_pages_sgt(&bo->base);
	if (IS_ERR(sgt)) {
		err = PTR_ERR(sgt);
		goto err;
	}
	m = kzalloc(sizeof(*m), GFP_KERNEL);
	if (!m) {
		err = -ENOMEM;
		goto err;
	}
	mutex_lock(&vm->lock);
	err = drm_mm_insert_node_in_range(&vm->mm, &m->node, obj->size,
					  SGX_MMU_PAGE_SIZE, 0,
					  heap_range[bo->heap].start,
					  heap_range[bo->heap].end,
					  DRM_MM_INSERT_BEST);
	if (!err) {
		err = psp2sgx_vm_map_sgt(vm, m->node.start, sgt, obj->size, 0);
		if (err)
			drm_mm_remove_node(&m->node);
	}
	mutex_unlock(&vm->lock);
	if (err) {
		kfree(m);
		goto err;
	}
	m->vm = psp2sgx_vm_get(vm);
	m->refs = 1;
	list_add(&m->link, &bo->maps);
out:
	if (va)
		*va = m->node.start;
	mutex_unlock(&bo->lock);
	return 0;
err:
	mutex_unlock(&bo->lock);
	return err;
}

void psp2sgx_bo_map_put(struct psp2sgx_bo *bo, struct psp2sgx_vm *vm)
{
	struct psp2sgx_bo_map *m;

	mutex_lock(&bo->lock);
	m = find_map(bo, vm);
	if (WARN_ON(!m) || --m->refs) {
		mutex_unlock(&bo->lock);
		return;
	}
	list_del(&m->link);
	mutex_unlock(&bo->lock);

	mutex_lock(&vm->lock);
	psp2sgx_vm_unmap(vm, m->node.start, m->node.size);
	drm_mm_remove_node(&m->node);
	mutex_unlock(&vm->lock);
	psp2sgx_vm_put(vm);
	kfree(m);
}

static int psp2sgx_gem_open(struct drm_gem_object *obj, struct drm_file *file)
{
	struct psp2sgx_file *pf = file->driver_priv;

	return psp2sgx_bo_map_get(to_psp2sgx_bo(obj), pf->vm, NULL);
}

static void psp2sgx_gem_close(struct drm_gem_object *obj, struct drm_file *file)
{
	struct psp2sgx_file *pf = file->driver_priv;

	psp2sgx_bo_map_put(to_psp2sgx_bo(obj), pf->vm);
}

static void psp2sgx_gem_free(struct drm_gem_object *obj)
{
	struct psp2sgx_bo *bo = to_psp2sgx_bo(obj);

	WARN_ON(!list_empty(&bo->maps));
	mutex_destroy(&bo->lock);
	drm_gem_shmem_free(&bo->base);
}

static const struct drm_gem_object_funcs psp2sgx_gem_funcs = {
	.free = psp2sgx_gem_free,
	.open = psp2sgx_gem_open,
	.close = psp2sgx_gem_close,
	.print_info = drm_gem_shmem_object_print_info,
	.pin = drm_gem_shmem_object_pin,
	.unpin = drm_gem_shmem_object_unpin,
	.get_sg_table = drm_gem_shmem_object_get_sg_table,
	.vmap = drm_gem_shmem_object_vmap,
	.vunmap = drm_gem_shmem_object_vunmap,
	.mmap = drm_gem_shmem_object_mmap,
	.vm_ops = &drm_gem_shmem_vm_ops,
};

struct drm_gem_object *psp2sgx_gem_create_object(struct drm_device *drm,
						 size_t size)
{
	struct psp2sgx_bo *bo = kzalloc(sizeof(*bo), GFP_KERNEL);

	if (!bo)
		return ERR_PTR(-ENOMEM);
	mutex_init(&bo->lock);
	INIT_LIST_HEAD(&bo->maps);
	bo->base.map_wc = true;
	bo->base.base.funcs = &psp2sgx_gem_funcs;
	return &bo->base.base;
}

int psp2sgx_ioctl_bo_create(struct drm_device *drm, void *data,
			    struct drm_file *file)
{
	struct drm_psp2sgx_bo_create *args = data;
	struct psp2sgx_file *pf = file->driver_priv;
	struct drm_gem_shmem_object *shmem;
	struct psp2sgx_bo *bo;
	u32 heap = args->flags & PSP2SGX_BO_HEAP_MASK;
	int err;

	if (args->flags & ~PSP2SGX_BO_HEAP_MASK || heap >= ARRAY_SIZE(heap_range) ||
	    args->pad || !args->size || args->size > SZ_256M)
		return -EINVAL;

	shmem = drm_gem_shmem_create(drm, args->size);
	if (IS_ERR(shmem))
		return PTR_ERR(shmem);
	bo = container_of(shmem, struct psp2sgx_bo, base);
	bo->heap = heap;

	/* the handle's open maps it into this file's address space */
	err = drm_gem_handle_create(file, &shmem->base, &args->handle);
	drm_gem_object_put(&shmem->base);
	if (err)
		return err;
	mutex_lock(&bo->lock);
	args->gpu_va = find_map(bo, pf->vm)->node.start;
	mutex_unlock(&bo->lock);
	return 0;
}

int psp2sgx_ioctl_bo_info(struct drm_device *drm, void *data,
			  struct drm_file *file)
{
	struct drm_psp2sgx_bo_info *args = data;
	struct psp2sgx_file *pf = file->driver_priv;
	struct drm_gem_object *obj;
	struct psp2sgx_bo *bo;
	struct psp2sgx_bo_map *m;
	int err;

	obj = drm_gem_object_lookup(file, args->handle);
	if (!obj)
		return -ENOENT;
	bo = to_psp2sgx_bo(obj);
	err = drm_gem_create_mmap_offset(obj);
	if (!err) {
		args->mmap_offset = drm_vma_node_offset_addr(&obj->vma_node);
		args->size = obj->size;
		mutex_lock(&bo->lock);
		m = find_map(bo, pf->vm);
		args->gpu_va = m ? m->node.start : 0;
		mutex_unlock(&bo->lock);
		if (!m)
			err = -ENOENT;
	}
	drm_gem_object_put(obj);
	return err;
}

int psp2sgx_ioctl_bo_wait(struct drm_device *drm, void *data,
			  struct drm_file *file)
{
	struct drm_psp2sgx_bo_wait *args = data;
	long timeout;
	int err;

	if (args->op & ~(PSP2SGX_BO_WAIT_READ | PSP2SGX_BO_WAIT_WRITE))
		return -EINVAL;
	if (!args->op)
		return 0;
	timeout = drm_timeout_abs_to_jiffies(args->timeout_ns);
	err = drm_gem_dma_resv_wait(file, args->handle,
				    args->op & PSP2SGX_BO_WAIT_WRITE, timeout);
	if (err == -ETIME)
		err = timeout ? -ETIMEDOUT : -EBUSY;
	return err;
}
