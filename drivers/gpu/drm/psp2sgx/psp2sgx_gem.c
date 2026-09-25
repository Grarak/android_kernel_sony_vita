// SPDX-License-Identifier: GPL-2.0-only
/*
 * psp2sgx: buffer objects.
 *
 * A buffer object lives in system memory (shmem) or, when userspace asks
 * for it with PSP2SGX_BO_CDRAM and the device tree gives the GPU a region,
 * in CDRAM (psp2sgx_cdram.c).  Both kinds are mapped write-combined for the
 * CPU: the GPU reads memory through its own caches and the system-level
 * cache, never snooping the CPU's, so userspace writes go straight to
 * memory.  A buffer object gets a GPU address in each address space (open
 * file) that holds a handle to it, in the range its heap names; jobs may
 * mix both kinds.
 */
#include <linux/dma-buf.h>
#include <linux/dma-resv.h>
#include <linux/slab.h>

#include <drm/drm_file.h>
#include <drm/drm_prime.h>
#include <drm/drm_print.h>
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

void psp2sgx_bo_init(struct psp2sgx_bo *bo, struct drm_gem_object *gem)
{
	bo->gem = gem;
	mutex_init(&bo->lock);
	INIT_LIST_HEAD(&bo->maps);
}

void psp2sgx_bo_fini(struct psp2sgx_bo *bo)
{
	WARN_ON(!list_empty(&bo->maps));
	mutex_destroy(&bo->lock);
}

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
	struct drm_gem_object *obj = bo->gem;
	struct psp2sgx_bo_map *m;
	struct sg_table *sgt = NULL;
	int err;

	mutex_lock(&bo->lock);
	m = find_map(bo, vm);
	if (m) {
		m->refs++;
		goto out;
	}
	if (!psp2sgx_gem_is_cdram(obj)) {
		sgt = drm_gem_shmem_get_pages_sgt(to_drm_gem_shmem_obj(obj));
		if (IS_ERR(sgt)) {
			err = PTR_ERR(sgt);
			goto err;
		}
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
		if (sgt)
			err = psp2sgx_vm_map_sgt(vm, m->node.start, sgt,
						 obj->size, 0);
		else
			err = psp2sgx_vm_map_contig(vm, m->node.start,
						    to_psp2sgx_cdram_bo(obj)->node.start,
						    obj->size, 0);
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

int psp2sgx_gem_open(struct drm_gem_object *obj, struct drm_file *file)
{
	struct psp2sgx_file *pf = file->driver_priv;

	return psp2sgx_bo_map_get(to_psp2sgx_bo(obj), pf->vm, NULL);
}

void psp2sgx_gem_close(struct drm_gem_object *obj, struct drm_file *file)
{
	struct psp2sgx_file *pf = file->driver_priv;

	psp2sgx_bo_map_put(to_psp2sgx_bo(obj), pf->vm);
}

/* ---------------------------------------------------------- system memory */

static void psp2sgx_shmem_free(struct drm_gem_object *obj)
{
	struct psp2sgx_device *sgx = to_psp2sgx(obj->dev);
	struct psp2sgx_bo *bo = to_psp2sgx_bo(obj);

	if (bo->counted) {
		atomic64_sub(obj->size, &sgx->shmem_bytes);
		atomic_dec(&sgx->shmem_objects);
	}
	psp2sgx_bo_fini(bo);
	drm_gem_shmem_free(to_drm_gem_shmem_obj(obj));
}

static const struct drm_gem_object_funcs psp2sgx_shmem_funcs = {
	.free = psp2sgx_shmem_free,
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

/* drm_driver.gem_create_object: shmem objects, created or imported */
struct drm_gem_object *psp2sgx_gem_create_object(struct drm_device *drm,
						 size_t size)
{
	struct psp2sgx_shmem_bo *sbo = kzalloc_obj(*sbo);

	if (!sbo)
		return ERR_PTR(-ENOMEM);
	psp2sgx_bo_init(&sbo->bo, &sbo->base.base);
	sbo->base.map_wc = true;
	sbo->base.base.funcs = &psp2sgx_shmem_funcs;
	return &sbo->base.base;
}

/* -------------------------------------------------------------- dma-buf */

/*
 * CDRAM objects export dma-bufs of their own (psp2sgx_cdram.c); importing
 * one of those, or a shmem object's, into this device takes the object
 * itself.
 */
struct drm_gem_object *psp2sgx_gem_prime_import(struct drm_device *drm,
						struct dma_buf *dma_buf)
{
	if (psp2sgx_cdram_is_dmabuf(drm, dma_buf)) {
		struct drm_gem_object *obj = dma_buf->priv;

		drm_gem_object_get(obj);
		return obj;
	}
	return drm_gem_prime_import(drm, dma_buf);
}

/* fdinfo: the file's buffer objects by the memory they are in */
void psp2sgx_show_fdinfo(struct drm_printer *p, struct drm_file *file)
{
	static const char * const region[] = { "system", "cdram" };
	struct drm_memory_stats stats[ARRAY_SIZE(region)] = {};
	struct drm_gem_object *obj;
	unsigned int i;
	int id;

	spin_lock(&file->table_lock);
	idr_for_each_entry(&file->object_idr, obj, id) {
		struct drm_memory_stats *s = &stats[psp2sgx_gem_is_cdram(obj)];

		if (drm_gem_object_is_shared_for_memory_stats(obj))
			s->shared += obj->size;
		else
			s->private += obj->size;
		/* a handle maps the object, which needs its pages */
		s->resident += obj->size;
		if (!dma_resv_test_signaled(obj->resv, dma_resv_usage_rw(true)))
			s->active += obj->size;
	}
	spin_unlock(&file->table_lock);

	for (i = 0; i < ARRAY_SIZE(region); i++)
		drm_print_memory_stats(p, &stats[i], DRM_GEM_OBJECT_RESIDENT |
				       DRM_GEM_OBJECT_ACTIVE, region[i]);
}

/* ---------------------------------------------------------------- ioctls */

static struct drm_gem_object *create_shmem(struct psp2sgx_device *sgx,
					   size_t size)
{
	struct drm_gem_shmem_object *shmem;

	shmem = drm_gem_shmem_create(&sgx->drm, size);
	if (IS_ERR(shmem))
		return ERR_CAST(shmem);
	to_psp2sgx_bo(&shmem->base)->counted = true;
	atomic64_add(shmem->base.size, &sgx->shmem_bytes);
	atomic_inc(&sgx->shmem_objects);
	return &shmem->base;
}

int psp2sgx_ioctl_bo_create(struct drm_device *drm, void *data,
			    struct drm_file *file)
{
	struct psp2sgx_device *sgx = to_psp2sgx(drm);
	struct drm_psp2sgx_bo_create *args = data;
	struct psp2sgx_file *pf = file->driver_priv;
	struct drm_gem_object *obj = ERR_PTR(-ENOSPC);
	struct psp2sgx_bo *bo;
	u32 heap = args->flags & PSP2SGX_BO_HEAP_MASK;
	int err;

	if (args->flags & ~(PSP2SGX_BO_HEAP_MASK | PSP2SGX_BO_CDRAM) ||
	    heap >= ARRAY_SIZE(heap_range) || args->placement || !args->size ||
	    args->size > SZ_256M)
		return -EINVAL;

	/*
	 * CDRAM is a preference: without room there (or without a region at
	 * all) the object goes to system memory, and placement says so.
	 */
	if (args->flags & PSP2SGX_BO_CDRAM) {
		obj = psp2sgx_cdram_create(sgx, args->size);
		if (IS_ERR(obj) && PTR_ERR(obj) != -ENOSPC)
			return PTR_ERR(obj);
	}
	if (IS_ERR(obj))
		obj = create_shmem(sgx, args->size);
	if (IS_ERR(obj))
		return PTR_ERR(obj);
	bo = to_psp2sgx_bo(obj);
	bo->heap = heap;
	args->placement = psp2sgx_gem_is_cdram(obj) ? PSP2SGX_BO_PLACEMENT_CDRAM :
						      PSP2SGX_BO_PLACEMENT_SYSTEM;

	/* the handle's open maps it into this file's address space */
	err = drm_gem_handle_create(file, obj, &args->handle);
	drm_gem_object_put(obj);
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
