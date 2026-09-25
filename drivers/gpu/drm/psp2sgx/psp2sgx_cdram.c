// SPDX-License-Identifier: GPL-2.0-only
/*
 * psp2sgx: buffer objects in CDRAM.
 *
 * CDRAM is the Vita's 128 MiB of wide-IO graphics memory at physical
 * 0x20000000-0x27ffffff, non-secure ("VRAM. Graphics bar", wiki
 * Physical_Memory).  It is not system RAM: the boot loader enables it
 * (SMC 0x117, smc_cdram_enable in the wiki's SceExcpmgr table) and puts its
 * framebuffer at its start, and the device tree hands the GPU the rest as a
 * no-map reserved-memory region (memory-region).  The kernel has no linear
 * mapping of it and no struct pages for it.
 *
 * The Vita OS uses it for the same things: it splits it into
 * ScePhyMemPartGameCdram (0x20000000, 112 MiB) and ScePhyMemPartShellCdram
 * (0x27000000, 16 MiB) (SceProcessmgr), libgxm places its parameter
 * buffer there by default (libgxm_es4, "SceGxmParamBuffer" as
 * USER_CDRAM_RW unless the init flags ask for LPDDR), and Sony's samples
 * put display buffers and colour render targets there, depth, vertex data
 * and USSE code in LPDDR.  USER_CDRAM_RW (0x09408060) has the uncached
 * memtype of USER_MAIN_NC_RW, and SceGpuEs4 maps a CDRAM block into the GPU
 * with the same MMU flags as an uncached LPDDR block (gpu_es4): so
 * CPU mappings here are write-combined (normal non-cacheable), as for the
 * shmem objects, and the GPU PTEs are the plain valid ones.  The DMAC on the
 * Video Xbar is the one the wiki calls faster for CDRAM (DMAC page): the ARM
 * reaches CDRAM across the interconnect, so buffers the CPU streams through
 * belong in system memory.
 *
 * Objects are physically contiguous ranges from a drm_mm over the region,
 * page-aligned, zeroed when allocated.  Contiguous ranges keep the GPU and
 * CPU mappings trivial and are what the display controller would need to
 * scan a buffer out.  Nothing is evicted: when the region is full, BO_CREATE
 * places the object in system memory instead (psp2sgx_gem.c).
 *
 * dma-bufs of CDRAM objects carry their own ops: this device imports them
 * back as the same object, other devices only if they take sg tables
 * without pages (peer2peer), as for any device memory.
 */
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/of_reserved_mem.h>
#include <linux/scatterlist.h>
#include <linux/sizes.h>
#include <linux/slab.h>

#include <drm/drm_debugfs.h>
#include <drm/drm_managed.h>
#include <drm/drm_prime.h>
#include <drm/drm_print.h>

#include "psp2sgx_drv.h"
#include "psp2sgx_gem.h"

#define CDRAM_CLEAR_CHUNK	SZ_1M	/* mapped at a time to zero an object */

/* psp2sgx.cdram=0 keeps every object in system memory, for comparisons */
static bool cdram_enable = true;
module_param_named(cdram, cdram_enable, bool, 0444);
MODULE_PARM_DESC(cdram, "Place buffer objects in CDRAM when asked (default: true)");

static void cdram_release(struct drm_device *drm, void *data)
{
	struct psp2sgx_cdram *c = data;

	drm_mm_takedown(&c->mm);
	release_mem_region(c->base, c->size);
}

/*
 * The GPU's CDRAM region from the device tree.  Without one every buffer
 * object is in system memory; that is not an error.
 */
int psp2sgx_cdram_init(struct psp2sgx_device *sgx)
{
	struct psp2sgx_cdram *c = &sgx->cdram;
	struct device *dev = sgx->dev;
	struct resource res;
	int err;

	err = drmm_mutex_init(&sgx->drm, &c->lock);
	if (err)
		return err;
	if (!cdram_enable) {
		dev_info(dev, "CDRAM off (psp2sgx.cdram=0): buffer objects in system memory\n");
		return 0;
	}
	err = of_reserved_mem_region_to_resource(dev->of_node, 0, &res);
	if (err) {
		dev_info(dev, "no CDRAM region: buffer objects in system memory\n");
		return 0;
	}
	/* page-granular, and within the 32 bits a GPU PTE holds */
	if (!PAGE_ALIGNED(res.start) || !PAGE_ALIGNED(resource_size(&res)) ||
	    res.end > U32_MAX) {
		dev_err(dev, "unusable CDRAM region %pR\n", &res);
		return -EINVAL;
	}
	if (!request_mem_region(res.start, resource_size(&res), "psp2sgx cdram")) {
		dev_err(dev, "CDRAM region %pR is busy\n", &res);
		return -EBUSY;
	}
	c->base = res.start;
	c->size = resource_size(&res);
	drm_mm_init(&c->mm, c->base, c->size);
	err = drmm_add_action_or_reset(&sgx->drm, cdram_release, c);
	if (err) {
		c->size = 0;
		return err;
	}
	dev_info(dev, "CDRAM %pR: %llu KiB for buffer objects\n", &res,
		 (unsigned long long)c->size >> 10);
	return 0;
}

static int cdram_clear(phys_addr_t pa, size_t size)
{
	while (size) {
		size_t n = min_t(size_t, size, CDRAM_CLEAR_CHUNK);
		void __iomem *p = ioremap_wc(pa, n);

		if (!p)
			return -ENOMEM;
		memset_io(p, 0, n);
		iounmap(p);
		pa += n;
		size -= n;
	}
	/* out of the write buffer before the GPU or another process looks */
	wmb();
	return 0;
}

/* ------------------------------------------------------------- objects */

static void cdram_free(struct drm_gem_object *obj)
{
	struct psp2sgx_device *sgx = to_psp2sgx(obj->dev);
	struct psp2sgx_cdram_bo *cbo = to_psp2sgx_cdram_bo(obj);
	struct psp2sgx_cdram *c = &sgx->cdram;

	psp2sgx_bo_fini(&cbo->bo);
	drm_gem_object_release(obj);
	mutex_lock(&c->lock);
	c->used -= cbo->node.size;
	c->objects--;
	drm_mm_remove_node(&cbo->node);
	mutex_unlock(&c->lock);
	kfree(cbo);
}

static int cdram_mmap(struct drm_gem_object *obj, struct vm_area_struct *vma)
{
	struct psp2sgx_cdram_bo *cbo = to_psp2sgx_cdram_bo(obj);
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long pgoff;

	if (vma_is_cow_mapping(vma))
		return -EINVAL;
	/* drm_gem_mmap() and drm_gem_prime_mmap() leave the fake offset in */
	vma->vm_pgoff -= drm_vma_node_start(&obj->vma_node);
	pgoff = vma->vm_pgoff;
	if (pgoff >= obj->size >> PAGE_SHIFT ||
	    size > obj->size - (pgoff << PAGE_SHIFT))
		return -EINVAL;

	vm_flags_set(vma, VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_DONTDUMP);
	vma->vm_page_prot = pgprot_writecombine(vm_get_page_prot(vma->vm_flags));
	return io_remap_pfn_range(vma, vma->vm_start,
				  PHYS_PFN(cbo->node.start) + pgoff, size,
				  vma->vm_page_prot);
}

static void cdram_print_info(struct drm_printer *p, unsigned int indent,
			     const struct drm_gem_object *obj)
{
	const struct psp2sgx_cdram_bo *cbo =
		container_of(obj, struct psp2sgx_cdram_bo, base);

	drm_printf_indent(p, indent, "cdram=%#llx\n", cbo->node.start);
}

/* ---------------------------------------------------------------- dma-buf */

static struct sg_table *cdram_map_dma_buf(struct dma_buf_attachment *attach,
					  enum dma_data_direction dir)
{
	struct drm_gem_object *obj = attach->dmabuf->priv;
	struct psp2sgx_cdram_bo *cbo = to_psp2sgx_cdram_bo(obj);
	struct sg_table *sgt;
	dma_addr_t addr;
	int err;

	/* no struct pages behind CDRAM */
	if (!attach->peer2peer)
		return ERR_PTR(-EOPNOTSUPP);

	sgt = kzalloc_obj(*sgt);
	if (!sgt)
		return ERR_PTR(-ENOMEM);
	err = sg_alloc_table(sgt, 1, GFP_KERNEL);
	if (err)
		goto err_free;
	addr = dma_map_resource(attach->dev, cbo->node.start, obj->size, dir,
				DMA_ATTR_SKIP_CPU_SYNC);
	err = dma_mapping_error(attach->dev, addr);
	if (err)
		goto err_table;
	sg_set_page(sgt->sgl, NULL, obj->size, 0);
	sg_dma_address(sgt->sgl) = addr;
	sg_dma_len(sgt->sgl) = obj->size;
	return sgt;

err_table:
	sg_free_table(sgt);
err_free:
	kfree(sgt);
	return ERR_PTR(err);
}

static void cdram_unmap_dma_buf(struct dma_buf_attachment *attach,
				struct sg_table *sgt,
				enum dma_data_direction dir)
{
	dma_unmap_resource(attach->dev, sg_dma_address(sgt->sgl),
			   sg_dma_len(sgt->sgl), dir, DMA_ATTR_SKIP_CPU_SYNC);
	sg_free_table(sgt);
	kfree(sgt);
}

static const struct dma_buf_ops cdram_dmabuf_ops = {
	.attach = drm_gem_map_attach,
	.detach = drm_gem_map_detach,
	.map_dma_buf = cdram_map_dma_buf,
	.unmap_dma_buf = cdram_unmap_dma_buf,
	.release = drm_gem_dmabuf_release,
	.mmap = drm_gem_dmabuf_mmap,
};

static struct dma_buf *cdram_export(struct drm_gem_object *obj, int flags)
{
	struct dma_buf_export_info exp_info = {
		.exp_name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.ops = &cdram_dmabuf_ops,
		.size = obj->size,
		.flags = flags,
		.priv = obj,
		.resv = obj->resv,
	};

	return drm_gem_dmabuf_export(obj->dev, &exp_info);
}

bool psp2sgx_cdram_is_dmabuf(struct drm_device *drm, struct dma_buf *dma_buf)
{
	struct drm_gem_object *obj = dma_buf->priv;

	return dma_buf->ops == &cdram_dmabuf_ops && obj->dev == drm;
}

static const struct vm_operations_struct cdram_vm_ops = {
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

const struct drm_gem_object_funcs psp2sgx_cdram_funcs = {
	.free = cdram_free,
	.open = psp2sgx_gem_open,
	.close = psp2sgx_gem_close,
	.print_info = cdram_print_info,
	.export = cdram_export,
	.mmap = cdram_mmap,
	.vm_ops = &cdram_vm_ops,
};

/*
 * A zeroed CDRAM object of size bytes (rounded up to pages), or
 * ERR_PTR(-ENOSPC) when the region has no room or there is none.
 */
struct drm_gem_object *psp2sgx_cdram_create(struct psp2sgx_device *sgx,
					    size_t size)
{
	struct psp2sgx_cdram *c = &sgx->cdram;
	struct psp2sgx_cdram_bo *cbo;
	struct drm_gem_object *obj;
	int err;

	if (!c->size)
		return ERR_PTR(-ENOSPC);
	size = PAGE_ALIGN(size);
	cbo = kzalloc_obj(*cbo);
	if (!cbo)
		return ERR_PTR(-ENOMEM);

	mutex_lock(&c->lock);
	err = drm_mm_insert_node_generic(&c->mm, &cbo->node, size, PAGE_SIZE,
					 0, DRM_MM_INSERT_BEST);
	if (!err) {
		c->used += size;
		c->peak = max(c->peak, c->used);
		c->objects++;
	} else {
		c->fallbacks++;
	}
	mutex_unlock(&c->lock);
	if (err) {
		kfree(cbo);
		dev_info_ratelimited(sgx->dev,
				     "CDRAM full: %zu KiB in system memory\n",
				     size >> 10);
		return ERR_PTR(-ENOSPC);
	}

	obj = &cbo->base;
	obj->funcs = &psp2sgx_cdram_funcs;
	drm_gem_private_object_init(&sgx->drm, obj, size);
	psp2sgx_bo_init(&cbo->bo, obj);
	err = cdram_clear(cbo->node.start, size);
	if (err) {
		drm_gem_object_put(obj);
		return ERR_PTR(err);
	}
	return obj;
}

/* ---------------------------------------------------------------- debugfs */

static int memory_show(struct seq_file *m, void *data)
{
	struct drm_debugfs_entry *entry = m->private;
	struct psp2sgx_device *sgx = to_psp2sgx(entry->dev);
	struct psp2sgx_cdram *c = &sgx->cdram;
	u64 hole_start, hole_end, largest = 0;
	struct drm_mm_node *hole;

	seq_printf(m, "system: %llu KiB in %d objects\n",
		   (unsigned long long)atomic64_read(&sgx->shmem_bytes) >> 10,
		   atomic_read(&sgx->shmem_objects));
	if (!c->size) {
		seq_puts(m, "cdram: none\n");
		return 0;
	}
	mutex_lock(&c->lock);
	drm_mm_for_each_hole(hole, &c->mm, hole_start, hole_end)
		largest = max(largest, hole_end - hole_start);
	seq_printf(m, "cdram: %llu KiB in %u objects, %llu KiB free (largest %llu KiB), peak %llu KiB, of %llu KiB at %pa\n",
		   c->used >> 10, c->objects,
		   ((unsigned long long)c->size - c->used) >> 10, largest >> 10,
		   c->peak >> 10, (unsigned long long)c->size >> 10, &c->base);
	seq_printf(m, "cdram requests placed in system memory: %llu\n",
		   c->fallbacks);
	mutex_unlock(&c->lock);
	return 0;
}

static int cdram_mm_show(struct seq_file *m, void *data)
{
	struct drm_debugfs_entry *entry = m->private;
	struct psp2sgx_cdram *c = &to_psp2sgx(entry->dev)->cdram;
	struct drm_printer p = drm_seq_file_printer(m);

	if (!c->size)
		return 0;
	mutex_lock(&c->lock);
	drm_mm_print(&c->mm, &p);
	mutex_unlock(&c->lock);
	return 0;
}

static const struct drm_debugfs_info cdram_debugfs[] = {
	{ "memory", memory_show },
	{ "cdram_mm", cdram_mm_show },
};

/* before drm_dev_register() */
void psp2sgx_cdram_debugfs_init(struct psp2sgx_device *sgx)
{
	drm_debugfs_add_files(&sgx->drm, cdram_debugfs, ARRAY_SIZE(cdram_debugfs));
}
