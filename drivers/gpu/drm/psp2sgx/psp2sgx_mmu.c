// SPDX-License-Identifier: GPL-2.0-only
/*
 * psp2sgx: GPU address spaces.
 *
 * The SGX MMU (the BIF) walks a two-level table: a 1024-entry page directory
 * of 4 MiB entries and 1024-entry page tables of 4 KiB pages, each entry the
 * physical address in bits 31..12 plus flags (sgxmmu.h; a PDE's bits 3..1
 * select the page size, 0 for 4 KiB).  The probe ran exactly this format.
 *
 * Every open file gets an address space of its own.  All of them map the
 * kernel region (microkernel, event program, host control block, job code)
 * at SGX_KREGION_VA through one shared page table.  The hardware tables live
 * in write-combined memory, so the GPU sees the CPU's writes once they leave
 * the write buffer; the BIF's TLBs are dropped before every job
 * (psp2sgx_hw_set_vm).
 */
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>

#include "psp2sgx_drv.h"

static void *table_alloc(struct psp2sgx_device *sgx, dma_addr_t *dma)
{
	void *t = dma_alloc_wc(sgx->dev, SGX_MMU_PAGE_SIZE, dma, GFP_KERNEL);

	if (t)
		memset(t, 0, SGX_MMU_PAGE_SIZE);
	return t;
}

static void table_free(struct psp2sgx_device *sgx, void *t, dma_addr_t dma)
{
	dma_free_wc(sgx->dev, SGX_MMU_PAGE_SIZE, t, dma);
}

int psp2sgx_kregion_init(struct psp2sgx_device *sgx)
{
	unsigned int n;

	sgx->kregion_size = PAGE_ALIGN(SGX_MK_REGION_USED);
	sgx->kregion = dma_alloc_wc(sgx->dev, sgx->kregion_size,
				    &sgx->kregion_dma, GFP_KERNEL);
	if (!sgx->kregion)
		return -ENOMEM;
	memset(sgx->kregion, 0, sgx->kregion_size);
	sgx->kregion_pt = table_alloc(sgx, &sgx->kregion_pt_dma);
	if (!sgx->kregion_pt)
		goto err_region;
	/*
	 * Read-write for the GPU: the microkernel writes the host control
	 * block.  The code and program pages could be READONLY, but what the
	 * bit does on this BIF is not known yet.
	 */
	for (n = 0; n < sgx->kregion_size >> SGX_MMU_PAGE_SHIFT; n++)
		sgx->kregion_pt[(SGX_KREGION_VA >> SGX_MMU_PAGE_SHIFT) % SGX_MMU_ENTRIES + n] =
			(sgx->kregion_dma + (n << SGX_MMU_PAGE_SHIFT)) | SGX_MMU_PTE_VALID;

	sgx->kernel_vm = psp2sgx_vm_create(sgx);
	if (IS_ERR(sgx->kernel_vm)) {
		int err = PTR_ERR(sgx->kernel_vm);

		sgx->kernel_vm = NULL;
		table_free(sgx, sgx->kregion_pt, sgx->kregion_pt_dma);
		dma_free_wc(sgx->dev, sgx->kregion_size, sgx->kregion, sgx->kregion_dma);
		return err;
	}
	wmb();
	return 0;

err_region:
	dma_free_wc(sgx->dev, sgx->kregion_size, sgx->kregion, sgx->kregion_dma);
	return -ENOMEM;
}

void psp2sgx_kregion_fini(struct psp2sgx_device *sgx)
{
	if (sgx->kernel_vm)
		psp2sgx_vm_put(sgx->kernel_vm);
	table_free(sgx, sgx->kregion_pt, sgx->kregion_pt_dma);
	dma_free_wc(sgx->dev, sgx->kregion_size, sgx->kregion, sgx->kregion_dma);
}

struct psp2sgx_vm *psp2sgx_vm_create(struct psp2sgx_device *sgx)
{
	struct psp2sgx_vm *vm = kzalloc(sizeof(*vm), GFP_KERNEL);

	if (!vm)
		return ERR_PTR(-ENOMEM);
	vm->sgx = sgx;
	kref_init(&vm->ref);
	mutex_init(&vm->lock);
	vm->pd = table_alloc(sgx, &vm->pd_dma);
	if (!vm->pd) {
		kfree(vm);
		return ERR_PTR(-ENOMEM);
	}
	vm->pd[SGX_KREGION_VA >> SGX_MMU_PDE_SHIFT] =
		sgx->kregion_pt_dma | SGX_MMU_PDE_VALID;
	drm_mm_init(&vm->mm, PSP2SGX_GENERAL_BASE,
		    PSP2SGX_CODE_END - PSP2SGX_GENERAL_BASE);
	wmb();
	return vm;
}

static void psp2sgx_vm_release(struct kref *ref)
{
	struct psp2sgx_vm *vm = container_of(ref, struct psp2sgx_vm, ref);
	struct psp2sgx_device *sgx = vm->sgx;
	unsigned int n;

	drm_mm_takedown(&vm->mm);
	for (n = 0; n < SGX_MMU_ENTRIES; n++)
		if (vm->pt[n])
			table_free(sgx, vm->pt[n], vm->pt_dma[n]);
	table_free(sgx, vm->pd, vm->pd_dma);
	mutex_destroy(&vm->lock);
	kfree(vm);
}

void psp2sgx_vm_put(struct psp2sgx_vm *vm)
{
	if (vm)
		kref_put(&vm->ref, psp2sgx_vm_release);
}

static u32 *vm_pte(struct psp2sgx_vm *vm, u32 va, bool alloc)
{
	u32 pde = va >> SGX_MMU_PDE_SHIFT;

	if (pde == SGX_KREGION_VA >> SGX_MMU_PDE_SHIFT)
		return NULL;		/* never through a user mapping */
	if (!vm->pt[pde]) {
		if (!alloc)
			return NULL;
		vm->pt[pde] = table_alloc(vm->sgx, &vm->pt_dma[pde]);
		if (!vm->pt[pde])
			return NULL;
		vm->pd[pde] = vm->pt_dma[pde] | SGX_MMU_PDE_VALID;
	}
	return &vm->pt[pde][(va >> SGX_MMU_PAGE_SHIFT) % SGX_MMU_ENTRIES];
}

/* Maps the first size bytes of sgt at va; the caller holds vm->lock. */
int psp2sgx_vm_map_sgt(struct psp2sgx_vm *vm, u32 va, struct sg_table *sgt,
		       size_t size, u32 flags)
{
	struct sg_dma_page_iter it;
	u32 at = va, end = va + size;

	for_each_sgtable_dma_page(sgt, &it, 0) {
		dma_addr_t addr = sg_page_iter_dma_address(&it);
		u32 *pte;

		if (at >= end)
			break;
		pte = vm_pte(vm, at, true);
		if (!pte) {
			psp2sgx_vm_unmap(vm, va, at - va);
			return -ENOMEM;
		}
		*pte = (u32)addr | SGX_MMU_PTE_VALID | flags;
		at += SGX_MMU_PAGE_SIZE;
	}
	wmb();
	return at >= end ? 0 : -EINVAL;
}

void psp2sgx_vm_unmap(struct psp2sgx_vm *vm, u32 va, size_t size)
{
	u32 at;

	for (at = va; at < va + size; at += SGX_MMU_PAGE_SIZE) {
		u32 *pte = vm_pte(vm, at, false);

		if (pte)
			*pte = 0;
	}
	wmb();
}

u32 psp2sgx_vm_pte_read(struct psp2sgx_vm *vm, u32 va)
{
	u32 *pte;

	mutex_lock(&vm->lock);
	pte = vm_pte(vm, va, false);
	mutex_unlock(&vm->lock);
	return pte ? READ_ONCE(*pte) : 0;
}
