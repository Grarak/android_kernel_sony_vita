// SPDX-License-Identifier: GPL-2.0-only
/*
 * psp2sgx: power, start-up, initialisation, caches and the kick.
 *
 * The sequences are those a bare-metal SGX probe ran on the DevKit, which
 * in turn replay what SceGpuEs4 and gpuinit_es4 write on a genuine resume
 * (an MMIO trace).  Anything the probe did not run is marked as such.
 */
#include <linux/delay.h>
#include <linux/iopoll.h>

#include "psp2sgx_drv.h"
#include "psp2sgx_mk.h"
#include "psp2sgx_pds.h"
#include "psp2sgx_usse.h"

/*
 * The GPU power domain belongs to the secure world: SceGpuEs4 switches it
 * with SMC 0x110 (bus 0 = GPU, 1 = start, 0 = stop), and the monitor then
 * programs the Pervasive reset and gate registers.  Whether the monitor is
 * still resident depends on how Linux was started, so the device tree says
 * whether to call it (sony,power-smc).
 */
static u32 psp2sgx_smc_bus(u32 service, u32 bus, u32 command)
{
	register u32 r0 asm("r0") = bus;
	register u32 r1 asm("r1") = command;
	register u32 r12 asm("r12") = service;

	asm volatile(".arch_extension sec\n\tsmc #0"
		     : "+r"(r0) : "r"(r1), "r"(r12) : "r2", "r3", "memory");
	return r0;
}

/*
 * SceGpuEs4's SGXStart after the power-up: master clocks and SLC set-up,
 * master soft reset with the BIF set up during it, core clock gating.  The
 * genuine resume writes exactly these before its first SGX read; any read
 * before them hangs the bus.  Its delays are 100 SGX clocks.
 */
static void psp2sgx_start(struct psp2sgx_device *sgx)
{
	sgx_write(sgx, SGX_MASTER_CORE, 3);
	sgx_write(sgx, SGX_MASTER_CLKGATECTL2, 0x2a9);
	sgx_write(sgx, SGX_MASTER_CLKGATECTL, 0xaa);
	sgx_write(sgx, SGX_MASTER_SOFT_RESET, 0x7ff);
	wmb();
	udelay(10);
	sgx_write(sgx, SGX_MASTER_BIF_CTRL, 0);
	sgx_write(sgx, 0x4d00, 0x4c800);
	sgx_write(sgx, 0x4d04, 0x4000000);
	wmb();
	udelay(10);
	sgx_write(sgx, SGX_MASTER_SOFT_RESET, 0);
	wmb();
	udelay(10);
	sgx_write(sgx, SGX_CLKGATECTL, 0x2aaaaa);
	sgx_write(sgx, SGX_CLKGATECTL2, 0xa8a8aaa);
	wmb();
	udelay(10);
}

int psp2sgx_hw_power_on(struct psp2sgx_device *sgx)
{
	if (sgx->power_smc >= 0) {
		u32 ret = psp2sgx_smc_bus(sgx->power_smc, sgx->power_smc_bus, 1);

		if (ret)
			dev_warn(sgx->dev, "power SMC returned %#x\n", ret);
		/* not measured; the probe waited far longer for other reasons */
		usleep_range(1000, 2000);
	}
	psp2sgx_start(sgx);
	sgx->core_id = sgx_read(sgx, SGX_CORE_ID);
	sgx->core_revision = sgx_read(sgx, SGX_CORE_REVISION);
	return 0;
}

void psp2sgx_hw_power_off(struct psp2sgx_device *sgx)
{
	sgx_write(sgx, SGX_EVENT_HOST_ENABLE, 0);
	sgx_write(sgx, SGX_EVENT_HOST_ENABLE2, 0);
	if (sgx->power_smc >= 0)
		psp2sgx_smc_bus(sgx->power_smc, sgx->power_smc_bus, 0);
}

static int poll_bits(struct psp2sgx_device *sgx, u32 reg, u32 bits, u32 *seen)
{
	u32 v;
	int err;

	err = readl_poll_timeout_atomic(sgx->regs + reg, v, (v & bits) == bits,
					1, 1000);
	if (seen)
		*seen = v;
	return err;
}

/*
 * Write the system-level cache back and drop its lines, as the microkernel
 * does before the host reads what the GPU wrote (program A 0x1071).  USSE
 * stores are invisible to the CPU without it, and a program's last store is
 * no fence: the host flushes after it sees the completion interrupt.
 */
int psp2sgx_hw_slc_flush(struct psp2sgx_device *sgx)
{
	int err;

	sgx_write(sgx, SGX_MASTER_SLC_CTRL_FLUSH, SGX_SLC_FLUSH_ALL);
	err = poll_bits(sgx, SGX_MASTER_SLC_EVENT_STATUS, SGX_SLC_EVENT_FLUSH, NULL);
	sgx_write(sgx, SGX_MASTER_SLC_EVENT_CLEAR, SGX_SLC_EVENT_FLUSH);
	if (err)
		return err;
	sgx_write(sgx, SGX_MASTER_SLC_CTRL_INVAL, SGX_SLC_INVAL_ALL);
	err = poll_bits(sgx, SGX_MASTER_SLC_EVENT_STATUS, SGX_SLC_EVENT_INVAL, NULL);
	sgx_write(sgx, SGX_MASTER_SLC_EVENT_CLEAR, SGX_SLC_EVENT_INVAL);
	return err;
}

/*
 * Drop the GPU's own caches of code and data the CPU has rewritten: the
 * USSE instruction cache, the texture and data caches and the PDS code and
 * data caches.  Sony's microkernel issues these writes itself (A 0x104f..
 * 0x105c, inside lock #7) and polls the PDS cache status; here the host does
 * it while the GPU is idle.  UNVERIFIED on the hardware from the CPU side;
 * the completion bits are those Sony's polls test.
 */
int psp2sgx_hw_invalidate_caches(struct psp2sgx_device *sgx)
{
	int err = 0;

	unsigned int n;

	sgx_write(sgx, SGX_DCU_ICTRL, 1);
	sgx_write(sgx, SGX_TCU_ICTRL, 1);
	sgx_write(sgx, SGX_USE_CACHE, 1);
	/* the other cores' data, texture and USSE code caches as well */
	for (n = 0; n < PSP2SGX_NUM_CORES; n++) {
		sgx_write(sgx, SGX_CORE_BANK(n) + SGX_DCU_ICTRL, 1);
		sgx_write(sgx, SGX_CORE_BANK(n) + SGX_TCU_ICTRL, 1);
		sgx_write(sgx, SGX_CORE_BANK(n) + SGX_USE_CACHE, 1);
	}
	sgx_write(sgx, SGX_PDS_INV0, 1);
	sgx_write(sgx, SGX_PDS_INV_CSC, 1);
	if (poll_bits(sgx, SGX_PDS_CACHE_STATUS,
		      SGX_PDS_CACHE_CSC_INV | SGX_PDS_CACHE_DSC_INV0, NULL))
		err = -ETIMEDOUT;
	sgx_write(sgx, SGX_PDS_CACHE_HOST_CLEAR,
		  SGX_PDS_CACHE_CSC_INV | SGX_PDS_CACHE_DSC_INV0);
	return err;
}

/* Point every requestor at vm's page directory and drop the BIF's TLBs. */
void psp2sgx_hw_set_vm(struct psp2sgx_device *sgx, struct psp2sgx_vm *vm)
{
	unsigned int n;

	if (sgx->current_vm != vm) {
		psp2sgx_vm_put(sgx->current_vm);
		sgx->current_vm = psp2sgx_vm_get(vm);
	}
	sgx_write(sgx, SGX_BIF_DIR_LIST_BASE(0), vm->pd_dma);
	sgx_write(sgx, SGX_BIF_DIR_LIST_BASE(7), vm->pd_dma);
	/*
	 * as the microkernel invalidates (A 0x0c89: str [0xc34] = #8), from 0:
	 * the register reads back 8 afterwards, so rewriting 8 may not start
	 * another invalidate.  On the DevKit cores 2 and 3 faulted at a page
	 * whose PTE was valid (BIF_FAULT 0x01010000, PTE 0x7a677001) after
	 * the directory and 8 had been written to every bank.
	 */
	sgx_write(sgx, SGX_BIF_CTRL_INVAL, 0);
	sgx_write(sgx, SGX_BIF_CTRL_INVAL, SGX_BIF_CTRL_INVAL_ALL);
	/*
	 * Sony keeps one GPU address space, this driver switches per process.
	 * On the DevKit the first job of a second process faulted on cores 2
	 * and 3 only (BIF_FAULT 0x01010000, invalid page) while cores 0 and 1
	 * ran it: the bank 0 write may not reach the other cores' TLBs.  Each
	 * core and the master get the directory and the invalidate too.
	 */
	sgx_write(sgx, SGX_MASTER_BIF_CTRL + SGX_BIF_DIR_LIST_BASE(0) - 0x0c00, vm->pd_dma);
	sgx_write(sgx, SGX_MASTER_BIF_CTRL + SGX_BIF_DIR_LIST_BASE(7) - 0x0c00, vm->pd_dma);
	sgx_write(sgx, SGX_MASTER_BIF_CTRL + SGX_BIF_CTRL_INVAL - 0x0c00, 0);
	sgx_write(sgx, SGX_MASTER_BIF_CTRL + SGX_BIF_CTRL_INVAL - 0x0c00, SGX_BIF_CTRL_INVAL_ALL);
	for (n = 0; n < PSP2SGX_NUM_CORES; n++) {
		u32 bank = SGX_CORE_BANK(n);

		sgx_write(sgx, bank + SGX_BIF_DIR_LIST_BASE(0), vm->pd_dma);
		sgx_write(sgx, bank + SGX_BIF_DIR_LIST_BASE(7), vm->pd_dma);
		sgx_write(sgx, bank + SGX_BIF_CTRL_INVAL, 0);
		sgx_write(sgx, bank + SGX_BIF_CTRL_INVAL, SGX_BIF_CTRL_INVAL_ALL);
	}
	wmb();
}

/* Core 0's event kick: runs the event PDS program, which starts the microkernel. */
void psp2sgx_hw_kick(struct psp2sgx_device *sgx)
{
	wmb();
	sgx_write(sgx, SGX_CORE_BANK(0) + SGX_EVENT_KICK2, 1);
}

static u32 code_base(u32 va, u32 dm)
{
	return (va >> 6) | dm << SGX_USE_CODE_BASE_DM_SHIFT;
}

/*
 * The genuine init script's writes after the start sequence, in its order
 * (an MMIO trace of a resume), with our addresses.  The values
 * without a comment are the traced ones, which the DevKit ran.
 */
int psp2sgx_hw_init(struct psp2sgx_device *sgx)
{
	static const u32 zeroed[] = {
		SGX_BIF_CTRL, SGX_BIF_BANK_SET, SGX_BIF_BANK0, SGX_BIF_DIR_LIST_BASE(0),
		0x0c38, 0x0c3c, 0x0c40, 0x0c44, 0x0c48, 0x0c4c, 0x0c50,
	};
	const u32 kva = SGX_KREGION_VA;
	unsigned int n;

	for (n = 0; n < ARRAY_SIZE(zeroed); n++)
		sgx_write(sgx, zeroed[n], 0);
	/* every data master's requests go through directory 7 or 0; both hold the same */
	sgx_write(sgx, SGX_BIF_BANK0, 7);
	psp2sgx_hw_set_vm(sgx, sgx->kernel_vm);
	sgx_write(sgx, 0x001c, 0);
	sgx_write(sgx, SGX_EVENT_HOST_ENABLE, SGX_EVENT_SW_EVENT);
	sgx_write(sgx, SGX_EVENT_HOST_ENABLE2, 0xc00);
	sgx_write(sgx, SGX_PDS_CACHE_HOST_ENABLE, 0);
	sgx_write(sgx, 0x0a7c, 0);
	sgx_write(sgx, 0x0a80, 0);
	sgx_write(sgx, SGX_DMS_CTRL, 0x208200);
	sgx_write(sgx, 0x0abc, 0x40);
	sgx_write(sgx, 0x0ab8, 0);
	sgx_write(sgx, SGX_TSP_CONFIG, 0x10);
	sgx_write(sgx, SGX_DCU_CTRL, 0);
	sgx_write(sgx, SGX_TCU_CTRL, 0x5e0);
	sgx_write(sgx, SGX_DCU_PCTRL, 0xffff);
	sgx_write(sgx, 0x0a00, 0x7c000);
	/*
	 * Code bases: 0 and 1 are the vertex and fragment windows of every
	 * address space (data master 0 and 1 in bits 26..27, not run on the
	 * DevKit); 15 is the microkernel, in the init script's own form with
	 * the event data master (the probe's run G4).
	 */
	for (n = 0; n < 15; n++)
		sgx_write(sgx, SGX_USE_CODE_BASE(n), code_base(kva + SGX_MK_CODE, 0));
	sgx_write(sgx, SGX_USE_CODE_BASE(0), code_base(PSP2SGX_CODE_VERTEX_BASE, 0));
	sgx_write(sgx, SGX_USE_CODE_BASE(1), code_base(PSP2SGX_CODE_FRAGMENT_BASE, 1));
	sgx_write(sgx, SGX_USE_CODE_BASE(SGX_MK_CODE_BASE), code_base(kva + SGX_MK_CODE, 3));
	sgx_write(sgx, 0x0b30, 0x100);
	sgx_write(sgx, SGX_EVENT_KICKER, kva + SGX_MK_KICKER);
	/* bank 0's event program is never kicked; it gets ours so it is valid */
	sgx_write(sgx, SGX_EVENT_OTHER_PDS_EXEC, kva + SGX_MK_EVENT_PDS);
	sgx_write(sgx, SGX_EVENT_OTHER_PDS_DATA, SGX_PDS_EVENT_UNITS);
	sgx_write(sgx, SGX_EVENT_OTHER_PDS_INFO, 0x12001);
	sgx_write(sgx, SGX_CORE_BANK(0) + SGX_EVENT_OTHER_PDS_EXEC, kva + SGX_MK_EVENT_PDS);
	sgx_write(sgx, SGX_CORE_BANK(0) + SGX_EVENT_OTHER_PDS_DATA, SGX_PDS_EVENT_UNITS);
	sgx_write(sgx, SGX_CORE_BANK(0) + SGX_EVENT_OTHER_PDS_INFO, 0x12001);
	sgx_write(sgx, SGX_CORE_BANK(0) + SGX_EVENT_PDS_ENABLE, 0x2200c000);
	sgx_write(sgx, 0x4a58, 0x804300f);
	sgx_write(sgx, 0x4808, 4);
	sgx_write(sgx, 0x4144, 0x103e8);
	sgx_write(sgx, 0x414c, 0);
	sgx_write(sgx, 0x0bb4, 0);
	/* start from a clean host interrupt state */
	sgx_write(sgx, SGX_EVENT_HOST_CLEAR, SGX_EVENT_SW_EVENT);
	wmb();
	return 0;
}

/*
 * The kernel region's fixed contents: the event PDS program, the kicker
 * word and the microkernel.  Job code is written per job (psp2sgx_job.c).
 */
int psp2sgx_hw_build_kregion(struct psp2sgx_device *sgx)
{
	u8 *k = sgx->kregion;
	u32 ev[SGX_PDS_EVENT_WORDS];
	u64 *code;
	u32 len;
	int err;

	code = kcalloc(SGX_MK_CODE_MAX, sizeof(*code), GFP_KERNEL);
	if (!code)
		return -ENOMEM;
	err = sgx_mk_build(code, SGX_MK_CODE_MAX, &len, SGX_KREGION_VA);
	if (err) {
		dev_err(sgx->dev, "microkernel does not assemble: %d\n", err);
		goto out;
	}
	memset(k, 0, SGX_MK_REGION_USED);
	psp2sgx_put_code(k + SGX_MK_CODE, code, len);
	sgx_pds_event_program(ev, SGX_KREGION_VA + SGX_MK_HOSTCTL,
			      sgx_pds_doutu0(SGX_MK_CODE_BASE, 0, SGX_MK_TEMP_UNITS));
	memcpy(k + SGX_MK_EVENT_PDS, ev, sizeof(ev));
	/* the probe's kicker word */
	*(u32 *)(k + SGX_MK_KICKER) = 0x5a;
	wmb();
	dev_dbg(sgx->dev, "microkernel: %u instructions\n", len);
out:
	kfree(code);
	return err;
}

/* After a hang: restart the GPU and program it again. */
int psp2sgx_hw_reset(struct psp2sgx_device *sgx)
{
	u32 *hc = psp2sgx_hostctl(sgx);

	psp2sgx_start(sgx);
	WRITE_ONCE(hc[SGX_HC_STATE], SGX_MK_IDLE);
	wmb();
	return psp2sgx_hw_init(sgx);
}
