// SPDX-License-Identifier: GPL-2.0-only
/*
 * psp2sgx: hardware checks for the DevKit.
 *
 * In the first DevKit runs every job timed out with the host control block
 * still reading "pending, 0 runs", while a bare-metal probe had run USSE
 * programs on the same GPU.
 * These checks tell apart a microkernel that never starts from one that
 * starts and hangs:
 *
 *  - at probe, the microkernel is kicked once with no job posted; it only
 *    counts the run and exits, so the run counter must go up;
 *  - that result and the first job timeouts dump the registers the probe
 *    read on the DevKit: every event status view, core 0's USSE task,
 *    memory write and event program counters, the BIF fault registers, and
 *    the code base, event program and kicker settings as the GPU holds them.
 *
 * The host control block is read after an SLC flush: the microkernel's
 * stores sit in the system-level cache until then.  Everything is logged
 * as a warning: Android lowers the console level to 4 during boot.  The
 * self-test result
 * and a dump are printed again 90 s after probe, for a console attached
 * after the kernel started (the DevKit's mini-USB port must be unplugged
 * across the loader's standby).
 */
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/workqueue.h>

#include "psp2sgx_drv.h"
#include "psp2sgx_mk.h"
#include "psp2sgx_regdump.h"

/* core registers the bare-metal probe read */
#define SGX_USE0_SERV_EVENT		0x0b10	/* USSE tasks started */
#define SGX_PDS_EVDM_PC_BASE		0x0b2c	/* last event program's code address >> 2 */
#define SGX_EMU_MEM_WRITE		0x0ebc	/* memory writes issued */
#define SGX_MASTER_BIF_INT_STAT		0x4c04
#define SGX_MASTER_BIF_FAULT		0x4c08

#define SELFTEST_WAIT_US		100000

/* psp2sgx.regdump_at_probe=1: the full register dump after the self-test too */
static bool regdump_at_probe;
module_param(regdump_at_probe, bool, 0444);

void psp2sgx_diag_dump(struct psp2sgx_device *sgx, const char *why)
{
	u32 *hc = psp2sgx_hostctl(sgx);
	u32 c0 = SGX_CORE_BANK(0);
	/* the GPU may still be running: write back, drop nothing */
	int flush = psp2sgx_hw_slc_writeback(sgx);
	unsigned int n;

	rmb();
	dev_warn(sgx->dev, "diag %s: kregion dma %pad, page directory %pad, SLC flush %s\n",
		 why, &sgx->kregion_dma, &sgx->kernel_vm->pd_dma, flush ? "TIMED OUT" : "ok");
	dev_warn(sgx->dev, "diag %s: posted %08x, hostctl %08x %08x %08x %08x %08x | %08x %08x %08x %08x %08x %08x\n",
		 why, psp2sgx_hostpost(sgx)[SGX_HP_POSTED],
		 hc[SGX_HC_STATE], hc[SGX_HC_CLAIMED], hc[SGX_HC_DONE_SEQNO],
		 hc[SGX_HC_ERROR], hc[SGX_HC_RUNS], hc[SGX_HC_DIAG_TA], hc[SGX_HC_DIAG_TA + 1],
		 hc[SGX_HC_DIAG_TA + 2], hc[SGX_HC_DIAG_EVENT], hc[SGX_HC_DIAG_EVENT + 1],
		 hc[SGX_HC_DIAG_EVENT + 2]);
	dev_warn(sgx->dev, "diag %s: EVENT_STATUS bank0 %08x master %08x cores %08x %08x %08x %08x; STATUS2 bank0 %08x core0 %08x\n",
		 why, sgx_read(sgx, SGX_EVENT_STATUS), sgx_read(sgx, SGX_MASTER_EVENT_STATUS),
		 sgx_read(sgx, SGX_CORE_BANK(0) + SGX_EVENT_STATUS),
		 sgx_read(sgx, SGX_CORE_BANK(1) + SGX_EVENT_STATUS),
		 sgx_read(sgx, SGX_CORE_BANK(2) + SGX_EVENT_STATUS),
		 sgx_read(sgx, SGX_CORE_BANK(3) + SGX_EVENT_STATUS),
		 sgx_read(sgx, SGX_EVENT_STATUS2), sgx_read(sgx, c0 + SGX_EVENT_STATUS2));
	for (n = 0; n < 4; n++)
		dev_warn(sgx->dev, "diag %s: core %u USSE tasks %08x memory writes %08x event program %08x (%08x)\n",
			 why, n, sgx_read(sgx, SGX_CORE_BANK(n) + SGX_USE0_SERV_EVENT),
			 sgx_read(sgx, SGX_CORE_BANK(n) + SGX_EMU_MEM_WRITE),
			 sgx_read(sgx, SGX_CORE_BANK(n) + SGX_PDS_EVDM_PC_BASE),
			 sgx_read(sgx, SGX_CORE_BANK(n) + SGX_PDS_EVDM_PC_BASE) << 2);
	dev_warn(sgx->dev, "diag %s: BIF int/fault cores %08x/%08x %08x/%08x %08x/%08x %08x/%08x master %08x/%08x; SLC status %08x\n",
		 why, sgx_read(sgx, c0 + SGX_BIF_INT_STAT), sgx_read(sgx, c0 + SGX_BIF_FAULT),
		 sgx_read(sgx, SGX_CORE_BANK(1) + SGX_BIF_INT_STAT),
		 sgx_read(sgx, SGX_CORE_BANK(1) + SGX_BIF_FAULT),
		 sgx_read(sgx, SGX_CORE_BANK(2) + SGX_BIF_INT_STAT),
		 sgx_read(sgx, SGX_CORE_BANK(2) + SGX_BIF_FAULT),
		 sgx_read(sgx, SGX_CORE_BANK(3) + SGX_BIF_INT_STAT),
		 sgx_read(sgx, SGX_CORE_BANK(3) + SGX_BIF_FAULT),
		 sgx_read(sgx, SGX_MASTER_BIF_INT_STAT), sgx_read(sgx, SGX_MASTER_BIF_FAULT),
		 sgx_read(sgx, SGX_MASTER_SLC_EVENT_STATUS));
	for (n = 0; n < PSP2SGX_NUM_CORES; n++) {
		u32 fault = sgx_read(sgx, SGX_CORE_BANK(n) + SGX_BIF_FAULT) & 0xfffff000;

		u32 start = 0, size = 0;

		if (!(sgx_read(sgx, SGX_CORE_BANK(n) + SGX_BIF_INT_STAT) & 0x3fff) || !sgx->current_vm)
			continue;
		if (!psp2sgx_vm_node_at(sgx->current_vm, fault, &start, &size))
			start = size = 0;
		dev_warn(sgx->dev, "diag %s: core %u fault at %08x: PTE %08x, mapping %08x + %x in the current address space\n",
			 why, n, fault, psp2sgx_vm_pte_read(sgx->current_vm, fault), start, size);
	}
	dev_warn(sgx->dev, "diag %s: code base 15 bank0 %08x core0 %08x; event program %08x/%08x info %08x, PDS enable %08x; kicker %08x; directories %08x %08x\n",
		 why, sgx_read(sgx, SGX_USE_CODE_BASE(SGX_MK_CODE_BASE)),
		 sgx_read(sgx, c0 + SGX_USE_CODE_BASE(SGX_MK_CODE_BASE)),
		 sgx_read(sgx, c0 + SGX_EVENT_OTHER_PDS_EXEC),
		 sgx_read(sgx, c0 + SGX_EVENT_OTHER_PDS_DATA),
		 sgx_read(sgx, c0 + SGX_EVENT_OTHER_PDS_INFO),
		 sgx_read(sgx, c0 + SGX_EVENT_PDS_ENABLE), sgx_read(sgx, SGX_EVENT_KICKER),
		 sgx_read(sgx, SGX_BIF_DIR_LIST_BASE(0)), sgx_read(sgx, SGX_BIF_DIR_LIST_BASE(7)));
}

/*
 * Kick the microkernel with nothing posted: it takes its lock, counts the
 * run and exits without an interrupt.  Returns 0 when the count went up.
 */
int psp2sgx_diag_selftest(struct psp2sgx_device *sgx)
{
	u32 *hc = psp2sgx_hostctl(sgx);
	u32 before, runs = 0;
	unsigned int us;

	mutex_lock(&sgx->hw_lock);
	/* nothing is posted at probe (both blocks are zero); the run only counts */
	psp2sgx_hw_slc_flush(sgx);
	psp2sgx_hw_invalidate_caches(sgx);
	rmb();
	before = READ_ONCE(hc[SGX_HC_RUNS]);
	psp2sgx_hw_kick(sgx);
	for (us = 0; us < SELFTEST_WAIT_US; us += 100) {
		udelay(100);
		psp2sgx_hw_slc_flush(sgx);
		rmb();
		runs = READ_ONCE(hc[SGX_HC_RUNS]);
		if (runs != before)
			break;
	}
	mutex_unlock(&sgx->hw_lock);

	if (runs != before)
		snprintf(sgx->selftest, sizeof(sgx->selftest),
			 "ran (runs %u -> %u) within %u us", before, runs, us + 100);
	else
		snprintf(sgx->selftest, sizeof(sgx->selftest),
			 "DID NOT RUN within %u us (runs %u)", SELFTEST_WAIT_US, runs);
	dev_warn(sgx->dev, "microkernel self-test: %s\n", sgx->selftest);
	psp2sgx_diag_dump(sgx, "after self-test");
	if (regdump_at_probe) {
		psp2sgx_diag_regdump(sgx, "after self-test");
		sgx->regs_dumped = false;	/* the first job error still gets one */
	}
	schedule_delayed_work(&sgx->diag_work, 90 * HZ);
	return runs != before ? 0 : -EIO;
}

static void psp2sgx_diag_later(struct work_struct *work)
{
	struct psp2sgx_device *sgx = container_of(to_delayed_work(work),
						  struct psp2sgx_device, diag_work);

	dev_warn(sgx->dev, "microkernel self-test at probe: %s\n", sgx->selftest);
	mutex_lock(&sgx->hw_lock);
	psp2sgx_diag_dump(sgx, "at 90 s");
	mutex_unlock(&sgx->hw_lock);
}

void psp2sgx_diag_init(struct psp2sgx_device *sgx)
{
	INIT_DELAYED_WORK(&sgx->diag_work, psp2sgx_diag_later);
	strscpy(sgx->selftest, "not run", sizeof(sgx->selftest));
}

void psp2sgx_diag_fini(struct psp2sgx_device *sgx)
{
	cancel_delayed_work_sync(&sgx->diag_work);
}

static void regdump_block(struct psp2sgx_device *sgx, const char *block, u32 base,
			  const u16 *regs, unsigned int n)
{
	char line[8 * 14 + 1];
	unsigned int i, len = 0;

	for (i = 0; i < n; i++) {
		len += scnprintf(line + len, sizeof(line) - len, " %04x=%08x",
				 regs[i], sgx_read(sgx, base + regs[i]));
		if (i % 8 == 7 || i == n - 1) {
			dev_warn(sgx->dev, "regs %s:%s\n", block, line);
			len = 0;
		}
	}
}

/*
 * Every register of SceGpuEs4's own debug dump, once per boot, on the first
 * job the microkernel gives up on: offset=value, 8 per line, the master
 * block then each core's bank (offsets within the bank).  Only the cores in
 * use: with two enabled, reading core 2's bank past 0x900 ended in an
 * imprecise external abort and a panic on the DevKit.
 */
void psp2sgx_diag_regdump(struct psp2sgx_device *sgx, const char *why)
{
	static const char *const core_name[] = { "core0", "core1", "core2", "core3" };
	unsigned int n;

	if (sgx->regs_dumped)
		return;
	sgx->regs_dumped = true;
	dev_warn(sgx->dev, "register dump (%s): SceGpuEs4's SGXDumpDebugState list\n", why);
	regdump_block(sgx, "master", 0, psp2sgx_dump_master, ARRAY_SIZE(psp2sgx_dump_master));
	for (n = 0; n < psp2sgx_hw_cores(); n++)
		regdump_block(sgx, core_name[n], SGX_CORE_BANK(n), psp2sgx_dump_core,
			      ARRAY_SIZE(psp2sgx_dump_core));
	dev_warn(sgx->dev, "register dump end\n");
}
