// SPDX-License-Identifier: GPL-2.0-only
/*
 * psp2sgx: jobs.
 *
 * A job is a list of register operations (or, for CAP_SYS_ADMIN, one raw
 * PDS task).  At submit the kernel checks every operation and assembles the
 * job into microkernel code (psp2sgx_mk.c); the scheduler runs one job at a
 * time: it copies the code to the kernel region, points the MMU at the
 * job's address space, posts the job in the host control block and kicks
 * core 0.  The microkernel's SW_EVENT interrupt completes it.
 */
#include <linux/dma-fence.h>
#include <linux/file.h>
#include <linux/sync_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <drm/drm_exec.h>
#include <drm/drm_file.h>
#include <drm/drm_gem.h>
#include <drm/drm_syncobj.h>
#include <drm/psp2sgx_drm.h>

#include "psp2sgx_drv.h"
#include "psp2sgx_gem.h"
#include "psp2sgx_mk.h"

struct psp2sgx_job {
	struct drm_sched_job base;
	struct psp2sgx_device *sgx;
	struct psp2sgx_vm *vm;
	struct drm_gem_object **bos;
	u32 *bo_va;			/* where each of bos sits in vm */
	u32 nr_bos;
	u64 *code;
	u32 code_len;
	u32 seqno;
	struct dma_fence *done;		/* the hardware fence */
};

static inline struct psp2sgx_job *to_job(struct drm_sched_job *job)
{
	return container_of(job, struct psp2sgx_job, base);
}

/* ------------------------------------------------------------------ fences */

static const char *psp2sgx_fence_driver_name(struct dma_fence *fence)
{
	return "psp2sgx";
}

static const char *psp2sgx_fence_timeline_name(struct dma_fence *fence)
{
	return "psp2sgx-mk";
}

static const struct dma_fence_ops psp2sgx_fence_ops = {
	.get_driver_name = psp2sgx_fence_driver_name,
	.get_timeline_name = psp2sgx_fence_timeline_name,
};

/* ------------------------------------------------------------------ checks */

struct reg_range {
	u32 first, last;
};

/*
 * What a render may write: the vertex data master, tiling engine, MTE, ISP,
 * DPM, TPU and pixel event registers the DDK's microkernel programs for a
 * TA/3D kick, the BIF requestor bases (addresses the MMU still translates),
 * and the event clears for what a render waits on.  Not the BIF, code bases,
 * event enables or kicks: those belong to the kernel.  Addresses are in the
 * broadcast bank unless noted; per-core copies are in core_write_ok.
 */
static const struct reg_range write_ok[] = {
	{ 0x0114, 0x0114 },	/* EVENT_HOST_CLEAR2, bits checked below */
	{ 0x0134, 0x0134 },	/* EVENT_HOST_CLEAR, bits checked below */
	{ 0x0140, 0x0140 },	/* PDS_CACHE_HOST_CLEAR */
	{ 0x0148, 0x0148 },	/* DOUBLE_PIXEL_PARTITIONS */
	{ 0x0200, 0x025c },	/* VDM, TE, MTE */
	{ 0x0380, 0x038c },	/* TE_RGNHDR_INIT */
	{ 0x0400, 0x052c },	/* ISP */
	{ 0x0600, 0x06a0 },	/* DPM */
	{ 0x0750, 0x0750 },	/* DPM_MTILE_ABORTED, cleared before each TA kick */
	{ 0x081c, 0x0848 },	/* TPU_YUV */
	{ 0x0a5c, 0x0a64 },	/* EVENT_PIXEL_PDS_EXEC/DATA/INFO */
	{ 0x0ae8, 0x0ae8 },	/* PDS_PP_INDEPENDANT_STATE */
	{ 0x0c90, 0x0c90 },	/* BIF_TA_REQ_BASE */
	{ 0x0cac, 0x0cb0 },	/* BIF_3D_REQ_BASE, BIF_ZLS_REQ_BASE */
	{ 0x4134, 0x4134 },	/* MASTER_EVENT_HOST_CLEAR, bits checked below */
	{ 0x4320, 0x4320 },	/* MASTER_VDM_BATCH_NUM */
	{ 0x435c, 0x435c },	/* MASTER_VDM_PIM */
	{ 0x4368, 0x4368 },	/* MASTER_DPM_TASK_DPLIST */
	{ 0x4370, 0x4370 },	/* MASTER_DPM_DPLIST_CLEAR_EVENT */
	{ 0x4510, 0x451c },	/* MASTER_ISP_RGN_BASE1..3, MASTER_ISP_RGN */
	{ 0x4800, 0x4800 },	/* MASTER_DPM_PROACTIVE_PIM_SPEC */
	{ 0x480c, 0x480c },	/* MASTER_DPM_DRAIN_HEAP */
};

/*
 * Per-core bank offsets (SGX_CORE_BANK(n) + off): what the DDK programs per
 * core for a kick, the region and TPC bases and the TE state.
 */
static const u32 core_write_ok[] = {
	0x021c,			/* TE_PSGREGION_BASE */
	0x0220,			/* TE_TPC_BASE */
	0x0274,			/* TE_STATE */
	0x04c0,			/* ISP_VISREGBASE */
	0x050c,			/* ISP_ZLS_EXTZ_RGN_BASE */
	0x0ae8,			/* PDS_PP_INDEPENDANT_STATE */
};

/* What a render may wait on: status registers only. */
static const struct reg_range wait_ok[] = {
	{ 0x0118, 0x0118 },	/* EVENT_STATUS2 */
	{ 0x012c, 0x012c },	/* EVENT_STATUS */
	{ 0x0138, 0x0138 },	/* PDS_CACHE_STATUS */
	{ 0x0600, 0x0730 },	/* DPM state and status */
	{ 0x412c, 0x412c },	/* MASTER_EVENT_STATUS */
	{ 0x4360, 0x4360 },	/* MASTER_VDM_PIM_STATUS */
	{ 0x436c, 0x436c },	/* MASTER_DPM_DPLIST_EVENT_STATUS */
};

static bool core_reg_ok(u32 reg)
{
	unsigned int i;
	u32 off;

	if (reg < SGX_CORE_BANK(0) || reg >= SGX_CORE_BANK(PSP2SGX_NUM_CORES) ||
	    reg & 3)
		return false;
	off = (reg - SGX_CORE_BANK(0)) & 0x3fff;
	for (i = 0; i < ARRAY_SIZE(core_write_ok); i++)
		if (off == core_write_ok[i])
			return true;
	return false;
}

static bool reg_in(const struct reg_range *r, unsigned int n, u32 reg)
{
	unsigned int i;

	if (reg & 3)
		return false;
	for (i = 0; i < n; i++)
		if (reg >= r[i].first && reg <= r[i].last)
			return true;
	return false;
}

static int check_write(u32 reg, u32 value)
{
	if (!reg_in(write_ok, ARRAY_SIZE(write_ok), reg) && !core_reg_ok(reg))
		return -EPERM;
	/* the host owns the interrupts it has enabled */
	if ((reg == SGX_EVENT_HOST_CLEAR || reg == SGX_MASTER_EVENT_HOST_CLEAR) &&
	    value & SGX_EVENT_SW_EVENT)
		return -EPERM;
	if (reg == SGX_EVENT_HOST_CLEAR2 && value & 0xc00)
		return -EPERM;
	return 0;
}

/*
 * A bank 0 host clear does not reach the core banks: on the DevKit every
 * core still held OTPM_INV and DPM_CONTROL_CLEAR (0x04100200) after the
 * render cleared them in bank 0, and core 0 is one of the views a wait
 * accepts (psp2sgx_mk.c), so a stale bit there would end the next wait early.
 */
static void clear_cores(struct sgx_mk_job *j, u32 reg, u32 value)
{
	unsigned int n;

	for (n = 0; n < PSP2SGX_NUM_CORES; n++)
		sgx_mk_job_write(j, SGX_CORE_BANK(n) + reg, value);
}

/*
 * TA_FINISHED in bank 0 and the master does not mean every core is done
 * with the TA.  On the DevKit (2026-09-24) job 3's TA wait passed on the
 * master's bit while core 0 did not have it yet; cores 0 and 1 set it only
 * after the wait, and cores 2 and 3 never did: their TA, TE and VDM clocks
 * were still running when the 3D pass started over them, and they hung
 * there with every later job.  So a TA starts with the bit cleared in every
 * view, whatever the previous job left, and its end also waits for each
 * core's tiling units to go idle.  That holds for a core that got no batch
 * too, which a wait for each core's own TA_FINISHED would not.
 */
static void clear_ta_finished(struct sgx_mk_job *j)
{
	sgx_mk_job_write(j, SGX_EVENT_HOST_CLEAR, SGX_EVENT_TA_FINISHED);
	sgx_mk_job_write(j, SGX_MASTER_EVENT_HOST_CLEAR, SGX_EVENT_TA_FINISHED);
	clear_cores(j, SGX_EVENT_HOST_CLEAR, SGX_EVENT_TA_FINISHED);
}

static void wait_ta_idle(struct sgx_mk_job *j)
{
	unsigned int n;

	for (n = 0; n < psp2sgx_hw_cores(); n++)
		sgx_mk_job_wait(j, SGX_CORE_BANK(n) + SGX_CLKGATESTATUS,
				SGX_CLKGATESTATUS_TA_UNITS, 0, PSP2SGX_WAIT_MAX_POLLS);
}

/*
 * The same for the 3D pass: PIXELBE_END_RENDER is latched in core 0's bank
 * and the master only, and it came while cores 2 and 3 were still shading
 * (on the DevKit: before job 3 their ISP2, TSP, PDS and PBE clocks ran,
 * CLKGATESTATUS 0x01008987, their pixel PDS stuck at the start of a
 * program; cores 0 and 1 idle).  The job then ended over them: the USSE
 * clocks back to automatic gating, the host's SLC flush and invalidate, the
 * next job's directory switch, and they never finished.  So a render ends
 * when every core's pixel units are idle.
 */
static void wait_3d_idle(struct sgx_mk_job *j)
{
	unsigned int n;

	for (n = 0; n < psp2sgx_hw_cores(); n++)
		sgx_mk_job_wait(j, SGX_CORE_BANK(n) + SGX_CLKGATESTATUS,
				SGX_CLKGATESTATUS_3D_UNITS, 0, PSP2SGX_WAIT_MAX_POLLS);
}

/*
 * What Sony's microkernel does between a TA and the 3D pass that a render's
 * operations cannot (a: shell register trace, every kick): once the TA is
 * done, the VDM batch number loaded and set back to 0 in the master and
 * bank 0 (on the DevKit cores 2 and 3 read VDM_BATCH_NUM_STATUS 0 and
 * never got a batch); after the tail pointer cache flush, an ISP2 soft reset
 * pulse in bank 0 before the 3D set-up.
 */
static void ta_done(struct sgx_mk_job *j)
{
	sgx_mk_job_write(j, SGX_MASTER_VDM_BATCH_NUM, 1);
	sgx_mk_job_write(j, SGX_MASTER_VDM_BATCH_NUM, 0);
	sgx_mk_job_write(j, SGX_VDM_BATCH_NUM, 1);
	sgx_mk_job_write(j, SGX_VDM_BATCH_NUM, 0);
}

static void tpc_flushed(struct sgx_mk_job *j)
{
	sgx_mk_job_write(j, SGX_SOFT_RESET, SGX_SOFT_RESET_ISP2);
	sgx_mk_job_write(j, SGX_SOFT_RESET, 0);
}

static int build_render(struct sgx_mk_job *j,
			const struct drm_psp2sgx_reg_op *ops, u32 n)
{
	u32 i;

	for (i = 0; i < n; i++) {
		const struct drm_psp2sgx_reg_op *op = &ops[i];

		switch (op->op) {
		case PSP2SGX_REG_WRITE:
			if (check_write(op->reg, op->value))
				return -EPERM;
			if (op->reg == SGX_VDM_START)
				clear_ta_finished(j);
			sgx_mk_job_write(j, op->reg, op->value);
			if (op->reg == SGX_EVENT_HOST_CLEAR || op->reg == SGX_EVENT_HOST_CLEAR2)
				clear_cores(j, op->reg, op->value);
			break;
		case PSP2SGX_REG_WAIT:
			if (!reg_in(wait_ok, ARRAY_SIZE(wait_ok), op->reg))
				return -EPERM;
			sgx_mk_job_wait(j, op->reg, op->mask, op->value & op->mask,
					PSP2SGX_WAIT_MAX_POLLS);
			if ((op->reg == SGX_EVENT_STATUS || op->reg == SGX_MASTER_EVENT_STATUS) &&
			    op->mask & op->value & SGX_EVENT_TA_FINISHED) {
				wait_ta_idle(j);
				ta_done(j);
			}
			if (op->reg == SGX_EVENT_STATUS &&
			    op->mask & op->value & SGX_EVENT_TPC_FLUSH)
				tpc_flushed(j);
			if ((op->reg == SGX_EVENT_STATUS || op->reg == SGX_MASTER_EVENT_STATUS) &&
			    op->mask & op->value & SGX_EVENT_PIXELBE_END_RENDER)
				wait_3d_idle(j);
			break;
		default:
			return -EINVAL;
		}
		if (j->a.err)
			return j->a.err;
	}
	return 0;
}

static int build_task(struct sgx_mk_job *j, const struct drm_psp2sgx_task *t)
{
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	if (!t->data_units || t->data_units > 31 || t->attr_size >> 15 ||
	    t->sideband >> 14 || t->pds_addr & 0xf || t->pad)
		return -EINVAL;
	/* the task word as Sony's microkernel builds it (0x800900: 1 unit, 9) */
	sgx_mk_job_task(j, t->data_units << 23 | t->attr_size << 8, t->pds_addr,
			t->ir0, t->sideband);
	return j->a.err;
}

static int build_code(struct psp2sgx_job *job, const struct drm_psp2sgx_submit *args)
{
	void __user *cmds = u64_to_user_ptr(args->cmds);
	struct sgx_mk_job j;
	void *buf = NULL;
	int err;

	job->code = kvmalloc_array(SGX_MK_JOB_CODE_MAX, sizeof(u64), GFP_KERNEL);
	if (!job->code)
		return -ENOMEM;
	sgx_mk_job_begin(&j, job->code, SGX_MK_JOB_CODE_MAX);
	/*
	 * What Sony's microkernel writes before every TA and 3D kick (a
	 * register trace of the shell): the USSE clocks forced on, and the
	 * data master partitions, 3 vertex and 8 pixel.  The init value
	 * 0x208200 allows none: on the DevKit the vertex tasks of a TA were
	 * never served (USEn_SERV_VERTEX 0), core 0 took one VDM batch, cores 1
	 * to 3 none, and TA_FINISHED never came (register dump, 2026-09-24).
	 * The clocks go back to automatic gating when the job ends, as Sony's
	 * do; a job that gives up early leaves them on.
	 */
	sgx_mk_job_write(&j, SGX_CLKGATECTL2, SGX_CLKGATECTL2_USE_ON);
	sgx_mk_job_write(&j, SGX_DMS_CTRL, SGX_DMS_CTRL_JOB);

	switch (args->type) {
	case PSP2SGX_JOB_RENDER:
		if (!args->nr_cmds || args->nr_cmds > PSP2SGX_MAX_REG_OPS)
			return -EINVAL;
		buf = memdup_array_user(cmds, args->nr_cmds,
					sizeof(struct drm_psp2sgx_reg_op));
		if (IS_ERR(buf))
			return PTR_ERR(buf);
		err = build_render(&j, buf, args->nr_cmds);
		break;
	case PSP2SGX_JOB_TASK:
		if (args->nr_cmds != 1)
			return -EINVAL;
		buf = memdup_user(cmds, sizeof(struct drm_psp2sgx_task));
		if (IS_ERR(buf))
			return PTR_ERR(buf);
		err = build_task(&j, buf);
		break;
	default:
		return -EINVAL;
	}
	kfree(buf);
	if (!err) {
		sgx_mk_job_write(&j, SGX_CLKGATECTL2, SGX_CLKGATECTL2_AUTO);
		err = sgx_mk_job_end(&j);
	}
	job->code_len = j.a.len;
	return err;
}

/* ------------------------------------------------------------------ scheduler */

static struct dma_fence *psp2sgx_run_job(struct drm_sched_job *sched_job)
{
	struct psp2sgx_job *job = to_job(sched_job);
	struct psp2sgx_device *sgx = job->sgx;
	u32 *hc = psp2sgx_hostctl(sgx);
	struct dma_fence *fence;
	unsigned long flags;
	unsigned int i;

	if (unlikely(sched_job->s_fence->finished.error))
		return NULL;

	fence = kzalloc(sizeof(*fence), GFP_KERNEL);
	if (!fence)
		return ERR_PTR(-ENOMEM);

	mutex_lock(&sgx->hw_lock);
	/*
	 * A job the microkernel gave up on leaves the GPU in its state (a TA
	 * that never ends keeps going); without a reset every later job
	 * failed as well (on the DevKit).  Sony restarts the GPU the same
	 * way (SGXStart) after a lockup.
	 */
	if (sgx->needs_reset) {
		sgx->needs_reset = false;
		sgx->resets++;
		dev_warn(sgx->dev, "GPU reset %u after a failed job, before job %llu\n",
			 sgx->resets, sgx->emit_seqno + 1);
		if (psp2sgx_hw_reset(sgx))
			dev_err(sgx->dev, "GPU reset failed\n");
	}
	spin_lock_irqsave(&sgx->fence_lock, flags);
	dma_fence_init(fence, &psp2sgx_fence_ops, &sgx->fence_lock,
		       sgx->fence_context, ++sgx->emit_seqno);
	job->seqno = lower_32_bits(sgx->emit_seqno);
	dma_fence_put(job->done);
	job->done = dma_fence_get(fence);
	sgx->current_job = job;
	spin_unlock_irqrestore(&sgx->fence_lock, flags);

	/*
	 * Empty the SLC before touching the host control block.  The
	 * microkernel runs on every event, also after the job's completion
	 * interrupt, and its run count store leaves the block's line dirty in
	 * the SLC with the old state.  A flush after the writes below wrote
	 * that line back over them: on the DevKit job 2 read "idle, job 1"
	 * and timed out (2026-09-24).
	 */
	if (psp2sgx_hw_slc_flush(sgx))
		dev_warn_ratelimited(sgx->dev, "SLC flush timed out\n");
	psp2sgx_put_code((u8 *)sgx->kregion + SGX_MK_JOB_CODE, job->code,
			 job->code_len);
	WRITE_ONCE(hc[SGX_HC_JOB_SEQNO], job->seqno);
	WRITE_ONCE(hc[SGX_HC_ERROR], 0);
	for (i = SGX_HC_DIAG_TA; i < SGX_HC_DIAG_EVENT + 3; i++)
		WRITE_ONCE(hc[i], 0);
	wmb();
	WRITE_ONCE(hc[SGX_HC_STATE], SGX_MK_PENDING);
	psp2sgx_hw_wait_cores_idle(sgx, job->seqno);
	psp2sgx_hw_set_vm(sgx, job->vm);
	/* the GPU must see the code, the job and the tables, not stale lines */
	if (psp2sgx_hw_slc_flush(sgx) || psp2sgx_hw_invalidate_caches(sgx))
		dev_warn_ratelimited(sgx->dev, "cache maintenance timed out\n");
	psp2sgx_hw_kick(sgx);
	mutex_unlock(&sgx->hw_lock);
	return fence;
}

/* The SW_EVENT interrupt, from the threaded handler. */
/*
 * Which of the job's buffers holds each core's MMU fault address.  On the
 * DevKit cores 2 and 3 faulted at 0x01010000 while its PTE was valid.
 */
static void report_fault_bos(struct psp2sgx_device *sgx, struct psp2sgx_job *job)
{
	unsigned int n;
	u32 i;

	for (n = 0; n < PSP2SGX_NUM_CORES; n++) {
		u32 stat = sgx_read(sgx, SGX_CORE_BANK(n) + SGX_BIF_INT_STAT);
		u32 addr = sgx_read(sgx, SGX_CORE_BANK(n) + SGX_BIF_FAULT) & 0xfffff000;
		bool found = false;

		if (!(stat & 0x3fff))
			continue;
		for (i = 0; i < job->nr_bos && job->bos[i]; i++) {
			size_t size = job->bos[i]->size;

			if (addr >= job->bo_va[i] && addr - job->bo_va[i] < size) {
				dev_warn(sgx->dev, "job %u: core %u fault %08x is in its buffer %u of %u (%08x, %zu bytes)\n",
					 job->seqno, n, addr, i, job->nr_bos, job->bo_va[i], size);
				found = true;
			}
		}
		if (found)
			continue;
		dev_warn(sgx->dev, "job %u: core %u fault %08x is in none of its %u buffers:\n",
			 job->seqno, n, addr, job->nr_bos);
		for (i = 0; i < job->nr_bos && job->bos[i] && i < 48; i++)
			dev_warn(sgx->dev, "job %u:   buffer %u at %08x, %zu bytes\n",
				 job->seqno, i, job->bo_va[i], job->bos[i]->size);
	}
}

void psp2sgx_job_irq(struct psp2sgx_device *sgx)
{
	u32 *hc = psp2sgx_hostctl(sgx);
	struct psp2sgx_job *job;
	unsigned long flags;
	u32 done, error;
	bool dump = false;

	/* the microkernel's stores sit in the SLC until flushed */
	if (psp2sgx_hw_slc_flush(sgx))
		dev_warn_ratelimited(sgx->dev, "SLC flush timed out\n");
	rmb();
	done = READ_ONCE(hc[SGX_HC_DONE_SEQNO]);
	error = READ_ONCE(hc[SGX_HC_ERROR]);

	spin_lock_irqsave(&sgx->fence_lock, flags);
	job = sgx->current_job;
	if (job && (s32)(done - job->seqno) >= 0) {
		if (error) {
			dev_warn_ratelimited(sgx->dev, "job %u: microkernel error %#x\n",
					     job->seqno, error);
			dma_fence_set_error(job->done, -EIO);
			dump = !sgx->regs_dumped;
			if (dump)
				report_fault_bos(sgx, job);
			sgx->needs_reset = true;
		}
		/* which banks latch the TA and 3D events is open on the hardware */
		if (error || (!sgx->events_reported &&
			      (hc[SGX_HC_DIAG_TA + 1] | hc[SGX_HC_DIAG_EVENT + 1] |
			       hc[SGX_HC_DIAG_TA] | hc[SGX_HC_DIAG_EVENT]))) {
			sgx->events_reported = true;
			dev_info_ratelimited(sgx->dev,
				"job %u EVENT_STATUS bank0/master/core0: TA wait %#x/%#x/%#x, other %#x/%#x/%#x\n",
				job->seqno, hc[SGX_HC_DIAG_TA], hc[SGX_HC_DIAG_TA + 1],
				hc[SGX_HC_DIAG_TA + 2], hc[SGX_HC_DIAG_EVENT],
				hc[SGX_HC_DIAG_EVENT + 1], hc[SGX_HC_DIAG_EVENT + 2]);
		}
		sgx->current_job = NULL;
		dma_fence_signal_locked(job->done);
	}
	spin_unlock_irqrestore(&sgx->fence_lock, flags);

	/* the GPU as the failed wait left it (a TA that never ends keeps going) */
	if (dump) {
		mutex_lock(&sgx->hw_lock);
		psp2sgx_diag_dump(sgx, "microkernel error");
		psp2sgx_diag_regdump(sgx, "microkernel error");
		mutex_unlock(&sgx->hw_lock);
	}
}

static enum drm_gpu_sched_stat psp2sgx_timedout_job(struct drm_sched_job *sched_job)
{
	struct psp2sgx_job *job = to_job(sched_job);
	struct psp2sgx_device *sgx = job->sgx;
	u32 *hc = psp2sgx_hostctl(sgx);
	unsigned long flags;

	synchronize_irq(sgx->irq);
	if (job->done && dma_fence_is_signaled(job->done))
		return DRM_GPU_SCHED_STAT_NO_HANG;

	dev_err(sgx->dev, "job %u timed out: state %u done %u error %#x runs %u, EVENT_STATUS %#x\n",
		job->seqno, READ_ONCE(hc[SGX_HC_STATE]), READ_ONCE(hc[SGX_HC_DONE_SEQNO]),
		READ_ONCE(hc[SGX_HC_ERROR]), READ_ONCE(hc[SGX_HC_RUNS]),
		sgx_read(sgx, SGX_EVENT_STATUS));
	if (sgx->timeouts_dumped < 2) {
		sgx->timeouts_dumped++;
		mutex_lock(&sgx->hw_lock);
		psp2sgx_diag_dump(sgx, "job timeout");
		mutex_unlock(&sgx->hw_lock);
	}

	drm_sched_stop(&sgx->sched, sched_job);
	drm_sched_increase_karma(sched_job);

	spin_lock_irqsave(&sgx->fence_lock, flags);
	sgx->current_job = NULL;
	spin_unlock_irqrestore(&sgx->fence_lock, flags);

	mutex_lock(&sgx->hw_lock);
	psp2sgx_hw_reset(sgx);
	mutex_unlock(&sgx->hw_lock);

	drm_sched_resubmit_jobs(&sgx->sched);
	drm_sched_start(&sgx->sched, 0);
	return DRM_GPU_SCHED_STAT_RESET;
}

static void psp2sgx_job_put_bos(struct psp2sgx_job *job)
{
	u32 i;

	for (i = 0; i < job->nr_bos; i++) {
		if (!job->bos[i])
			break;
		psp2sgx_bo_map_put(to_psp2sgx_bo(job->bos[i]), job->vm);
		drm_gem_object_put(job->bos[i]);
	}
	kvfree(job->bos);
	kvfree(job->bo_va);
}

static void psp2sgx_job_free(struct psp2sgx_job *job)
{
	psp2sgx_job_put_bos(job);
	dma_fence_put(job->done);
	kvfree(job->code);
	psp2sgx_vm_put(job->vm);
	kfree(job);
}

static void psp2sgx_free_job(struct drm_sched_job *sched_job)
{
	struct psp2sgx_job *job = to_job(sched_job);

	drm_sched_job_cleanup(sched_job);
	psp2sgx_job_free(job);
}

static const struct drm_sched_backend_ops psp2sgx_sched_ops = {
	.run_job = psp2sgx_run_job,
	.timedout_job = psp2sgx_timedout_job,
	.free_job = psp2sgx_free_job,
};

int psp2sgx_sched_init(struct psp2sgx_device *sgx)
{
	const struct drm_sched_init_args args = {
		.ops = &psp2sgx_sched_ops,
		.num_rqs = DRM_SCHED_PRIORITY_COUNT,
		.credit_limit = 1,	/* one job on the microkernel at a time */
		.hang_limit = 0,
		.timeout = msecs_to_jiffies(PSP2SGX_JOB_TIMEOUT_MS),
		.name = "psp2sgx",
		.dev = sgx->dev,
	};

	sgx->fence_context = dma_fence_context_alloc(1);
	spin_lock_init(&sgx->fence_lock);
	return drm_sched_init(&sgx->sched, &args);
}

void psp2sgx_sched_fini(struct psp2sgx_device *sgx)
{
	drm_sched_fini(&sgx->sched);
}

/* ------------------------------------------------------------------ submit */

static int lookup_bos(struct drm_file *file, struct psp2sgx_job *job,
		      const struct drm_psp2sgx_submit *args,
		      struct drm_psp2sgx_submit_bo **out)
{
	struct drm_psp2sgx_submit_bo *sbos;
	u32 i;
	int err;

	if (!args->nr_bos)
		return 0;
	if (args->nr_bos > 4096)
		return -EINVAL;
	sbos = memdup_array_user(u64_to_user_ptr(args->bos), args->nr_bos,
				 sizeof(*sbos));
	if (IS_ERR(sbos))
		return PTR_ERR(sbos);
	job->bos = kvcalloc(args->nr_bos, sizeof(*job->bos), GFP_KERNEL);
	job->bo_va = kvcalloc(args->nr_bos, sizeof(*job->bo_va), GFP_KERNEL);
	if (!job->bos || !job->bo_va) {
		kfree(sbos);
		return -ENOMEM;
	}
	job->nr_bos = args->nr_bos;
	for (i = 0; i < args->nr_bos; i++) {
		struct drm_gem_object *obj;

		if (sbos[i].flags & ~PSP2SGX_SUBMIT_BO_WRITE) {
			err = -EINVAL;
			goto err;
		}
		obj = drm_gem_object_lookup(file, sbos[i].handle);
		if (!obj) {
			err = -ENOENT;
			goto err;
		}
		/* keeps the mapping while the job runs, even if the handle goes */
		err = psp2sgx_bo_map_get(to_psp2sgx_bo(obj), job->vm, &job->bo_va[i]);
		if (err) {
			drm_gem_object_put(obj);
			goto err;
		}
		job->bos[i] = obj;
	}
	*out = sbos;
	return 0;
err:
	kfree(sbos);
	return err;
}

int psp2sgx_ioctl_submit(struct drm_device *drm, void *data,
			 struct drm_file *file)
{
	struct psp2sgx_device *sgx = to_psp2sgx(drm);
	struct psp2sgx_file *pf = file->driver_priv;
	struct drm_psp2sgx_submit *args = data;
	struct drm_psp2sgx_submit_bo *sbos = NULL;
	struct drm_syncobj *out_sync = NULL;
	struct sync_file *sync_file = NULL;
	struct dma_fence *fence;
	struct psp2sgx_job *job;
	struct drm_exec exec;
	int out_fd = -1;
	u32 i;
	int err;

	if (args->pad ||
	    args->flags & ~(PSP2SGX_SUBMIT_FENCE_FD_IN | PSP2SGX_SUBMIT_FENCE_FD_OUT))
		return -EINVAL;

	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (!job)
		return -ENOMEM;
	job->sgx = sgx;
	job->vm = psp2sgx_vm_get(pf->vm);

	err = build_code(job, args);
	if (err)
		goto err_free;
	err = lookup_bos(file, job, args, &sbos);
	if (err)
		goto err_free;

	if (args->out_sync) {
		out_sync = drm_syncobj_find(file, args->out_sync);
		if (!out_sync) {
			err = -ENOENT;
			goto err_free;
		}
	}
	if (args->flags & PSP2SGX_SUBMIT_FENCE_FD_OUT) {
		out_fd = get_unused_fd_flags(O_CLOEXEC);
		if (out_fd < 0) {
			err = out_fd;
			goto err_free;
		}
	}

	err = drm_sched_job_init(&job->base, &pf->entity, 1, NULL, file->client_id);
	if (err)
		goto err_free;

	if (args->in_sync) {
		err = drm_sched_job_add_syncobj_dependency(&job->base, file,
							   args->in_sync, 0);
		if (err)
			goto err_job;
	}
	if (args->flags & PSP2SGX_SUBMIT_FENCE_FD_IN) {
		struct dma_fence *in = sync_file_get_fence(args->fence_fd);

		if (!in) {
			err = -EINVAL;
			goto err_job;
		}
		err = drm_sched_job_add_dependency(&job->base, in);
		if (err)
			goto err_job;
	}

	drm_exec_init(&exec, DRM_EXEC_INTERRUPTIBLE_WAIT, job->nr_bos);
	drm_exec_until_all_locked(&exec) {
		err = drm_exec_prepare_array(&exec, job->bos, job->nr_bos, 1);
		drm_exec_retry_on_contention(&exec);
		if (err)
			goto err_exec;
	}
	for (i = 0; i < job->nr_bos; i++) {
		err = drm_sched_job_add_implicit_dependencies(&job->base, job->bos[i],
							      sbos[i].flags & PSP2SGX_SUBMIT_BO_WRITE);
		if (err)
			goto err_exec;
	}

	drm_sched_job_arm(&job->base);
	fence = dma_fence_get(&job->base.s_fence->finished);
	drm_sched_entity_push_job(&job->base);

	for (i = 0; i < job->nr_bos; i++)
		dma_resv_add_fence(job->bos[i]->resv, fence,
				   sbos[i].flags & PSP2SGX_SUBMIT_BO_WRITE ?
				   DMA_RESV_USAGE_WRITE : DMA_RESV_USAGE_READ);
	drm_exec_fini(&exec);
	kfree(sbos);

	if (out_sync) {
		drm_syncobj_replace_fence(out_sync, fence);
		drm_syncobj_put(out_sync);
	}
	if (out_fd >= 0) {
		sync_file = sync_file_create(fence);
		if (sync_file) {
			fd_install(out_fd, sync_file->file);
			args->fence_fd = out_fd;
		} else {
			put_unused_fd(out_fd);
			args->fence_fd = -1;
		}
	}
	dma_fence_put(fence);
	return 0;

err_exec:
	drm_exec_fini(&exec);
err_job:
	drm_sched_job_cleanup(&job->base);
err_free:
	if (out_fd >= 0)
		put_unused_fd(out_fd);
	if (out_sync)
		drm_syncobj_put(out_sync);
	kfree(sbos);
	psp2sgx_job_free(job);
	return err;
}
