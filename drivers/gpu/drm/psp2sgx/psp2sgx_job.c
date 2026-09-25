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
 *
 * With host_waits (the default) a render's waits do not run in the
 * microkernel.  The job is cut at each wait into steps: code, which one
 * microkernel run executes, and waits, which the host polls from the CPU
 * between runs.  A microkernel that polls for a whole render keeps a thread
 * on core 0's USSE the render itself needs: on the DevKit a 3D wait gave
 * up after 275 ms with PIXELBE_END_RENDER in
 * cores 1 to 3 only, and core 0 had it by the time the host looked, right
 * after the microkernel had stopped polling.  Sony's microkernel never
 * waits for a render either: it returns and runs again on the next event.
 */
#include <linux/delay.h>
#include <linux/dma-fence.h>
#include <linux/file.h>
#include <linux/ktime.h>
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

module_param_named(wait_yield, sgx_mk_wait_yield, uint, 0444);
MODULE_PARM_DESC(wait_yield,
		 "register reads per poll round of a job's waits, 0-16 (default 0)");

static bool host_waits = true;
module_param(host_waits, bool, 0444);
MODULE_PARM_DESC(host_waits,
		 "The host polls a render's waits between microkernel runs (default: true); 0: the microkernel polls them");

/* Room for all of a job's code: host_waits cuts it into runs */
#define JOB_CODE_TOTAL		(2 * SGX_MK_JOB_CODE_MAX)

enum { STEP_CODE, STEP_WAIT };
enum { WAIT_TA, WAIT_3D, WAIT_OTHER };

struct psp2sgx_step {
	u8 kind;
	u8 what;			/* STEP_WAIT: WAIT_*, for the statistics */
	u32 reg, mask, value;		/* STEP_WAIT: until (reg & mask) == value */
	u32 start, len;			/* STEP_CODE: instructions in job->code */
};

struct psp2sgx_job {
	struct drm_sched_job base;
	struct psp2sgx_device *sgx;
	struct psp2sgx_vm *vm;
	struct drm_gem_object **bos;
	u32 *bo_va;			/* where each of bos sits in vm */
	u32 nr_bos;
	u64 *code;
	u32 code_len;
	struct psp2sgx_step *steps;
	u32 nr_steps, max_steps;
	u32 step;			/* the next step to run (hw_lock) */
	u32 kick;			/* kick_seqno of its last microkernel run */
	u32 rescues;			/* timeouts that found its run done and went on */
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
 * A job under construction: microkernel code and, with host_waits, the
 * waits between its runs.  Without host_waits the whole job is one run and
 * its waits are microkernel polls.
 */
struct job_build {
	struct psp2sgx_job *job;
	struct sgx_mk_job j;		/* the open run */
	bool open;
	bool host;			/* host_waits */
	u32 used;			/* instructions of job->code taken */
	int err;
};

static struct psp2sgx_step *add_step(struct job_build *b)
{
	struct psp2sgx_job *job = b->job;

	if (job->nr_steps == job->max_steps) {
		b->err = -E2BIG;
		return NULL;
	}
	return memset(&job->steps[job->nr_steps++], 0, sizeof(*job->steps));
}

/* The open run, begun if there is none */
static struct sgx_mk_job *run(struct job_build *b)
{
	if (!b->open) {
		sgx_mk_job_begin(&b->j, b->job->code + b->used,
				 min_t(u32, JOB_CODE_TOTAL - b->used, SGX_MK_JOB_CODE_MAX));
		b->open = true;
	}
	return &b->j;
}

static void end_run(struct job_build *b)
{
	struct psp2sgx_step *s;
	int err;

	if (!b->open)
		return;
	b->open = false;
	err = sgx_mk_job_end(&b->j);
	if (err) {
		b->err = b->err ?: err;
		return;
	}
	s = add_step(b);
	if (!s)
		return;
	s->kind = STEP_CODE;
	s->start = b->used;
	s->len = b->j.a.len;
	b->used += b->j.a.len;
}

static void write_reg(struct job_build *b, u32 reg, u32 value)
{
	sgx_mk_job_write(run(b), reg, value);
}

static void wait_reg(struct job_build *b, u32 reg, u32 mask, u32 value, u8 what)
{
	struct psp2sgx_step *s;

	if (!b->host) {
		sgx_mk_job_wait(run(b), reg, mask, value, PSP2SGX_WAIT_MAX_POLLS);
		return;
	}
	end_run(b);
	s = add_step(b);
	if (!s)
		return;
	s->kind = STEP_WAIT;
	s->what = what;
	s->reg = reg;
	s->mask = mask;
	s->value = value;
}

/*
 * A bank 0 host clear does not reach the core banks: on the DevKit every
 * core still held OTPM_INV and DPM_CONTROL_CLEAR (0x04100200) after the
 * render cleared them in bank 0, and core 0 is one of the views a wait
 * accepts (psp2sgx_mk.c), so a stale bit there would end the next wait early.
 */
static void clear_cores(struct job_build *b, u32 reg, u32 value)
{
	unsigned int n;

	for (n = 0; n < PSP2SGX_NUM_CORES; n++)
		write_reg(b, SGX_CORE_BANK(n) + reg, value);
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
static void clear_ta_finished(struct job_build *b)
{
	write_reg(b, SGX_EVENT_HOST_CLEAR, SGX_EVENT_TA_FINISHED);
	write_reg(b, SGX_MASTER_EVENT_HOST_CLEAR, SGX_EVENT_TA_FINISHED);
	clear_cores(b, SGX_EVENT_HOST_CLEAR, SGX_EVENT_TA_FINISHED);
}

static void wait_ta_idle(struct job_build *b)
{
	unsigned int n;

	for (n = 0; n < psp2sgx_hw_cores(); n++)
		wait_reg(b, SGX_CORE_BANK(n) + SGX_CLKGATESTATUS,
			 SGX_CLKGATESTATUS_TA_UNITS, 0, WAIT_OTHER);
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
static void wait_3d_idle(struct job_build *b)
{
	unsigned int n;

	for (n = 0; n < psp2sgx_hw_cores(); n++)
		wait_reg(b, SGX_CORE_BANK(n) + SGX_CLKGATESTATUS,
			 SGX_CLKGATESTATUS_3D_UNITS, 0, WAIT_OTHER);
}

/*
 * What Sony's microkernel does between a TA and the 3D pass that a render's
 * operations cannot (a: shell register trace, every kick): once the TA is
 * done, the VDM batch number loaded and set back to 0 in the master and
 * bank 0 (on the DevKit cores 2 and 3 read VDM_BATCH_NUM_STATUS 0 and
 * never got a batch); after the tail pointer cache flush, an ISP2 soft reset
 * pulse in bank 0 before the 3D set-up.
 */
static void ta_done(struct job_build *b)
{
	write_reg(b, SGX_MASTER_VDM_BATCH_NUM, 1);
	write_reg(b, SGX_MASTER_VDM_BATCH_NUM, 0);
	write_reg(b, SGX_VDM_BATCH_NUM, 1);
	write_reg(b, SGX_VDM_BATCH_NUM, 0);
}

static void tpc_flushed(struct job_build *b)
{
	write_reg(b, SGX_SOFT_RESET, SGX_SOFT_RESET_ISP2);
	write_reg(b, SGX_SOFT_RESET, 0);
}

static int build_render(struct job_build *b,
			const struct drm_psp2sgx_reg_op *ops, u32 n)
{
	u32 i;

	for (i = 0; i < n; i++) {
		const struct drm_psp2sgx_reg_op *op = &ops[i];
		bool event = op->reg == SGX_EVENT_STATUS ||
			     op->reg == SGX_MASTER_EVENT_STATUS;
		u32 met = op->mask & op->value;

		switch (op->op) {
		case PSP2SGX_REG_WRITE:
			if (check_write(op->reg, op->value))
				return -EPERM;
			if (op->reg == SGX_VDM_START)
				clear_ta_finished(b);
			write_reg(b, op->reg, op->value);
			if (op->reg == SGX_EVENT_HOST_CLEAR || op->reg == SGX_EVENT_HOST_CLEAR2)
				clear_cores(b, op->reg, op->value);
			break;
		case PSP2SGX_REG_WAIT:
			if (!reg_in(wait_ok, ARRAY_SIZE(wait_ok), op->reg))
				return -EPERM;
			wait_reg(b, op->reg, op->mask, met,
				 event && met & SGX_EVENT_TA_FINISHED ? WAIT_TA :
				 event && met & SGX_EVENT_PIXELBE_END_RENDER ? WAIT_3D :
				 WAIT_OTHER);
			if (event && met & SGX_EVENT_TA_FINISHED) {
				wait_ta_idle(b);
				ta_done(b);
			}
			if (op->reg == SGX_EVENT_STATUS && met & SGX_EVENT_TPC_FLUSH)
				tpc_flushed(b);
			if (event && met & SGX_EVENT_PIXELBE_END_RENDER)
				wait_3d_idle(b);
			break;
		default:
			return -EINVAL;
		}
		if (b->err)
			return b->err;
		if (b->open && b->j.a.err)
			return b->j.a.err;
	}
	return 0;
}

static int build_task(struct job_build *b, const struct drm_psp2sgx_task *t)
{
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	if (!t->data_units || t->data_units > 31 || t->attr_size >> 15 ||
	    t->sideband >> 14 || t->pds_addr & 0xf || t->pad)
		return -EINVAL;
	/* the task word as Sony's microkernel builds it (0x800900: 1 unit, 9) */
	sgx_mk_job_task(run(b), t->data_units << 23 | t->attr_size << 8, t->pds_addr,
			t->ir0, t->sideband);
	return b->j.a.err;
}

static int build_code(struct psp2sgx_job *job, const struct drm_psp2sgx_submit *args)
{
	void __user *cmds = u64_to_user_ptr(args->cmds);
	struct job_build b = { .job = job, .host = host_waits };
	void *buf = NULL;
	u32 waits;
	int err;

	if (args->type == PSP2SGX_JOB_RENDER &&
	    (!args->nr_cmds || args->nr_cmds > PSP2SGX_MAX_REG_OPS))
		return -EINVAL;
	/* each operation is at most one wait and one wait per core after it */
	waits = args->type == PSP2SGX_JOB_RENDER ?
		args->nr_cmds * (1 + PSP2SGX_NUM_CORES) : 0;
	job->max_steps = 2 * waits + 2;
	job->steps = kvmalloc_array(job->max_steps, sizeof(*job->steps), GFP_KERNEL);
	job->code = kvmalloc_array(JOB_CODE_TOTAL, sizeof(u64), GFP_KERNEL);
	if (!job->steps || !job->code)
		return -ENOMEM;
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
	write_reg(&b, SGX_CLKGATECTL2, SGX_CLKGATECTL2_USE_ON);
	write_reg(&b, SGX_DMS_CTRL, SGX_DMS_CTRL_JOB);

	switch (args->type) {
	case PSP2SGX_JOB_RENDER:
		buf = memdup_array_user(cmds, args->nr_cmds,
					sizeof(struct drm_psp2sgx_reg_op));
		if (IS_ERR(buf))
			return PTR_ERR(buf);
		err = build_render(&b, buf, args->nr_cmds);
		break;
	case PSP2SGX_JOB_TASK:
		if (args->nr_cmds != 1)
			return -EINVAL;
		buf = memdup_user(cmds, sizeof(struct drm_psp2sgx_task));
		if (IS_ERR(buf))
			return PTR_ERR(buf);
		err = build_task(&b, buf);
		break;
	default:
		return -EINVAL;
	}
	kfree(buf);
	if (err)
		return err;
	write_reg(&b, SGX_CLKGATECTL2, SGX_CLKGATECTL2_AUTO);
	end_run(&b);
	job->code_len = b.used;
	return b.err;
}

/* ------------------------------------------------------------------ scheduler */

static void report_polls(struct psp2sgx_device *sgx, const char *why);
static void report_fault_bos(struct psp2sgx_device *sgx, struct psp2sgx_job *job);

/* Complete job on the hardware fence, unless a timeout took it already. */
static void finish_job(struct psp2sgx_device *sgx, struct psp2sgx_job *job, int err)
{
	unsigned long flags;

	spin_lock_irqsave(&sgx->fence_lock, flags);
	if (sgx->current_job == job) {
		sgx->current_job = NULL;
		if (err)
			dma_fence_set_error(job->done, err);
		dma_fence_signal_locked(job->done);
	}
	spin_unlock_irqrestore(&sgx->fence_lock, flags);
}

/* Post one microkernel run of the job's code; hw_lock held. */
static void kick_code(struct psp2sgx_device *sgx, struct psp2sgx_job *job,
		      const struct psp2sgx_step *s, bool first)
{
	u32 *post = psp2sgx_hostpost(sgx);

	psp2sgx_put_code((u8 *)sgx->kregion + SGX_MK_JOB_CODE, job->code + s->start,
			 s->len);
	WRITE_ONCE(job->kick, ++sgx->kick_seqno);
	if (first) {
		psp2sgx_hw_wait_cores_idle(sgx, job->seqno);
		psp2sgx_hw_set_vm(sgx, job->vm);
	}
	/*
	 * The GPU must see the code, the job and the tables, not stale lines;
	 * every run of a job has its code at the same address.  Any run of the
	 * microkernel takes a post it sees, also one an event started (a TA or
	 * 3D end the host just polled for), so the post goes out only once all
	 * of that is in place.
	 */
	if (psp2sgx_hw_slc_flush(sgx) || psp2sgx_hw_invalidate_caches(sgx))
		dev_warn_ratelimited(sgx->dev, "cache maintenance timed out\n");
	WRITE_ONCE(post[SGX_HP_POSTED], job->kick);
	wmb();
	/*
	 * Drop a clean copy of the old post a run took into the SLC meanwhile.
	 * The GPU never writes this block, so the flush cannot write anything
	 * over the post.  The host's words in the microkernel's block were
	 * written over that way (on the DevKit the block read back "idle,
	 * the previous job" at the job's timeout).
	 */
	if (psp2sgx_hw_slc_flush(sgx))
		dev_warn_ratelimited(sgx->dev, "SLC flush timed out\n");
	psp2sgx_hw_kick(sgx);
}

/*
 * A host wait is met as the microkernel's is (psp2sgx_mk.c): EVENT_STATUS
 * in bank 0 or the master by the bits in any of bank 0, the master and
 * core 0; EVENT_STATUS2 in bank 0 by bank 0 or core 0.
 */
static bool wait_met(struct psp2sgx_device *sgx, const struct psp2sgx_step *s,
		     u32 seen[3])
{
	static const u32 event_views[] = {
		SGX_EVENT_STATUS, SGX_MASTER_EVENT_STATUS,
		SGX_CORE_BANK(0) + SGX_EVENT_STATUS,
	};
	static const u32 status2_views[] = {
		SGX_EVENT_STATUS2, SGX_CORE_BANK(0) + SGX_EVENT_STATUS2,
	};
	const u32 *view = &s->reg;
	unsigned int n = 1, i;

	if (s->reg == SGX_EVENT_STATUS || s->reg == SGX_MASTER_EVENT_STATUS) {
		view = event_views;
		n = ARRAY_SIZE(event_views);
	} else if (s->reg == SGX_EVENT_STATUS2) {
		view = status2_views;
		n = ARRAY_SIZE(status2_views);
	}
	for (i = 0; i < n; i++) {
		seen[i] = sgx_read(sgx, view[i]);
		if ((seen[i] & s->mask) == s->value)
			return true;
	}
	return false;
}

/*
 * Poll from the CPU.  A TA took about 140 us on the DevKit and a 3D pass
 * about 1.4 ms (the wait statistics), so the first polls spin and
 * the rest sleep.
 */
static int host_wait(struct psp2sgx_device *sgx, const struct psp2sgx_step *s,
		     u32 seen[3], s64 *us)
{
	ktime_t t0 = ktime_get();
	unsigned int spins = 0;

	for (;;) {
		bool met = wait_met(sgx, s, seen);

		*us = ktime_us_delta(ktime_get(), t0);
		if (met)
			return 0;
		if (*us > PSP2SGX_HOST_WAIT_US)
			return -ETIMEDOUT;
		if (spins++ < 64)
			udelay(2);
		else
			usleep_range(10, 25);
	}
}

static void account_host_wait(struct psp2sgx_device *sgx, u8 what, s64 us)
{
	typeof(sgx->host_polls[0]) *p;

	if (what == WAIT_OTHER)
		return;
	p = &sgx->host_polls[what == WAIT_3D];
	p->jobs++;
	p->sum_us += us;
	p->max_us = max_t(u32, p->max_us, us);
}

/* A host wait gave up: report, and fail the job; hw_lock held. */
static void wait_failed(struct psp2sgx_device *sgx, struct psp2sgx_job *job,
			const struct psp2sgx_step *s, const u32 seen[3], s64 us)
{
	u32 late[3];

	dev_warn(sgx->dev, "job %u: wait for %#x & %#x == %#x gave up after %lld us: read %#x %#x %#x, cores EVENT_STATUS %#x %#x %#x %#x, clocks %08x %08x %08x %08x\n",
		 job->seqno, s->reg, s->mask, s->value, us, seen[0], seen[1], seen[2],
		 sgx_read(sgx, SGX_CORE_BANK(0) + SGX_EVENT_STATUS),
		 sgx_read(sgx, SGX_CORE_BANK(1) + SGX_EVENT_STATUS),
		 sgx_read(sgx, SGX_CORE_BANK(2) + SGX_EVENT_STATUS),
		 sgx_read(sgx, SGX_CORE_BANK(3) + SGX_EVENT_STATUS),
		 sgx_read(sgx, SGX_CORE_BANK(0) + SGX_CLKGATESTATUS),
		 sgx_read(sgx, SGX_CORE_BANK(1) + SGX_CLKGATESTATUS),
		 sgx_read(sgx, SGX_CORE_BANK(2) + SGX_CLKGATESTATUS),
		 sgx_read(sgx, SGX_CORE_BANK(3) + SGX_CLKGATESTATUS));
	report_polls(sgx, "before the failure");
	if (!sgx->regs_dumped)
		report_fault_bos(sgx, job);
	psp2sgx_diag_dump(sgx, "failed wait");
	psp2sgx_diag_regdump(sgx, "failed wait");
	/* did it come after all: a slow render, not a hang */
	dev_warn(sgx->dev, "job %u: after the dump the wait is %s\n", job->seqno,
		 wait_met(sgx, s, late) ? "met" : "still not met");
	sgx->needs_reset = true;
	finish_job(sgx, job, -EIO);
}

/*
 * Run the job's steps from job->step: host waits here, up to the next
 * code, which the microkernel runs; its interrupt continues from there.
 * hw_lock held.
 */
static void advance(struct psp2sgx_device *sgx, struct psp2sgx_job *job)
{
	while (job->step < job->nr_steps) {
		const struct psp2sgx_step *s = &job->steps[job->step++];
		u32 seen[3] = {};
		s64 us;

		if (READ_ONCE(sgx->current_job) != job)
			return;		/* a timeout took it */
		if (s->kind == STEP_CODE) {
			kick_code(sgx, job, s, s == job->steps);
			return;
		}
		if (host_wait(sgx, s, seen, &us)) {
			wait_failed(sgx, job, s, seen, us);
			return;
		}
		account_host_wait(sgx, s->what, us);
	}
	finish_job(sgx, job, 0);
}

/* Whether the job's last microkernel run is done, as the host can see it. */
static bool run_done(struct psp2sgx_device *sgx, struct psp2sgx_job *job)
{
	u32 *hc = psp2sgx_hostctl(sgx);

	if (psp2sgx_hw_slc_writeback(sgx))
		dev_warn_ratelimited(sgx->dev, "SLC write-back timed out\n");
	rmb();
	return (s32)(READ_ONCE(hc[SGX_HC_DONE_SEQNO]) - READ_ONCE(job->kick)) >= 0;
}

/*
 * Go on with the current job after its run.  Queued by the run's interrupt,
 * and by a timeout that finds the run done without one; the second must
 * not start the steps after a run that is still going.
 */
static void psp2sgx_step_work(struct work_struct *work)
{
	struct psp2sgx_device *sgx = container_of(work, struct psp2sgx_device,
						  step_work);
	struct psp2sgx_job *job;
	unsigned long flags;

	mutex_lock(&sgx->hw_lock);
	spin_lock_irqsave(&sgx->fence_lock, flags);
	job = sgx->current_job;
	spin_unlock_irqrestore(&sgx->fence_lock, flags);
	if (job && run_done(sgx, job))
		advance(sgx, job);
	mutex_unlock(&sgx->hw_lock);
}

static struct dma_fence *psp2sgx_run_job(struct drm_sched_job *sched_job)
{
	struct psp2sgx_job *job = to_job(sched_job);
	struct psp2sgx_device *sgx = job->sgx;
	struct dma_fence *fence;
	unsigned long flags;

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
	job->step = 0;
	sgx->current_job = job;
	spin_unlock_irqrestore(&sgx->fence_lock, flags);

	advance(sgx, job);	/* the first step is code: kicks and returns */
	mutex_unlock(&sgx->hw_lock);
	return fence;
}

/*
 * How long the TA and 3D waits of finished jobs polled, against the budget:
 * a render that is slow and one held back until its wait gives up look the
 * same in a timeout alone.  Reported every 2000 jobs and with each failure.
 */
static void account_polls(struct psp2sgx_device *sgx, const u32 *hc)
{
	static const u32 shift[4] = { 6, 4, 2, 1 };
	unsigned int k, i;

	for (k = 0; k < 2; k++) {
		u32 left = hc[k ? SGX_HC_POLLS_EVENT : SGX_HC_POLLS_TA], used;

		if (!left || left > PSP2SGX_WAIT_MAX_POLLS)
			continue;	/* no such wait, or it timed out */
		used = PSP2SGX_WAIT_MAX_POLLS - left + 1;
		sgx->polls[k].jobs++;
		sgx->polls[k].sum += used;
		sgx->polls[k].max = max(sgx->polls[k].max, used);
		for (i = 0; i < 4; i++)
			if (used > PSP2SGX_WAIT_MAX_POLLS >> shift[i])
				sgx->polls[k].over[i]++;
	}
}

static void report_polls(struct psp2sgx_device *sgx, const char *why)
{
	unsigned int k;

	for (k = 0; k < 2; k++) {
		typeof(sgx->polls[0]) *p = &sgx->polls[k];
		typeof(sgx->host_polls[0]) *h = &sgx->host_polls[k];

		if (host_waits) {
			dev_info(sgx->dev, "%s: %s waits %u, host polled mean %llu us max %u us of %u\n",
				 why, k ? "3D" : "TA", h->jobs,
				 h->jobs ? div_u64(h->sum_us, h->jobs) : 0, h->max_us,
				 PSP2SGX_HOST_WAIT_US);
			continue;
		}
		dev_info(sgx->dev, "%s: %s waits %u, polls mean %llu max %u of %u, over 1/64 %u 1/16 %u 1/4 %u 1/2 %u\n",
			 why, k ? "3D" : "TA", p->jobs,
			 p->jobs ? div_u64(p->sum, p->jobs) : 0, p->max,
			 PSP2SGX_WAIT_MAX_POLLS, p->over[0], p->over[1],
			 p->over[2], p->over[3]);
	}
	memset(sgx->polls, 0, sizeof(sgx->polls));
	memset(sgx->host_polls, 0, sizeof(sgx->host_polls));
}

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
	u32 kick = READ_ONCE(sgx->kick_seqno);
	struct psp2sgx_job *job;
	unsigned long flags;
	unsigned int us;
	u32 done, error;
	bool dump = false;

	/*
	 * The microkernel's stores sit in the SLC until written back.  Only
	 * written back, not dropped: the run may have started a TA or a 3D
	 * pass that is still storing, and an invalidate would lose its data.
	 */
	for (us = 0;; us += 20) {
		if (psp2sgx_hw_slc_writeback(sgx))
			dev_warn_ratelimited(sgx->dev, "SLC write-back timed out\n");
		rmb();
		done = READ_ONCE(hc[SGX_HC_DONE_SEQNO]);
		if (done == kick || us >= 1000)
			break;
		udelay(20);
	}
	error = READ_ONCE(hc[SGX_HC_ERROR]);
	sgx->sw_events++;
	sgx->irq_done = done;
	/*
	 * The run raised SW_EVENT only after its DONE_SEQNO store, so the host
	 * should read it at once.  On the DevKit a run finished, DONE_SEQNO
	 * read equal to its kick at the timeout 2 s later, and the job had not
	 * gone on.
	 */
	if (us)
		dev_warn_ratelimited(sgx->dev, "run %u: DONE_SEQNO %s %u us after its interrupt (%u so far)\n",
				     kick, done == kick ? "arrived" : "still behind, read",
				     us, ++sgx->late_done);

	spin_lock_irqsave(&sgx->fence_lock, flags);
	job = sgx->current_job;
	if (job && (s32)(done - READ_ONCE(job->kick)) >= 0) {
		/* host_waits: the job goes on after this run */
		if (!error && job->step < job->nr_steps) {
			queue_work(sgx->step_wq, &sgx->step_work);
			spin_unlock_irqrestore(&sgx->fence_lock, flags);
			return;
		}
		account_polls(sgx, hc);
		if (error) {
			dev_warn_ratelimited(sgx->dev, "job %u: microkernel error %#x, cores 1-3 EVENT_STATUS %#x %#x %#x\n",
					     job->seqno, error, hc[SGX_HC_DIAG_CORES],
					     hc[SGX_HC_DIAG_CORES + 1],
					     hc[SGX_HC_DIAG_CORES + 2]);
			report_polls(sgx, "before the failure");
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
		if (sgx->polls[1].jobs >= 2000 || sgx->host_polls[1].jobs >= 2000)
			report_polls(sgx, "wait statistics");
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

	/*
	 * Its last run finished but nothing took the job on: go on with it
	 * (the step work checks the run again under hw_lock).  A job that is
	 * in a slow host wait gets here too, and loses nothing.  Three times
	 * at most, so a job that never gets anywhere still fails.
	 */
	if (READ_ONCE(sgx->current_job) == job && job->rescues < 3 &&
	    run_done(sgx, job) && !READ_ONCE(hc[SGX_HC_ERROR])) {
		job->rescues++;
		dev_warn(sgx->dev, "job %u at step %u of %u: run %u is done without its completion taken (SW_EVENTs %u, the last read DONE_SEQNO %u, %u such runs); going on\n",
			 job->seqno, job->step, job->nr_steps, READ_ONCE(job->kick),
			 sgx->sw_events, sgx->irq_done, ++sgx->missed_done);
		queue_work(sgx->step_wq, &sgx->step_work);
		return DRM_GPU_SCHED_STAT_NO_HANG;
	}

	/* the job is off the hardware from here; no step work continues it */
	spin_lock_irqsave(&sgx->fence_lock, flags);
	sgx->current_job = NULL;
	spin_unlock_irqrestore(&sgx->fence_lock, flags);
	cancel_work_sync(&sgx->step_work);

	dev_err(sgx->dev, "job %u timed out at step %u of %u: kick %u posted %u, state %u claimed %u done %u error %#x runs %u, EVENT_STATUS %#x, SW_EVENTs %u last read done %u\n",
		job->seqno, job->step, job->nr_steps, READ_ONCE(job->kick),
		READ_ONCE(psp2sgx_hostpost(sgx)[SGX_HP_POSTED]),
		READ_ONCE(hc[SGX_HC_STATE]), READ_ONCE(hc[SGX_HC_CLAIMED]),
		READ_ONCE(hc[SGX_HC_DONE_SEQNO]), READ_ONCE(hc[SGX_HC_ERROR]),
		READ_ONCE(hc[SGX_HC_RUNS]), sgx_read(sgx, SGX_EVENT_STATUS),
		sgx->sw_events, sgx->irq_done);
	if (sgx->timeouts_dumped < 2) {
		sgx->timeouts_dumped++;
		mutex_lock(&sgx->hw_lock);
		psp2sgx_diag_dump(sgx, "job timeout");
		mutex_unlock(&sgx->hw_lock);
	}

	drm_sched_stop(&sgx->sched, sched_job);
	drm_sched_increase_karma(sched_job);

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
	kvfree(job->steps);
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

	int err;

	sgx->fence_context = dma_fence_context_alloc(1);
	spin_lock_init(&sgx->fence_lock);
	INIT_WORK(&sgx->step_work, psp2sgx_step_work);
	sgx->step_wq = alloc_ordered_workqueue("psp2sgx-steps", WQ_HIGHPRI | WQ_MEM_RECLAIM);
	if (!sgx->step_wq)
		return -ENOMEM;
	err = drm_sched_init(&sgx->sched, &args);
	if (err)
		destroy_workqueue(sgx->step_wq);
	return err;
}

void psp2sgx_sched_fini(struct psp2sgx_device *sgx)
{
	drm_sched_fini(&sgx->sched);
	destroy_workqueue(sgx->step_wq);
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
