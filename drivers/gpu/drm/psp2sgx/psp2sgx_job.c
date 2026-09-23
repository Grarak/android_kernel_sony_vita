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
 * DPM, TSP and pixel event registers the DDK's microkernel programs for a
 * TA/3D kick, the BIF requestor bases (addresses the MMU still translates),
 * and the event clears for what a render waits on.  Not the BIF, code bases,
 * event enables or kicks: those belong to the kernel.
 */
static const struct reg_range write_ok[] = {
	{ 0x0114, 0x0114 },	/* EVENT_HOST_CLEAR2, bits checked below */
	{ 0x0134, 0x0134 },	/* EVENT_HOST_CLEAR, bits checked below */
	{ 0x0140, 0x0140 },	/* PDS_CACHE_HOST_CLEAR */
	{ 0x0200, 0x025c },	/* VDM, TE, MTE */
	{ 0x0380, 0x038c },	/* TE_RGNHDR_INIT */
	{ 0x0400, 0x052c },	/* ISP */
	{ 0x0600, 0x06a0 },	/* DPM */
	{ 0x081c, 0x0848 },	/* TSP */
	{ 0x0a5c, 0x0a64 },	/* EVENT_PIXEL_PDS_EXEC/DATA/INFO */
	{ 0x0c90, 0x0c90 },	/* BIF_TA_REQ_BASE */
	{ 0x0cac, 0x0cb0 },	/* BIF_3D_REQ_BASE, BIF_ZLS_REQ_BASE */
	{ 0x4320, 0x4320 },	/* master, as the DDK's 3D kick */
	{ 0x4510, 0x451c },	/* master DPM, as the DDK's TA kick */
};

/* What a render may wait on: status registers only. */
static const struct reg_range wait_ok[] = {
	{ 0x0118, 0x0118 },	/* EVENT_STATUS2 */
	{ 0x012c, 0x012c },	/* EVENT_STATUS */
	{ 0x0138, 0x0138 },	/* PDS_CACHE_STATUS */
	{ 0x0600, 0x0730 },	/* DPM state and status */
	{ 0x412c, 0x412c },	/* MASTER_EVENT_STATUS */
};

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
	if (!reg_in(write_ok, ARRAY_SIZE(write_ok), reg))
		return -EPERM;
	/* the host owns the interrupts it has enabled */
	if (reg == SGX_EVENT_HOST_CLEAR && value & SGX_EVENT_SW_EVENT)
		return -EPERM;
	if (reg == SGX_EVENT_HOST_CLEAR2 && value & 0xc00)
		return -EPERM;
	return 0;
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
			sgx_mk_job_write(j, op->reg, op->value);
			break;
		case PSP2SGX_REG_WAIT:
			if (!reg_in(wait_ok, ARRAY_SIZE(wait_ok), op->reg))
				return -EPERM;
			sgx_mk_job_wait(j, op->reg, op->mask, op->value & op->mask,
					PSP2SGX_WAIT_MAX_POLLS);
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
	if (!err)
		err = sgx_mk_job_end(&j);
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

	if (unlikely(sched_job->s_fence->finished.error))
		return NULL;

	fence = kzalloc(sizeof(*fence), GFP_KERNEL);
	if (!fence)
		return ERR_PTR(-ENOMEM);

	mutex_lock(&sgx->hw_lock);
	spin_lock_irqsave(&sgx->fence_lock, flags);
	dma_fence_init(fence, &psp2sgx_fence_ops, &sgx->fence_lock,
		       sgx->fence_context, ++sgx->emit_seqno);
	job->seqno = lower_32_bits(sgx->emit_seqno);
	dma_fence_put(job->done);
	job->done = dma_fence_get(fence);
	sgx->current_job = job;
	spin_unlock_irqrestore(&sgx->fence_lock, flags);

	psp2sgx_put_code((u8 *)sgx->kregion + SGX_MK_JOB_CODE, job->code,
			 job->code_len);
	WRITE_ONCE(hc[SGX_HC_JOB_SEQNO], job->seqno);
	WRITE_ONCE(hc[SGX_HC_ERROR], 0);
	wmb();
	WRITE_ONCE(hc[SGX_HC_STATE], SGX_MK_PENDING);
	psp2sgx_hw_set_vm(sgx, job->vm);
	/* the GPU must see the code, the job and the tables, not stale lines */
	if (psp2sgx_hw_slc_flush(sgx) || psp2sgx_hw_invalidate_caches(sgx))
		dev_warn_ratelimited(sgx->dev, "cache maintenance timed out\n");
	psp2sgx_hw_kick(sgx);
	mutex_unlock(&sgx->hw_lock);
	return fence;
}

/* The SW_EVENT interrupt, from the threaded handler. */
void psp2sgx_job_irq(struct psp2sgx_device *sgx)
{
	u32 *hc = psp2sgx_hostctl(sgx);
	struct psp2sgx_job *job;
	unsigned long flags;
	u32 done, error;

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
		}
		sgx->current_job = NULL;
		dma_fence_signal_locked(job->done);
	}
	spin_unlock_irqrestore(&sgx->fence_lock, flags);
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
	if (!job->bos) {
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
		err = psp2sgx_bo_map_get(to_psp2sgx_bo(obj), job->vm, NULL);
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
