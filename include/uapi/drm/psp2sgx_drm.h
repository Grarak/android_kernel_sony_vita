/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * psp2sgx: the PowerVR SGX543MP4+ of the PlayStation Vita.
 *
 * The driver runs its own microkernel on the GPU.  Userspace allocates
 * buffer objects, which the kernel maps into a GPU address space of their
 * own per open file, and submits jobs: a list of register operations that
 * the kernel checks against what a render may touch, turns into microkernel
 * code and runs in order.  Shader and PDS programs, vertex control streams
 * and everything else the hardware reads come from buffer objects.
 */
#ifndef __PSP2SGX_DRM_H__
#define __PSP2SGX_DRM_H__

#include "drm.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define PSP2SGX_UAPI_VERSION	2

enum drm_psp2sgx_param {
	DRM_PSP2SGX_PARAM_UAPI_VERSION,
	DRM_PSP2SGX_PARAM_CORE_ID,		/* EUR_CR_CORE_ID */
	DRM_PSP2SGX_PARAM_CORE_REVISION,	/* EUR_CR_CORE_REVISION */
	DRM_PSP2SGX_PARAM_NUM_CORES,
	/* GPU virtual address ranges of the heaps (see PSP2SGX_BO_HEAP_*) */
	DRM_PSP2SGX_PARAM_GENERAL_HEAP_BASE,
	DRM_PSP2SGX_PARAM_GENERAL_HEAP_SIZE,
	/* USSE code windows: code base 0 (vertex) and 1 (fragment) */
	DRM_PSP2SGX_PARAM_CODE_VERTEX_BASE,
	DRM_PSP2SGX_PARAM_CODE_FRAGMENT_BASE,
	DRM_PSP2SGX_PARAM_CODE_WINDOW_SIZE,
	/* the PDS code heap: PDS programs and their data segments */
	DRM_PSP2SGX_PARAM_PDS_HEAP_BASE,
	DRM_PSP2SGX_PARAM_PDS_HEAP_SIZE,
	/* maximum register operations per job */
	DRM_PSP2SGX_PARAM_MAX_REG_OPS,
};

struct drm_psp2sgx_get_param {
	__u32 param;		/* in: enum drm_psp2sgx_param */
	__u32 pad;		/* must be zero */
	__u64 value;		/* out */
};

/* Which GPU address range a buffer object is placed in */
#define PSP2SGX_BO_HEAP_GENERAL		0
#define PSP2SGX_BO_HEAP_CODE_VERTEX	1	/* USSE code, code base 0 */
#define PSP2SGX_BO_HEAP_CODE_FRAGMENT	2	/* USSE code, code base 1 */
#define PSP2SGX_BO_HEAP_PDS		3	/* PDS programs */
#define PSP2SGX_BO_HEAP_MASK		0xf

struct drm_psp2sgx_bo_create {
	__u64 size;		/* in, bytes, rounded up to 4 KiB */
	__u32 flags;		/* in: PSP2SGX_BO_HEAP_* */
	__u32 handle;		/* out */
	__u32 gpu_va;		/* out: address in this file's GPU address space */
	__u32 pad;		/* must be zero */
};

struct drm_psp2sgx_bo_info {
	__u32 handle;		/* in */
	__u32 gpu_va;		/* out */
	__u64 mmap_offset;	/* out: for mmap() on the DRM fd (write-combined) */
	__u64 size;		/* out */
};

#define PSP2SGX_BO_WAIT_READ	(1 << 0)	/* wait for the GPU's writes */
#define PSP2SGX_BO_WAIT_WRITE	(1 << 1)	/* wait for all GPU use */

struct drm_psp2sgx_bo_wait {
	__u32 handle;
	__u32 op;		/* PSP2SGX_BO_WAIT_* */
	__s64 timeout_ns;	/* absolute, CLOCK_MONOTONIC */
};

/*
 * A register operation of a job.  Registers are byte offsets in the SGX
 * register block; the kernel accepts only those a render needs (the vertex
 * data master, tiling, ISP, DPM, TSP, pixel event and requestor base
 * registers, and the event status bits a render waits on and clears) and
 * rejects the job otherwise.
 */
#define PSP2SGX_REG_WRITE	1	/* reg = value */
#define PSP2SGX_REG_WAIT	2	/* until (reg & mask) == value, bounded */

struct drm_psp2sgx_reg_op {
	__u32 op;
	__u32 reg;
	__u32 value;
	__u32 mask;
};

/*
 * A raw PDS task, for bring-up and tests only (CAP_SYS_ADMIN): the kernel
 * emits it from the microkernel as Sony's microkernel emits its own tasks and
 * waits for the task to write 1 to core 0's USE_GLOBCOM1.
 */
struct drm_psp2sgx_task {
	__u32 pds_addr;		/* GPU address of the PDS program, 16-byte aligned */
	__u32 data_units;	/* its data segment in 16-byte units */
	__u32 attr_size;	/* the task word's attribute field */
	__u32 sideband;		/* the emit's 14-bit sideband */
	__u32 ir0;
	__u32 pad;
};

#define PSP2SGX_JOB_RENDER	0	/* cmds: struct drm_psp2sgx_reg_op[] */
#define PSP2SGX_JOB_TASK	1	/* cmds: one struct drm_psp2sgx_task */

#define PSP2SGX_SUBMIT_BO_WRITE		(1 << 0)

struct drm_psp2sgx_submit_bo {
	__u32 handle;
	__u32 flags;		/* PSP2SGX_SUBMIT_BO_WRITE */
};

#define PSP2SGX_SUBMIT_FENCE_FD_IN	(1 << 0)	/* wait on fence_fd first */
#define PSP2SGX_SUBMIT_FENCE_FD_OUT	(1 << 1)	/* return a sync_file in fence_fd */

struct drm_psp2sgx_submit {
	__u64 bos;		/* struct drm_psp2sgx_submit_bo[] */
	__u64 cmds;
	__u32 nr_bos;
	__u32 nr_cmds;
	__u32 type;		/* PSP2SGX_JOB_* */
	__u32 flags;		/* PSP2SGX_SUBMIT_* */
	__u32 in_sync;		/* syncobj to wait on, or 0 */
	__u32 out_sync;		/* syncobj to signal, or 0 */
	__s32 fence_fd;
	__u32 pad;
};

#define DRM_PSP2SGX_GET_PARAM	0x00
#define DRM_PSP2SGX_BO_CREATE	0x01
#define DRM_PSP2SGX_BO_INFO	0x02
#define DRM_PSP2SGX_BO_WAIT	0x03
#define DRM_PSP2SGX_SUBMIT	0x04

#define DRM_IOCTL_PSP2SGX_GET_PARAM \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_PSP2SGX_GET_PARAM, struct drm_psp2sgx_get_param)
#define DRM_IOCTL_PSP2SGX_BO_CREATE \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_PSP2SGX_BO_CREATE, struct drm_psp2sgx_bo_create)
#define DRM_IOCTL_PSP2SGX_BO_INFO \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_PSP2SGX_BO_INFO, struct drm_psp2sgx_bo_info)
#define DRM_IOCTL_PSP2SGX_BO_WAIT \
	DRM_IOW(DRM_COMMAND_BASE + DRM_PSP2SGX_BO_WAIT, struct drm_psp2sgx_bo_wait)
#define DRM_IOCTL_PSP2SGX_SUBMIT \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_PSP2SGX_SUBMIT, struct drm_psp2sgx_submit)

#if defined(__cplusplus)
}
#endif

#endif
