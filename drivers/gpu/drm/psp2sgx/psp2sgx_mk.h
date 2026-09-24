/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The psp2sgx microkernel: our own USSE code, assembled at probe time.
 *
 * The host kicks core 0 (EVENT_KICK2 in its bank).  The kick runs the event
 * PDS program, which hands IR0, IR1 and the host control block address to the
 * microkernel entry (code base 15, index 0).  The microkernel claims the job
 * the host posted in the host control block, calls the job's code, records
 * the job's sequence number as done and raises the host interrupt
 * (EVENT_STATUS SW_EVENT), as Sony's microkernel does.
 *
 * Job code is also assembled by the kernel, per job, from commands it has
 * validated (psp2sgx_job.c): register writes, bounded register polls and,
 * for CAP_SYS_ADMIN only, a raw PDS task.  It lives at a fixed place in the
 * same code window and returns to the microkernel with LAPC.
 *
 * Everything sits in the kernel region, one 4 MiB page directory entry that
 * every GPU VM maps at SGX_KREGION_VA (psp2sgx_mmu.c).
 */
#ifndef PSP2SGX_MK_H
#define PSP2SGX_MK_H

#include <linux/types.h>

#include "psp2sgx_usse.h"

#define SGX_KREGION_VA		0x00400000u
#define SGX_KREGION_SPAN	0x00400000u

/* Offsets in the kernel region */
#define SGX_MK_HOSTCTL		0x0000	/* host control block, 64 bytes */
#define SGX_MK_EVENT_PDS	0x0100	/* the event PDS program */
#define SGX_MK_KICKER		0x0200	/* EVENT_KICKER target word */
#define SGX_MK_CODE		0x1000	/* code base 15 window: microkernel */
#define SGX_MK_CODE_MAX		512	/* instructions */
#define SGX_MK_JOB_CODE		0x2000	/* job code, same window */
#define SGX_MK_JOB_CODE_MAX	4096	/* instructions */
#define SGX_MK_JOB_INDEX	((SGX_MK_JOB_CODE - SGX_MK_CODE) / 8)
#define SGX_MK_REGION_USED	(SGX_MK_JOB_CODE + SGX_MK_JOB_CODE_MAX * 8)

/* The USSE code base register the microkernel runs from */
#define SGX_MK_CODE_BASE	15
/* Temporaries of a microkernel task, in units of four registers */
#define SGX_MK_TEMP_UNITS	8

/* Host control block words */
enum sgx_mk_hostctl {
	SGX_HC_STATE,		/* host: PENDING; microkernel: RUNNING, IDLE */
	SGX_HC_JOB_SEQNO,	/* host: the posted job's sequence number */
	SGX_HC_DONE_SEQNO,	/* microkernel: the last finished job */
	SGX_HC_ERROR,		/* microkernel: 0, or SGX_MK_ERR_* | register */
	SGX_HC_RUNS,		/* microkernel: runs that took the lock */
	/*
	 * microkernel: EVENT_STATUS in bank 0, the master and core 0 when the
	 * last wait for TA_FINISHED, then for any other event, met or timed out
	 */
	SGX_HC_DIAG_TA,
	SGX_HC_DIAG_EVENT = SGX_HC_DIAG_TA + 3,
	SGX_HC_WORDS = 16,
};

#define SGX_MK_IDLE		0
#define SGX_MK_PENDING		1
#define SGX_MK_RUNNING		2

#define SGX_MK_ERR_WAIT_TIMEOUT	0x80000000u

/* Registers the microkernel keeps across job code */
#define SGX_MK_R_HOSTCTL	0

/* Assembles the microkernel for a kernel region at kregion_va. */
int sgx_mk_build(u64 *code, u32 cap, u32 *len, u32 kregion_va);

/* Job code */
struct sgx_mk_job {
	struct sgx_usse_asm a;
};

void sgx_mk_job_begin(struct sgx_mk_job *j, u64 *buf, u32 cap);
/* SGX register `reg` (byte offset) = value */
void sgx_mk_job_write(struct sgx_mk_job *j, u32 reg, u32 value);
/*
 * Poll until (register & mask) == value, at most max_polls rounds; on a
 * timeout record SGX_MK_ERR_WAIT_TIMEOUT | reg and return to the
 * microkernel, skipping the rest of the job.  A wait on EVENT_STATUS in
 * bank 0 or the master is met by the bits in any of bank 0, the master and
 * core 0, which of them latch the TA and 3D events on the hardware being
 * open, and leaves the three in the host control block's DIAG words.
 */
void sgx_mk_job_wait(struct sgx_mk_job *j, u32 reg, u32 mask, u32 value,
		     u32 max_polls);
/*
 * Emit a PDS task and wait for it to signal core 0's USE_GLOBCOM1 bit 0, as
 * Sony's microkernel does with its own tasks.
 */
void sgx_mk_job_task(struct sgx_mk_job *j, u32 task_word, u32 pds_addr,
		     u32 ir0, u32 sideband);
int sgx_mk_job_end(struct sgx_mk_job *j);

#endif
