// SPDX-License-Identifier: GPL-2.0-only
/*
 * The psp2sgx microkernel and job code; see psp2sgx_mk.h.
 *
 * Only instruction forms that run on the hardware are used: each is a form
 * Sony's microkernel (gpuinit_es4 program A) or a bare-metal probe runs,
 * with registers and immediates changed, and host tests compare the words.
 * What the hardware showed that shapes the code:
 *  - loads and register reads are only valid after WDF on their counter;
 *  - stores need IDF + WDF before anything that depends on them being seen,
 *    and even then sit in the system-level cache until someone flushes it
 *    (the host does, psp2sgx_hw.c);
 *  - 32-bit ordered compares are not verified, so every test here is an
 *    equality test (AND / XOR with the zero flag), as in Sony's code.
 *
 * Registers: r0 host control block (kept across job code), r1..r3 the
 * microkernel's scratch, r16..r23 job code scratch.  A task gets 32
 * temporaries (SGX_MK_TEMP_UNITS).
 */
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/types.h>

#include "psp2sgx_mk.h"
#include "psp2sgx_regs.h"

#define R(n)	USSE_R(n)
#define IMM(n)	USSE_IMM(n)
#define HC	R(SGX_MK_R_HOSTCTL)

#define SGX_USE_GLOBCOM1_CORE0	(SGX_CORE_BANK(0) + SGX_USE_GLOBCOM1)

int sgx_mk_build(u64 *code, u32 cap, u32 *len, u32 kregion_va)
{
	struct sgx_usse_asm as, *a = &as;
	int unlock_exit = 0, exit = 0;

	sgx_usse_init(a, code, cap, 0);
	unlock_exit = sgx_usse_label(a);
	exit = sgx_usse_label(a);

	sgx_usse_smlsi(a);
	sgx_usse_limm(a, SGX_USSE_ALWAYS, HC, kregion_va + SGX_MK_HOSTCTL);

	/*
	 * Claim the posted job.  Other events can start the event program
	 * too, so two microkernel threads may run at once; the mutex makes
	 * the check and the claim one step.
	 */
	sgx_usse_lock(a, 7);
	sgx_usse_ld(a, SGX_USSE_ALWAYS, R(1), HC, IMM(SGX_HC_RUNS), 0);
	sgx_usse_ld(a, SGX_USSE_ALWAYS, R(2), HC, IMM(SGX_HC_STATE), 0);
	sgx_usse_wdf(a, 0);
	sgx_usse_add(a, SGX_USSE_ALWAYS, R(1), R(1), IMM(1));
	sgx_usse_st(a, SGX_USSE_ALWAYS, HC, IMM(SGX_HC_RUNS), R(1));
	sgx_usse_test(a, SGX_USSE_ALWAYS, 0, SGX_USSE_TST_XOR, SGX_USSE_ZERO,
		      R(2), IMM(SGX_MK_PENDING));
	sgx_usse_ba(a, SGX_USSE_NOT_P0, unlock_exit);
	sgx_usse_limm(a, SGX_USSE_ALWAYS, R(2), SGX_MK_RUNNING);
	sgx_usse_st(a, SGX_USSE_ALWAYS, HC, IMM(SGX_HC_STATE), R(2));
	sgx_usse_limm(a, SGX_USSE_ALWAYS, R(2), 0);
	sgx_usse_st(a, SGX_USSE_ALWAYS, HC, IMM(SGX_HC_ERROR), R(2));
	sgx_usse_idf(a, 0);
	sgx_usse_wdf(a, 0);
	sgx_usse_release(a, 7);

	sgx_usse_call_index(a, SGX_USSE_ALWAYS, SGX_MK_JOB_INDEX);

	/* Done: publish the sequence number, go idle, interrupt the host. */
	sgx_usse_ld(a, SGX_USSE_ALWAYS, R(1), HC, IMM(SGX_HC_JOB_SEQNO), 0);
	sgx_usse_wdf(a, 0);
	sgx_usse_st(a, SGX_USSE_ALWAYS, HC, IMM(SGX_HC_DONE_SEQNO), R(1));
	sgx_usse_limm(a, SGX_USSE_ALWAYS, R(2), SGX_MK_IDLE);
	sgx_usse_st(a, SGX_USSE_ALWAYS, HC, IMM(SGX_HC_STATE), R(2));
	sgx_usse_idf(a, 0);
	sgx_usse_wdf(a, 0);
	sgx_usse_limm(a, SGX_USSE_ALWAYS, R(1), SGX_EVENT_SW_EVENT);
	sgx_usse_str(a, SGX_USSE_ALWAYS, SGX_EVENT_STATUS, R(1));
	sgx_usse_ba(a, SGX_USSE_ALWAYS, exit);

	sgx_usse_bind(a, unlock_exit);
	sgx_usse_idf(a, 0);
	sgx_usse_wdf(a, 0);
	sgx_usse_release(a, 7);

	sgx_usse_bind(a, exit);
	sgx_usse_phas_end(a);
	sgx_usse_nop_end(a);

	if (sgx_usse_finish(a))
		return a->err;
	if (a->len > SGX_MK_CODE_MAX)
		return -E2BIG;
	*len = a->len;
	return 0;
}

void sgx_mk_job_begin(struct sgx_mk_job *j, u64 *buf, u32 cap)
{
	sgx_usse_init(&j->a, buf, cap, SGX_MK_JOB_INDEX);
}

void sgx_mk_job_write(struct sgx_mk_job *j, u32 reg, u32 value)
{
	struct sgx_usse_asm *a = &j->a;

	sgx_usse_limm(a, SGX_USSE_ALWAYS, R(16), value);
	sgx_usse_str(a, SGX_USSE_ALWAYS, reg, R(16));
}

/*
 * Job code branches to computed indexes, so a job of any length needs no
 * labels: loops go back to an index taken before them, and the one forward
 * branch skips a sequence of known length.  The link register holds the
 * microkernel's return address throughout (job code makes no calls), so a
 * job ends early with a plain LAPC.
 */
#define WAIT_CHECK_LEN		5	/* instructions polling one register */
#define WAIT_TIMEOUT_TAIL	8	/* after the last check, without the diagnostic */
#define WAIT_DIAG_LEN		3	/* instructions storing one register */

static const u32 event_views[] = {
	SGX_EVENT_STATUS, SGX_MASTER_EVENT_STATUS, SGX_CORE_BANK(0) + SGX_EVENT_STATUS,
};

/*
 * On the DevKit TE_RGNHDR_INIT_COMPLETE (0x20) latched in core 0's
 * EVENT_STATUS2 only: bank 0 read 0x90 and core 0 0xb0 while the wait in
 * bank 0 went on (the psp2sgx diag of a job timeout, 2026-09-24).
 */
static const u32 status2_views[] = {
	SGX_EVENT_STATUS2, SGX_CORE_BANK(0) + SGX_EVENT_STATUS2,
};

static void wait_diag(struct sgx_usse_asm *a, u32 slot, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		sgx_usse_ldr(a, R(17), event_views[i], 0);
		sgx_usse_wdf(a, 0);
		sgx_usse_st(a, SGX_USSE_ALWAYS, HC, IMM(slot + i), R(17));
	}
}

void sgx_mk_job_wait(struct sgx_mk_job *j, u32 reg, u32 mask, u32 value,
		     u32 max_polls)
{
	struct sgx_usse_asm *a = &j->a;
	bool event = reg == SGX_EVENT_STATUS || reg == SGX_MASTER_EVENT_STATUS;
	const u32 *view = event ? event_views : reg == SGX_EVENT_STATUS2 ? status2_views : &reg;
	unsigned int views = event ? ARRAY_SIZE(event_views) :
			     reg == SGX_EVENT_STATUS2 ? ARRAY_SIZE(status2_views) : 1;
	unsigned int diag = event ? views : 0;
	u32 slot = mask & SGX_EVENT_TA_FINISHED ? SGX_HC_DIAG_TA : SGX_HC_DIAG_EVENT;
	u32 tail_len = WAIT_TIMEOUT_TAIL + diag * WAIT_DIAG_LEN;
	u32 loop, tail, check;
	unsigned int i;

	if (!max_polls) {
		a->err = -EINVAL;
		return;
	}
	sgx_usse_limm(a, SGX_USSE_ALWAYS, R(18), mask);
	sgx_usse_limm(a, SGX_USSE_ALWAYS, R(19), value);
	sgx_usse_limm(a, SGX_USSE_ALWAYS, R(20), 0xffffffffu);
	sgx_usse_limm(a, SGX_USSE_ALWAYS, R(21), max_polls);
	loop = sgx_usse_here(a);
	for (i = 0; i < views; i++) {
		check = a->len;
		sgx_usse_ldr(a, R(17), view[i], 0);
		sgx_usse_wdf(a, 0);
		sgx_usse_bitop(a, SGX_USSE_ALWAYS, SGX_USSE_AND, R(17), R(17), R(18), false);
		sgx_usse_test(a, SGX_USSE_ALWAYS, 0, SGX_USSE_TST_XOR, SGX_USSE_ZERO,
			      R(17), R(19));
		/* met: past the other checks and the timeout tail */
		sgx_usse_ba_index(a, SGX_USSE_P0, sgx_usse_here(a) + 1 +
				  (views - 1 - i) * WAIT_CHECK_LEN + tail_len);
		if (!a->err && a->len - check != WAIT_CHECK_LEN)
			a->err = -EINVAL;
	}
	tail = a->len;
	/* r21 -= 1: src0 * lo16(#1) + 0xffffffff */
	sgx_usse_imad2(a, SGX_USSE_ALWAYS, R(21), R(21), IMM(1), R(20));
	sgx_usse_test(a, SGX_USSE_ALWAYS, 1, SGX_USSE_TST_AND, SGX_USSE_ZERO,
		      R(21), R(21));
	sgx_usse_ba_index(a, SGX_USSE_NOT_P1, loop);
	/* timed out: record which register, back to the microkernel */
	sgx_usse_limm(a, SGX_USSE_ALWAYS, R(16), SGX_MK_ERR_WAIT_TIMEOUT | reg);
	sgx_usse_st(a, SGX_USSE_ALWAYS, HC, IMM(SGX_HC_ERROR), R(16));
	wait_diag(a, slot, diag);
	sgx_usse_idf(a, 0);
	sgx_usse_wdf(a, 0);
	sgx_usse_ret(a, SGX_USSE_ALWAYS);
	if (!a->err && a->len - tail != tail_len)
		a->err = -EINVAL;	/* the tail and its length went out of step */
	/* met */
	wait_diag(a, slot, diag);
}

void sgx_mk_job_task(struct sgx_mk_job *j, u32 task_word, u32 pds_addr,
		     u32 ir0, u32 sideband)
{
	struct sgx_usse_asm *a = &j->a;
	u32 loopback, wait;

	if (pds_addr & 0xf) {
		a->err = -EINVAL;
		return;
	}
	/* Sony's A 0x0e99..0x0eaa: wait for LOOPBACK, emit, wait for GLOBCOM1 */
	loopback = sgx_usse_here(a);
	sgx_usse_ldr(a, R(19), SGX_LOOPBACK, 1);
	sgx_usse_wdf(a, 1);
	sgx_usse_test(a, SGX_USSE_ALWAYS, 2, SGX_USSE_TST_AND, SGX_USSE_ZERO,
		      R(19), IMM(1));
	sgx_usse_ba_index(a, SGX_USSE_P2, loopback);
	sgx_usse_str(a, SGX_USSE_ALWAYS, SGX_USE_GLOBCOM1_CORE0, IMM(0));
	sgx_usse_limm(a, SGX_USSE_ALWAYS, R(20), ir0);
	sgx_usse_limm(a, SGX_USSE_ALWAYS, R(16), task_word);
	sgx_usse_limm(a, SGX_USSE_ALWAYS, R(18), pds_addr >> 4);
	sgx_usse_emitpds(a, 16, 18, 20, sideband);
	sgx_usse_setm(a, 1);
	wait = sgx_usse_here(a);
	sgx_usse_ldr(a, R(17), SGX_USE_GLOBCOM1_CORE0, 0);
	sgx_usse_wdf(a, 0);
	sgx_usse_test(a, SGX_USSE_ALWAYS, 2, SGX_USSE_TST_AND, SGX_USSE_ZERO,
		      R(17), IMM(1));
	sgx_usse_ba_monitor_index(a, SGX_USSE_P2, wait);
	sgx_usse_str(a, SGX_USSE_ALWAYS, SGX_USE_GLOBCOM1_CORE0, IMM(0));
}

int sgx_mk_job_end(struct sgx_mk_job *j)
{
	struct sgx_usse_asm *a = &j->a;

	sgx_usse_ret(a, SGX_USSE_ALWAYS);
	if (!a->err && a->len > SGX_MK_JOB_CODE_MAX)
		a->err = -E2BIG;
	return a->err;
}
