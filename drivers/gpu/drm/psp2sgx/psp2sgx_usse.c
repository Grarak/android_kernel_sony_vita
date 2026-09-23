// SPDX-License-Identifier: GPL-2.0-only
/*
 * USSE assembler for the SGX543 microkernel; see psp2sgx_usse.h.
 *
 * Field layouts, high word (bits 63..32) and low word (31..0):
 *
 *   VBW      hi: op1 31..27, pred 26..24, dest ext 19, src1 ext 17,
 *                src2 ext 16, invert 11, src2 imm bits 14..15 at 5..4, op2 3,
 *                dest bank 1..0
 *            lo: src1 bank 31..30, src2 bank 29..28, dest 27..21,
 *                src2 imm bits 7..13 at 20..14, src1 13..7, src2 6..0
 *   I32MAD2  hi: 11010, pred 26..24, dest ext 19, src1 ext 17, src2 ext 16,
 *                src0 ext 15, src0 bank 2, dest bank 1..0
 *            lo: as VBW, with src0 at 20..14
 *   VTST     hi: 01001, pred 26..24, src1 ext 17, src2 ext 16, sign test
 *                11..10, zero test 9..8, AND 7, predicate 3..2, dest bank 1..0
 *            lo: src1 bank 31..30, src2 bank 29..28, dest 27..21, write 20,
 *                ALU select 19..18, ALU op 17..14, src1 13..7, src2 6..0
 *   VLDST    hi: 111, load 01 / store 10 in 28..27, pred 26..24, MOE expand
 *                21, src0 ext 18, src1 ext 17, src2 ext 16, dest PA 7,
 *                src0 bank 2, cache bypass 1, DRC 0
 *            lo: src1 bank 31..30, src2 bank 29..28, dest 27..21,
 *                src0 20..14, src1 13..7, src2 6..0
 *   special  op1 11111; category hi 21..20, operation hi 26..24
 */
#include <linux/errno.h>
#include <linux/types.h>

#include "psp2sgx_usse.h"

#define HI(x)	((u64)(u32)(x) << 32)

/* Integer source 1/2 bank and extension bit. */
static bool src12(struct sgx_usse_asm *a, struct sgx_usse_reg r, u32 *bank,
		  u32 *ext, u32 *num, u32 imm_bits)
{
	*ext = 0;
	switch (r.bank) {
	case SGX_USSE_R:
		*bank = 0;
		break;
	case SGX_USSE_O:
		*bank = 1;
		break;
	case SGX_USSE_PA:
		*bank = 2;
		break;
	case SGX_USSE_SA:
		*bank = 3;
		break;
	case SGX_USSE_IMM:
		*bank = 2;
		*ext = 1;
		if (r.n >> imm_bits)
			goto bad;
		*num = r.n;
		return true;
	default:
		goto bad;
	}
	/* r124..r127 alias the internal registers */
	if (r.n > 127 || (r.bank == SGX_USSE_R && r.n >= 124))
		goto bad;
	*num = r.n;
	return true;
bad:
	a->err = -EINVAL;
	return false;
}

static bool dest(struct sgx_usse_asm *a, struct sgx_usse_reg r, u32 *bank,
		 u32 *ext)
{
	*ext = 0;
	switch (r.bank) {
	case SGX_USSE_R:
		*bank = 0;
		break;
	case SGX_USSE_O:
		*bank = 1;
		break;
	case SGX_USSE_PA:
		*bank = 2;
		break;
	case SGX_USSE_SA:
		*bank = 0;
		*ext = 1;
		break;
	default:
		a->err = -EINVAL;
		return false;
	}
	if (r.n > 127 || (r.bank == SGX_USSE_R && r.n >= 124)) {
		a->err = -EINVAL;
		return false;
	}
	return true;
}

/* Source 0 of the three-source forms and the load/store base. */
static bool src0(struct sgx_usse_asm *a, struct sgx_usse_reg r, u32 *bank,
		 u32 *ext)
{
	switch (r.bank) {
	case SGX_USSE_R:
		*bank = 0;
		*ext = 0;
		break;
	case SGX_USSE_PA:
		*bank = 1;
		*ext = 0;
		break;
	case SGX_USSE_O:
		*bank = 0;
		*ext = 1;
		break;
	case SGX_USSE_SA:
		*bank = 1;
		*ext = 1;
		break;
	default:
		a->err = -EINVAL;
		return false;
	}
	if (r.n > 127 || (r.bank == SGX_USSE_R && r.n >= 124)) {
		a->err = -EINVAL;
		return false;
	}
	return true;
}

static void emit(struct sgx_usse_asm *a, u64 word)
{
	if (a->err)
		return;
	if (a->len >= a->cap) {
		a->err = -ENOSPC;
		return;
	}
	a->code[a->len++] = word;
}

void sgx_usse_init(struct sgx_usse_asm *a, u64 *buf, u32 cap, u32 base)
{
	a->code = buf;
	a->cap = cap;
	a->len = 0;
	a->base = base;
	a->err = 0;
	a->nlabels = 0;
	a->nfix = 0;
}

int sgx_usse_label(struct sgx_usse_asm *a)
{
	if (a->nlabels >= SGX_USSE_MAX_LABELS) {
		a->err = -ENOSPC;
		return 0;
	}
	a->label[a->nlabels] = ~0u;
	return a->nlabels++;
}

void sgx_usse_bind(struct sgx_usse_asm *a, int label)
{
	if (label < 0 || (u32)label >= a->nlabels || a->label[label] != ~0u) {
		a->err = -EINVAL;
		return;
	}
	a->label[label] = a->len;
}

u32 sgx_usse_label_index(struct sgx_usse_asm *a, int label)
{
	if (label < 0 || (u32)label >= a->nlabels || a->label[label] == ~0u) {
		a->err = -EINVAL;
		return 0;
	}
	return a->base + a->label[label];
}

static void fixup(struct sgx_usse_asm *a, int label, bool relative)
{
	if (a->nfix >= SGX_USSE_MAX_FIXUPS || label < 0 ||
	    (u32)label >= a->nlabels) {
		a->err = -EINVAL;
		return;
	}
	a->fix[a->nfix].at = a->len;
	a->fix[a->nfix].label = label;
	a->fix[a->nfix].relative = relative;
	a->nfix++;
}

int sgx_usse_finish(struct sgx_usse_asm *a)
{
	u32 n;

	for (n = 0; n < a->nfix && !a->err; n++) {
		u32 at = a->fix[n].at, target = a->label[a->fix[n].label];
		u32 field;

		if (target == ~0u) {
			a->err = -EINVAL;
			break;
		}
		if (a->fix[n].relative)
			field = (target - at) & 0xfffff;
		else
			field = a->base + target;
		if (!a->fix[n].relative && field > 0xfffff) {
			a->err = -ERANGE;
			break;
		}
		a->code[at] = (a->code[at] & ~0xfffffull) | field;
	}
	return a->err;
}

u64 sgx_usse_limm_word(unsigned int pred, unsigned int dst_bank_ext,
		       unsigned int dst_bank, unsigned int dst, u32 value)
{
	u32 hi = 0xfc200000u | dst_bank_ext << 19 |
		 ((value >> 26) & 0x3f) << 12 | (pred & 7) << 9 |
		 ((value >> 21) & 0x1f) << 4 | (dst_bank & 3);
	u32 lo = (dst & 0x7f) << 21 | (value & 0x1fffff);

	return HI(hi) | lo;
}

void sgx_usse_limm(struct sgx_usse_asm *a, enum sgx_usse_pred p,
		   struct sgx_usse_reg dst, u32 value)
{
	u32 db, dx;

	if (!dest(a, dst, &db, &dx))
		return;
	emit(a, sgx_usse_limm_word(p, dx, db, dst.n, value));
}

void sgx_usse_bitop(struct sgx_usse_asm *a, enum sgx_usse_pred p,
		    enum sgx_usse_bitop op, struct sgx_usse_reg dst,
		    struct sgx_usse_reg s1, struct sgx_usse_reg s2, bool invert)
{
	static const u8 op1[] = {
		[SGX_USSE_AND] = 0x0a, [SGX_USSE_OR] = 0x0a, [SGX_USSE_XOR] = 0x0b,
		[SGX_USSE_SHL] = 0x0c, [SGX_USSE_SHR] = 0x0d, [SGX_USSE_ASR] = 0x0d,
	};
	static const u8 op2[] = {
		[SGX_USSE_OR] = 1, [SGX_USSE_ASR] = 1,
	};
	u32 db, dx, b1, x1, n1, b2, x2, n2, hi, lo;

	if ((unsigned int)op > SGX_USSE_ASR) {
		a->err = -EINVAL;
		return;
	}
	if (!dest(a, dst, &db, &dx) || !src12(a, s1, &b1, &x1, &n1, 7) ||
	    !src12(a, s2, &b2, &x2, &n2, 16))
		return;
	hi = (u32)op1[op] << 27 | (p & 7) << 24 | dx << 19 | x1 << 17 |
	     x2 << 16 | (invert ? 1u << 11 : 0) | (u32)op2[op] << 3 | db;
	lo = b1 << 30 | b2 << 28 | dst.n << 21 | n1 << 7;
	if (x2) {
		/* a 16-bit immediate spreads over three fields */
		hi |= ((n2 >> 14) & 3) << 4;
		lo |= ((n2 >> 7) & 0x7f) << 14 | (n2 & 0x7f);
	} else {
		lo |= n2;
	}
	emit(a, HI(hi) | lo);
}

void sgx_usse_mov(struct sgx_usse_asm *a, enum sgx_usse_pred p,
		  struct sgx_usse_reg dst, struct sgx_usse_reg src)
{
	sgx_usse_bitop(a, p, SGX_USSE_OR, dst, src, USSE_IMM(0), false);
}

void sgx_usse_imad2(struct sgx_usse_asm *a, enum sgx_usse_pred p,
		    struct sgx_usse_reg dst, struct sgx_usse_reg s0,
		    struct sgx_usse_reg s1, struct sgx_usse_reg s2)
{
	u32 db, dx, b0, x0, b1, x1, n1, b2, x2, n2, hi, lo;

	if (!dest(a, dst, &db, &dx) || !src0(a, s0, &b0, &x0) ||
	    !src12(a, s1, &b1, &x1, &n1, 7) || !src12(a, s2, &b2, &x2, &n2, 7))
		return;
	hi = 0xd0000000u | (p & 7) << 24 | dx << 19 | x1 << 17 | x2 << 16 |
	     x0 << 15 | b0 << 2 | db;
	lo = b1 << 30 | b2 << 28 | dst.n << 21 | s0.n << 14 | n1 << 7 | n2;
	emit(a, HI(hi) | lo);
}

void sgx_usse_add(struct sgx_usse_asm *a, enum sgx_usse_pred p,
		  struct sgx_usse_reg dst, struct sgx_usse_reg s0,
		  struct sgx_usse_reg s2)
{
	sgx_usse_imad2(a, p, dst, s0, USSE_IMM(1), s2);
}

void sgx_usse_test(struct sgx_usse_asm *a, enum sgx_usse_pred p,
		   unsigned int pdst, enum sgx_usse_tstop op,
		   enum sgx_usse_cond cond, struct sgx_usse_reg s1,
		   struct sgx_usse_reg s2)
{
	/* sign test (none, negative, positive), zero test (none, zero, nonzero), AND */
	static const u8 tests[][3] = {
		[SGX_USSE_ZERO] = { 0, 1, 1 },
		[SGX_USSE_NONZERO] = { 0, 2, 1 },
		[SGX_USSE_NEG] = { 1, 0, 1 },
		[SGX_USSE_POS_OR_ZERO] = { 2, 0, 1 },
		[SGX_USSE_NEG_OR_ZERO] = { 1, 1, 0 },
		[SGX_USSE_POS_NONZERO] = { 2, 2, 1 },
	};
	static const u8 alu[][2] = {
		[SGX_USSE_TST_AND] = { 3, 0 },
		[SGX_USSE_TST_XOR] = { 3, 2 },
		[SGX_USSE_TST_SUB_S32] = { 1, 14 },
		[SGX_USSE_TST_SUB_U32] = { 1, 15 },
	};
	u32 b1, x1, n1, b2, x2, n2, hi, lo;

	if (pdst > 3 || (unsigned int)cond > SGX_USSE_POS_NONZERO ||
	    (unsigned int)op > SGX_USSE_TST_SUB_U32) {
		a->err = -EINVAL;
		return;
	}
	if (!src12(a, s1, &b1, &x1, &n1, 7) || !src12(a, s2, &b2, &x2, &n2, 7))
		return;
	/*
	 * No register result (write-back enable, lo bit 20, clear) and the
	 * destination set to the special bank's sp0 (dest ext bit 19, bank 1),
	 * the form of every test in Sony's microkernel.
	 */
	hi = 0x48080001u | (p & 7) << 24 | x1 << 17 | x2 << 16 |
	     (u32)tests[cond][0] << 10 | (u32)tests[cond][1] << 8 |
	     (u32)tests[cond][2] << 7 | pdst << 2;
	lo = b1 << 30 | b2 << 28 | (u32)alu[op][0] << 18 | (u32)alu[op][1] << 14 |
	     n1 << 7 | n2;
	emit(a, HI(hi) | lo);
}

static void ldst(struct sgx_usse_asm *a, bool load, enum sgx_usse_pred p,
		 struct sgx_usse_reg dst, struct sgx_usse_reg base,
		 struct sgx_usse_reg off, struct sgx_usse_reg val, unsigned int drc)
{
	u32 b0, x0, b1, x1, n1, b2, x2, n2, hi, lo, dpa = 0;

	if (drc > 1) {
		a->err = -EINVAL;
		return;
	}
	if (!src0(a, base, &b0, &x0) || !src12(a, off, &b1, &x1, &n1, 7) ||
	    !src12(a, val, &b2, &x2, &n2, 7))
		return;
	if (load) {
		if (dst.bank == SGX_USSE_PA)
			dpa = 1;
		else if (dst.bank != SGX_USSE_R || dst.n >= 124)
			a->err = -EINVAL;
	}
	/* MOE expand (single access), cache bypass bit as every firmware access sets it */
	hi = (load ? 0xe8000000u : 0xf0000000u) | (p & 7) << 24 | 1u << 21 |
	     x0 << 18 | x1 << 17 | x2 << 16 | dpa << 7 | b0 << 2 | 1u << 1 | drc;
	lo = b1 << 30 | b2 << 28 | (load ? dst.n : 0) << 21 | base.n << 14 |
	     n1 << 7 | n2;
	emit(a, HI(hi) | lo);
}

void sgx_usse_ld(struct sgx_usse_asm *a, enum sgx_usse_pred p,
		 struct sgx_usse_reg dst, struct sgx_usse_reg base,
		 struct sgx_usse_reg offset, unsigned int drc)
{
	ldst(a, true, p, dst, base, offset, USSE_IMM(0), drc);
}

void sgx_usse_st(struct sgx_usse_asm *a, enum sgx_usse_pred p,
		 struct sgx_usse_reg base, struct sgx_usse_reg offset,
		 struct sgx_usse_reg value)
{
	ldst(a, false, p, USSE_R(0), base, offset, value, 0);
}

void sgx_usse_ldr(struct sgx_usse_asm *a, struct sgx_usse_reg dst, u32 reg,
		  unsigned int drc)
{
	u32 index = reg >> 2, hi, lo;

	if ((reg & 3) || index >> 18 || drc > 3 ||
	    (dst.bank != SGX_USSE_R && dst.bank != SGX_USSE_PA) || dst.n > 127 ||
	    (dst.bank == SGX_USSE_R && dst.n >= 124)) {
		a->err = -EINVAL;
		return;
	}
	hi = 0xfe210000u | ((index >> 14) & 0xf) << 2 |
	     (dst.bank == SGX_USSE_PA ? 1u << 7 : 0) | drc;
	lo = 2u << 28 | dst.n << 21 | ((index >> 7) & 0x7f) << 14 | (index & 0x7f);
	emit(a, HI(hi) | lo);
}

/* The short predicate of STR: always, p0, p1, !p0. */
static bool short_pred(struct sgx_usse_asm *a, enum sgx_usse_pred p, u32 *out)
{
	switch (p) {
	case SGX_USSE_ALWAYS:
		*out = 0;
		return true;
	case SGX_USSE_P0:
		*out = 1;
		return true;
	case SGX_USSE_P1:
		*out = 2;
		return true;
	case SGX_USSE_NOT_P0:
		*out = 3;
		return true;
	default:
		a->err = -EINVAL;
		return false;
	}
}

void sgx_usse_str(struct sgx_usse_asm *a, enum sgx_usse_pred p, u32 reg,
		  struct sgx_usse_reg src)
{
	u32 index = reg >> 2, sp, b1, x1, n1, hi, lo;

	if ((reg & 3) || index >> 18) {
		a->err = -EINVAL;
		return;
	}
	if (!short_pred(a, p, &sp) || !src12(a, src, &b1, &x1, &n1, 7))
		return;
	hi = 0xfe290000u | x1 << 17 | sp << 9 | ((index >> 14) & 0xf) << 2;
	lo = b1 << 30 | 2u << 28 | ((index >> 7) & 0x7f) << 14 | n1 << 7 |
	     (index & 0x7f);
	emit(a, HI(hi) | lo);
}

void sgx_usse_str_indirect(struct sgx_usse_asm *a, struct sgx_usse_reg index,
			   struct sgx_usse_reg src)
{
	u32 b1, x1, n1, b2, x2, n2;

	if (!src12(a, src, &b1, &x1, &n1, 7) ||
	    !src12(a, index, &b2, &x2, &n2, 0))
		return;
	if (x2) {
		a->err = -EINVAL;
		return;
	}
	emit(a, HI(0xfe280000u | x1 << 17) | b1 << 30 | b2 << 28 | n1 << 7 | n2);
}

static void branch(struct sgx_usse_asm *a, enum sgx_usse_pred p, int label,
		   bool link, bool monitor)
{
	u32 hi = 0xf8000000u | (p & 7) << 24 | (monitor ? 1u << 10 : 0) |
		 (link ? 1u << 9 : 0);

	fixup(a, label, false);
	emit(a, HI(hi));
}

void sgx_usse_ba(struct sgx_usse_asm *a, enum sgx_usse_pred p, int label)
{
	branch(a, p, label, false, false);
}

void sgx_usse_call(struct sgx_usse_asm *a, enum sgx_usse_pred p, int label)
{
	branch(a, p, label, true, false);
}

void sgx_usse_ba_monitor(struct sgx_usse_asm *a, enum sgx_usse_pred p,
			 int label)
{
	branch(a, p, label, false, true);
}

static void branch_index(struct sgx_usse_asm *a, enum sgx_usse_pred p,
			 u32 index, bool link, bool monitor)
{
	if (index > 0xfffff) {
		a->err = -ERANGE;
		return;
	}
	emit(a, HI(0xf8000000u | (p & 7) << 24 | (monitor ? 1u << 10 : 0) |
		   (link ? 1u << 9 : 0)) | index);
}

void sgx_usse_ba_index(struct sgx_usse_asm *a, enum sgx_usse_pred p, u32 index)
{
	branch_index(a, p, index, false, false);
}

void sgx_usse_call_index(struct sgx_usse_asm *a, enum sgx_usse_pred p,
			 u32 index)
{
	branch_index(a, p, index, true, false);
}

void sgx_usse_ba_monitor_index(struct sgx_usse_asm *a, enum sgx_usse_pred p,
			       u32 index)
{
	branch_index(a, p, index, false, true);
}

void sgx_usse_ret(struct sgx_usse_asm *a, enum sgx_usse_pred p)
{
	emit(a, HI(0xf8000080u | (p & 7) << 24));
}

void sgx_usse_setl(struct sgx_usse_asm *a, struct sgx_usse_reg src)
{
	if (src.bank != SGX_USSE_R || src.n >= 124) {
		a->err = -EINVAL;
		return;
	}
	emit(a, HI(0xf80000c0u) | src.n << 7);
}

void sgx_usse_savl(struct sgx_usse_asm *a, struct sgx_usse_reg dst)
{
	if (dst.bank != SGX_USSE_R || dst.n >= 124) {
		a->err = -EINVAL;
		return;
	}
	emit(a, HI(0xf8000100u) | dst.n << 21);
}

void sgx_usse_nop(struct sgx_usse_asm *a)
{
	emit(a, HI(0xf8000140u));
}

void sgx_usse_nop_end(struct sgx_usse_asm *a)
{
	emit(a, HI(0xf8040140u));
}

void sgx_usse_phas_end(struct sgx_usse_asm *a)
{
	emit(a, HI(0xfa440700u));
}

void sgx_usse_smlsi(struct sgx_usse_asm *a)
{
	emit(a, HI(0xfa100000u) | 1);
}

void sgx_usse_idf(struct sgx_usse_asm *a, unsigned int drc)
{
	emit(a, HI(0xf8200000u | (drc & 3)));
}

void sgx_usse_wdf(struct sgx_usse_asm *a, unsigned int drc)
{
	emit(a, HI(0xf9200000u | (drc & 3)));
}

void sgx_usse_lock(struct sgx_usse_asm *a, unsigned int mutex)
{
	emit(a, HI(0xfd200000u) | (mutex & 0xf) << 4);
}

void sgx_usse_release(struct sgx_usse_asm *a, unsigned int mutex)
{
	emit(a, HI(0xfd200000u) | (mutex & 0xf) << 4 | 1);
}

void sgx_usse_setm(struct sgx_usse_asm *a, unsigned int monitor)
{
	emit(a, HI(0xfa200000u) | (monitor & 0xf));
}

void sgx_usse_emitpds(struct sgx_usse_asm *a, unsigned int r_task,
		      unsigned int r_pds, unsigned int r_ir0, u32 sideband)
{
	u32 hi, lo;

	/* VEC34 cores count the emit sources in register pairs */
	if ((r_task | r_pds | r_ir0) & 1 || r_task >= 124 || r_pds >= 124 ||
	    r_ir0 >= 124 || sideband >> 14) {
		a->err = -EINVAL;
		return;
	}
	hi = 0xfb200000u | ((sideband >> 12) & 3) << 22 | 2u << 14 | 1u << 13 |
	     1u << 12 | ((sideband >> 6) & 0x3f) << 3;
	lo = (sideband & 0x3f) << 22 | (r_task / 2) << 14 | (r_pds / 2) << 7 |
	     (r_ir0 / 2);
	emit(a, HI(hi) | lo);
}
