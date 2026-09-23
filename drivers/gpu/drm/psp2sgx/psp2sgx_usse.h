/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * A small USSE assembler for the PowerVR SGX543 microkernel.
 *
 * The microkernel is our own USSE code, assembled when the driver probes,
 * with the device virtual addresses it needs written straight into its
 * immediates.  Only the integer, memory, control-register and flow control
 * forms a microkernel needs are here; the floating point forms live in the
 * userspace compiler.
 *
 * Every encoder builds the 64-bit instruction word from its fields.  The
 * field layouts follow Imagination's hwdefs (sgxdefs.h) and the Vita3K
 * decoder patterns; each form is one that Sony's microkernel or a
 * bare-metal probe runs on the hardware, with only register numbers,
 * immediates and the operation changed.  Host tests compare the words with
 * those programs.
 *
 * Register numbers are those the hardware uses for 32-bit words: r0..r127
 * temporaries (the top four alias the internal registers and are not
 * accepted), pa0..pa127, sa0..sa127, o0..o127.
 */
#ifndef PSP2SGX_USSE_H
#define PSP2SGX_USSE_H

#include <linux/types.h>

/*
 * Labels are for small fixed programs (the microkernel); generated code with
 * many branches (job code) branches to computed indexes instead.
 */
#define SGX_USSE_MAX_LABELS	16
#define SGX_USSE_MAX_FIXUPS	32

enum sgx_usse_bank {
	SGX_USSE_R,		/* temporary */
	SGX_USSE_O,		/* output */
	SGX_USSE_PA,		/* primary attribute */
	SGX_USSE_SA,		/* secondary attribute */
	SGX_USSE_IMM,		/* immediate (source only) */
};

struct sgx_usse_reg {
	u8 bank;
	u32 n;			/* register number, or the immediate value */
};

#define USSE_R(x)	((struct sgx_usse_reg){ SGX_USSE_R, (x) })
#define USSE_O(x)	((struct sgx_usse_reg){ SGX_USSE_O, (x) })
#define USSE_PA(x)	((struct sgx_usse_reg){ SGX_USSE_PA, (x) })
#define USSE_SA(x)	((struct sgx_usse_reg){ SGX_USSE_SA, (x) })
#define USSE_IMM(x)	((struct sgx_usse_reg){ SGX_USSE_IMM, (x) })

/* The extended (3-bit) predicate field: EURASIA_USE1_EPRED_*. */
enum sgx_usse_pred {
	SGX_USSE_ALWAYS = 0,
	SGX_USSE_P0 = 1,
	SGX_USSE_P1 = 2,
	SGX_USSE_P2 = 3,
	SGX_USSE_P3 = 4,
	SGX_USSE_NOT_P0 = 5,
	SGX_USSE_NOT_P1 = 6,
};

/* VBW operations: op1 in bits 63..59 and op2 in bit 35. */
enum sgx_usse_bitop {
	SGX_USSE_AND,
	SGX_USSE_OR,
	SGX_USSE_XOR,
	SGX_USSE_SHL,
	SGX_USSE_SHR,
	SGX_USSE_ASR,
};

/*
 * VTST: which ALU operation the test runs.  The result's sign and zero
 * flags are what the test reads.  AND and XOR are the zero tests Sony's
 * microkernel uses throughout.  The 32-bit subtracts are SGXVEC_USE0_TEST_ALUOP
 * _I16_SUB32/SUBU32 of the vec34 hwdefs; the DevKit has run the words, but
 * their predicate results had not reached memory yet, so their semantics are
 * unverified and the microkernel does not use them.
 */
enum sgx_usse_tstop {
	SGX_USSE_TST_AND,	/* alu_sel 3, op 0: src1 & src2 */
	SGX_USSE_TST_XOR,	/* alu_sel 3, op 2: src1 ^ src2 */
	SGX_USSE_TST_SUB_S32,	/* alu_sel 1, op 14: signed src1 - src2 (unverified) */
	SGX_USSE_TST_SUB_U32,	/* alu_sel 1, op 15: unsigned src1 - src2 (unverified) */
};

/* VTST conditions, as sign test / zero test / AND-combination. */
enum sgx_usse_cond {
	SGX_USSE_ZERO,		/* result == 0 */
	SGX_USSE_NONZERO,	/* result != 0 */
	SGX_USSE_NEG,		/* result < 0 */
	SGX_USSE_POS_OR_ZERO,	/* result >= 0 (zero counts as positive) */
	SGX_USSE_NEG_OR_ZERO,	/* result <= 0 */
	SGX_USSE_POS_NONZERO,	/* result > 0 */
};

struct sgx_usse_asm {
	u64 *code;
	u32 len;
	u32 cap;
	/* instruction index of code[0] within its code base window */
	u32 base;
	int err;
	u32 label[SGX_USSE_MAX_LABELS];
	u32 nlabels;
	struct {
		u32 at;
		u16 label;
		u8 relative;
	} fix[SGX_USSE_MAX_FIXUPS];
	u32 nfix;
};

void sgx_usse_init(struct sgx_usse_asm *a, u64 *buf, u32 cap, u32 base);
/* Resolves the branches; returns 0 or the first error. */
int sgx_usse_finish(struct sgx_usse_asm *a);

/* Labels: allocate, then bind to the next instruction. */
int sgx_usse_label(struct sgx_usse_asm *a);
void sgx_usse_bind(struct sgx_usse_asm *a, int label);
/* Instruction index (within the code base window) of a bound label. */
u32 sgx_usse_label_index(struct sgx_usse_asm *a, int label);

/* dst = 32-bit immediate (LIMM). */
void sgx_usse_limm(struct sgx_usse_asm *a, enum sgx_usse_pred p,
		   struct sgx_usse_reg dst, u32 value);
/*
 * dst = src1 op src2 (VBW).  src2 may be an immediate of up to 16 bits;
 * `invert` complements src2 first (so AND with ~imm clears bits).
 */
void sgx_usse_bitop(struct sgx_usse_asm *a, enum sgx_usse_pred p,
		    enum sgx_usse_bitop op, struct sgx_usse_reg dst,
		    struct sgx_usse_reg src1, struct sgx_usse_reg src2,
		    bool invert);
/* dst = src (a VBW OR with immediate 0, the compiler's word copy). */
void sgx_usse_mov(struct sgx_usse_asm *a, enum sgx_usse_pred p,
		  struct sgx_usse_reg dst, struct sgx_usse_reg src);
/*
 * dst = src0 * lo16(src1) + src2 (I32MAD2, unsigned).  src1 and src2 may be
 * 7-bit immediates.  With src1 = #1 this is a full 32-bit add.
 */
void sgx_usse_imad2(struct sgx_usse_asm *a, enum sgx_usse_pred p,
		    struct sgx_usse_reg dst, struct sgx_usse_reg src0,
		    struct sgx_usse_reg src1, struct sgx_usse_reg src2);
/* dst = src0 + src2, 32 bits. */
void sgx_usse_add(struct sgx_usse_asm *a, enum sgx_usse_pred p,
		  struct sgx_usse_reg dst, struct sgx_usse_reg src0,
		  struct sgx_usse_reg src2);
/* p<pdst> = cond(src1 op src2) (VTST, no register result). */
void sgx_usse_test(struct sgx_usse_asm *a, enum sgx_usse_pred p,
		   unsigned int pdst, enum sgx_usse_tstop op,
		   enum sgx_usse_cond cond, struct sgx_usse_reg src1,
		   struct sgx_usse_reg src2);

/*
 * dst = the 32-bit word at base + 4 * offset (LDA32, single load).  base is
 * a temporary, primary or secondary attribute or output; offset a register
 * or a 7-bit immediate in words.  The value is valid after sgx_usse_wdf(drc).
 */
void sgx_usse_ld(struct sgx_usse_asm *a, enum sgx_usse_pred p,
		 struct sgx_usse_reg dst, struct sgx_usse_reg base,
		 struct sgx_usse_reg offset, unsigned int drc);
/* The word at base + 4 * offset = value (STA32).  value may be a 7-bit immediate. */
void sgx_usse_st(struct sgx_usse_asm *a, enum sgx_usse_pred p,
		 struct sgx_usse_reg base, struct sgx_usse_reg offset,
		 struct sgx_usse_reg value);

/* dst = SGX register at byte offset `reg` (LDR); valid after sgx_usse_wdf(drc). */
void sgx_usse_ldr(struct sgx_usse_asm *a, struct sgx_usse_reg dst, u32 reg,
		  unsigned int drc);
/*
 * SGX register at byte offset `reg` = src (STR).  The predicate is the
 * short form: always, p0, p1 or !p0.
 */
void sgx_usse_str(struct sgx_usse_asm *a, enum sgx_usse_pred p, u32 reg,
		  struct sgx_usse_reg src);
/* SGX register at byte offset (4 * index register) = src. */
void sgx_usse_str_indirect(struct sgx_usse_asm *a, struct sgx_usse_reg index,
			   struct sgx_usse_reg src);

/* Branches to a label: absolute (BA), optionally saving the return in the link register. */
void sgx_usse_ba(struct sgx_usse_asm *a, enum sgx_usse_pred p, int label);
void sgx_usse_call(struct sgx_usse_asm *a, enum sgx_usse_pred p, int label);
/* The code base window index the next instruction will have. */
static inline u32 sgx_usse_here(const struct sgx_usse_asm *a)
{
	return a->base + a->len;
}

/* Branches to a fixed instruction index of the code base window. */
void sgx_usse_ba_index(struct sgx_usse_asm *a, enum sgx_usse_pred p, u32 index);
void sgx_usse_call_index(struct sgx_usse_asm *a, enum sgx_usse_pred p, u32 index);
void sgx_usse_ba_monitor_index(struct sgx_usse_asm *a, enum sgx_usse_pred p,
			       u32 index);
/* The polling branch Sony's microkernel uses while it waits on a monitor. */
void sgx_usse_ba_monitor(struct sgx_usse_asm *a, enum sgx_usse_pred p, int label);
/* Return through the link register (LAPC). */
void sgx_usse_ret(struct sgx_usse_asm *a, enum sgx_usse_pred p);
void sgx_usse_setl(struct sgx_usse_asm *a, struct sgx_usse_reg src);
void sgx_usse_savl(struct sgx_usse_asm *a, struct sgx_usse_reg dst);

void sgx_usse_nop(struct sgx_usse_asm *a);
/* NOP with END: ends the thread. */
void sgx_usse_nop_end(struct sgx_usse_asm *a);
/* The end-of-phases marker every program starts with (PHAS, END). */
void sgx_usse_phas_end(struct sgx_usse_asm *a);
/* SMLSI 0, 0, 0, 1: unit increments for repeats, as every microkernel thread sets. */
void sgx_usse_smlsi(struct sgx_usse_asm *a);
/* Issue / wait for a data fence on dependent read counter drc (0..3). */
void sgx_usse_idf(struct sgx_usse_asm *a, unsigned int drc);
void sgx_usse_wdf(struct sgx_usse_asm *a, unsigned int drc);
void sgx_usse_lock(struct sgx_usse_asm *a, unsigned int mutex);
void sgx_usse_release(struct sgx_usse_asm *a, unsigned int mutex);
void sgx_usse_setm(struct sgx_usse_asm *a, unsigned int monitor);
/*
 * Hand a task to the PDS (EMITPDS.tasks.taske): task control word, PDS
 * program address >> 4 and the task's IR0, each in an even temporary, plus
 * the 14-bit PDS sideband (data master, pipe, instances, ...).
 */
void sgx_usse_emitpds(struct sgx_usse_asm *a, unsigned int r_task,
		      unsigned int r_pds, unsigned int r_ir0, u32 sideband);

/* Encoding helpers shared with the tests. */
u64 sgx_usse_limm_word(unsigned int pred, unsigned int dst_bank_ext,
		       unsigned int dst_bank, unsigned int dst, u32 value);

#endif
