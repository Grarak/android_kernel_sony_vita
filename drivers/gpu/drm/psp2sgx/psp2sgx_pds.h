/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * PDS programs for the psp2sgx microkernel.
 *
 * A PDS program in device memory is its data segment followed by its code.
 * The data segment is made of 16-byte units; each unit holds two words of
 * DS0 and then two words of DS1, so DS0 word n sits at byte
 * (n / 2) * 16 + (n % 2) * 4 and DS1 word n eight bytes further.  The code is
 * a sequence of 32-bit instructions.  The size registers and the emitpds task
 * word count the data segment in units.
 *
 * Only the MOVS forms the driver needs are here: DOUTA (a word into a primary
 * attribute of the task), DOUTU (start the USSE task) and HALT.  The layout
 * follows the hwdefs' EURASIA_PDS_MOVS_* fields with the extended sources of
 * the SGX543; every word a builder makes matches one of the event programs
 * of Sony's gpuinit_es4, which a bare-metal probe ran on the DevKit.
 */
#ifndef PSP2SGX_PDS_H
#define PSP2SGX_PDS_H

#include <linux/types.h>

/* MOVS destinations */
#define SGX_PDS_DOUTD		3
#define SGX_PDS_DOUTU		5
#define SGX_PDS_DOUTA		6

/* MOVS source 1 is a DS0 word pair or an input register, source 2 a DS1 pair or one */
#define SGX_PDS_SRC_DS		0
#define SGX_PDS_SRC_REG		1
#define SGX_PDS_REG_IR0		0
#define SGX_PDS_REG_IR1		1

/* Swizzle: which 32-bit half of which source an output word takes */
#define SGX_PDS_SRC1L		0
#define SGX_PDS_SRC1H		1
#define SGX_PDS_SRC2L		2
#define SGX_PDS_SRC2H		3

#define SGX_PDS_HALT		0xaf000000u	/* halt, always */
#define SGX_PDS_UNIT_BYTES	16

/*
 * DOUTU word 0: the USSE code base (0..15) holding the program, its
 * instruction index in that window, and the task's temporaries in units of
 * four registers.
 */
static inline u32 sgx_pds_doutu0(unsigned int code_base, u32 index,
				 unsigned int temp_units)
{
	return (code_base & 0xf) | (index & 0xfffff) << 4 | (temp_units & 0xff) << 24;
}

/* MOVS, condition always. src1/src2 index a word pair (ds0[2 * n], ds0[2 * n + 1]). */
u32 sgx_pds_movs(unsigned int dest, unsigned int src1_sel, unsigned int src1,
		 unsigned int src2_sel, unsigned int src2,
		 unsigned int swiz0, unsigned int swiz1, unsigned int swiz2,
		 unsigned int swiz3);

/* Places DS0 / DS1 word n of a data segment. */
void sgx_pds_put_ds0(u32 *prog, unsigned int n, u32 value);
void sgx_pds_put_ds1(u32 *prog, unsigned int n, u32 value);

/*
 * The event program the host kick runs: pa0 = IR0, pa1 = IR1, pa2 = `word`,
 * then start the USSE task `doutu0` once per instance.  Writes
 * SGX_PDS_EVENT_WORDS words to prog; the data segment is SGX_PDS_EVENT_UNITS.
 */
#define SGX_PDS_EVENT_UNITS	3
#define SGX_PDS_EVENT_WORDS	(SGX_PDS_EVENT_UNITS * 4 + 7)
void sgx_pds_event_program(u32 *prog, u32 word, u32 doutu0);

/*
 * A program that only starts a USSE task (DOUTU words 0..2).  Writes
 * SGX_PDS_DOUTU_WORDS words; the data segment is SGX_PDS_DOUTU_UNITS.
 */
#define SGX_PDS_DOUTU_UNITS	1
#define SGX_PDS_DOUTU_WORDS	(SGX_PDS_DOUTU_UNITS * 4 + 2)
void sgx_pds_doutu_program(u32 *prog, u32 doutu0, u32 doutu1, u32 doutu2);

#endif
