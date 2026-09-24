/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * PowerVR SGX543MP4+ register offsets used by psp2sgx.
 *
 * Offsets are from the start of the SGX register block (PA 0xE8400000 on the
 * PS Vita, 0x20000 bytes).  Bank 0 is at 0x0000, the multi-core master at
 * 0x4000 and core n's own bank at 0x8000 + n * 0x4000.  Names follow the
 * EUR_CR_* registers of Imagination's hwdefs (sgx543defs.h, sgxmpdefs.h,
 * sgx544defs.h); the values the driver writes and why are in psp2sgx_hw.c.
 */
#ifndef PSP2SGX_REGS_H
#define PSP2SGX_REGS_H

#include <linux/bits.h>

#define SGX_REG_SIZE			0x20000
#define SGX_MASTER			0x4000
#define SGX_CORE_BANK(n)		(0x8000 + (n) * 0x4000)

/* bank 0 */
#define SGX_CLKGATECTL			0x0000
#define SGX_CLKGATECTL2			0x0004
#define SGX_CLKGATESTATUS		0x0008
/* a core's tiling units: TE, VDM and TA clocks, off once its TA is done */
#define SGX_TA_CLK_GATE			0x0310	/* bit 0: START_TO_END */
#define SGX_CLKGATESTATUS_TA_UNITS	(BIT(3) | BIT(6) | BIT(20))
#define SGX_CLKGATESTATUS_3D_UNITS	(BIT(1) | BIT(2) | BIT(7) | BIT(8))	/* ISP2 TSP PDS PBE */
/* the pipeline: ISP2 TSP TE MTE DPM VDM PDS PBE IDXFIFO TA (sgx543defs.h);
 * ISP and the USSE pipes read 1 on an idle DevKit core (0x01008801) */
#define SGX_CLKGATESTATUS_PIPELINE	0x001801feu
#define SGX_BIF_MEM_REQ_STAT		0x0ca8	/* bits 7..0: reads in flight */
#define  SGX_CLKGATECTL2_AUTO		0x0a8a8aaa	/* SGXStart's value */
#define  SGX_CLKGATECTL2_USE_ON		0x0a8a4a6a	/* USE0/USE1 clocks on, Sony's during a job */
#define SGX_CORE_ID			0x0020
#define SGX_CORE_REVISION		0x0024
#define SGX_SOFT_RESET			0x0080
#define SGX_SOFT_RESET_ISP2		BIT(6)
#define SGX_VDM_BATCH_NUM		0x0320
#define SGX_MASTER_VDM_BATCH_NUM	0x4320
#define SGX_EVENT_HOST_ENABLE2		0x0110
#define SGX_EVENT_HOST_CLEAR2		0x0114
#define SGX_EVENT_STATUS2		0x0118
#define SGX_EVENT_STATUS		0x012c
#define SGX_EVENT_HOST_ENABLE		0x0130
#define SGX_EVENT_HOST_CLEAR		0x0134
#define SGX_PDS_CACHE_STATUS		0x0138
#define SGX_PDS_CACHE_HOST_ENABLE	0x013c
#define SGX_PDS_CACHE_HOST_CLEAR	0x0140

#define SGX_EVENT_TPC_FLUSH		BIT(10)
#define SGX_EVENT_TA_FINISHED		BIT(13)
#define SGX_EVENT_SW_EVENT		BIT(14)
#define SGX_EVENT_PIXELBE_END_RENDER	BIT(18)
#define SGX_EVENT_TIMER			BIT(29)

#define SGX_PDS_CACHE_CSC_INV		BIT(0)
#define SGX_PDS_CACHE_DSC_INV0		BIT(1)
#define SGX_PDS_CACHE_DSC_INV1		BIT(2)

#define SGX_VDM_START			0x0200
#define SGX_TSP_PARAMETER_CACHE		0x043c
#define SGX_VDM_CTRL_STREAM_BASE	0x0238
#define SGX_TE_RGNHDR_INIT		0x0380
#define SGX_ISP_START_RENDER		0x0428
#define SGX_TSP_CONFIG			0x0800
#define SGX_TCU_CTRL			0x0804
#define SGX_TCU_ICTRL			0x0808
#define SGX_DCU_ICTRL			0x0810
#define SGX_DCU_PCTRL			0x0814
#define SGX_DCU_CTRL			0x0818
#define SGX_USE_CACHE			0x0a08
#define SGX_USE_CODE_BASE(n)		(0x0a0c + (n) * 4)
#define SGX_USE_CODE_BASE_DM_SHIFT	26
#define SGX_EVENT_PDS_ENABLE		0x0a58
#define SGX_EVENT_PIXEL_PDS_EXEC	0x0a5c
#define SGX_EVENT_PIXEL_PDS_DATA	0x0a60
#define SGX_EVENT_PIXEL_PDS_INFO	0x0a64
#define SGX_EVENT_OTHER_PDS_EXEC	0x0a68
#define SGX_EVENT_OTHER_PDS_DATA	0x0a6c
#define SGX_EVENT_OTHER_PDS_INFO	0x0a70
#define SGX_DMS_CTRL			0x0a74
#define  SGX_DMS_CTRL_JOB		0x0e208200	/* 3 vertex, 8 pixel partitions; 32 tasks each */
#define SGX_EVENT_KICK2			0x0ac0
#define SGX_EVENT_KICKER		0x0ac4
#define SGX_EVENT_KICK			0x0ac8
#define SGX_EVENT_TIMER_REG		0x0acc
#define SGX_PDS_INV0			0x0ad0
#define SGX_PDS_INV1			0x0ad4
#define SGX_PDS_INV_CSC			0x0ae0
#define SGX_LOOPBACK			0x0b04
#define SGX_USE_GLOBCOM1		0x0b44

#define SGX_BIF_CTRL			0x0c00
#define SGX_BIF_INT_STAT		0x0c04
#define SGX_BIF_FAULT			0x0c08
#define SGX_BIF_CTRL_INVAL		0x0c34
#define SGX_BIF_CTRL_INVAL_PTE		BIT(2)
#define SGX_BIF_CTRL_INVAL_ALL		BIT(3)
#define SGX_BIF_DIR_LIST_BASE(n)	((n) ? 0x0c38 + ((n) - 1) * 4 : 0x0c84)
#define SGX_BIF_BANK_SET		0x0c74
#define SGX_BIF_BANK0			0x0c78
#define SGX_BIF_TA_REQ_BASE		0x0c90
#define SGX_BIF_3D_REQ_BASE		0x0cac
#define SGX_BIF_ZLS_REQ_BASE		0x0cb0

/* master bank */
#define SGX_MASTER_CORE			0x4000
#define SGX_MASTER_CLKGATECTL		0x4004
#define SGX_MASTER_CLKGATECTL2		0x4020
#define SGX_MASTER_EVENT_STATUS		0x412c
#define SGX_MASTER_EVENT_HOST_CLEAR	0x4134
#define SGX_MASTER_SOFT_RESET		0x4080
#define SGX_MASTER_BIF_CTRL		0x4c00
#define SGX_MASTER_SLC_CTRL		0x4d00
#define SGX_MASTER_SLC_EVENT_STATUS	0x4d10
#define SGX_MASTER_SLC_EVENT_CLEAR	0x4d14
#define SGX_MASTER_SLC_CTRL_INVAL	0x4d28
#define SGX_MASTER_SLC_CTRL_FLUSH	0x4d2c
#define SGX_SLC_FLUSH_ALL		0x10
#define SGX_SLC_INVAL_ALL		0x1
#define SGX_SLC_EVENT_INVAL		BIT(0)
#define SGX_SLC_EVENT_FLUSH		BIT(1)

/* SGX MMU (BIF) page tables: two levels, 4 KiB pages, 1024 entries each */
#define SGX_MMU_PAGE_SHIFT		12
#define SGX_MMU_PAGE_SIZE		(1u << SGX_MMU_PAGE_SHIFT)
#define SGX_MMU_PDE_SHIFT		22
#define SGX_MMU_PDE_SPAN		(1u << SGX_MMU_PDE_SHIFT)
#define SGX_MMU_ENTRIES			1024
#define SGX_MMU_PDE_VALID		BIT(0)
#define SGX_MMU_PTE_VALID		BIT(0)
#define SGX_MMU_PTE_WRITEONLY		BIT(1)
#define SGX_MMU_PTE_READONLY		BIT(2)
#define SGX_MMU_PTE_CACHECONSISTENT	BIT(3)
#define SGX_MMU_PTE_EDMPROTECT		BIT(4)

#endif
