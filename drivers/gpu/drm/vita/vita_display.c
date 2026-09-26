// SPDX-License-Identifier: GPL-2.0
/*
 * PlayStation Vita display: the OLED head.
 *
 * The OLED (PCH-1000 and the DevKit) is fed by DSI0 (0xE5050000, a TC358768-like
 * DSI host behind Sony's timing generator) from the IFTU0 compositor (planes a
 * and b at 0xE5020000/0xE5021000, control block 0xE5022000). The panel itself
 * (a Samsung AMS495QA0x) takes its commands as 9-bit DCS frames over SPI2
 * (0xE0A20000); its supply is GPIO0 port 0. The display clocks, resets and the
 * DSI PLL are in Pervasive. Everything here replays the Vita OS 3.65
 * sequences (SceDsi and SceIftu in lowio.skprx, SceDisplay, SceOled), as
 * the modules and register traces of the OS show them:
 *
 *   enable:  BaseClk DSI0 PLL, Pervasive gate +0x80 = 0xF, reset +0x80 &= ~7,
 *            Misc +0x140 = 0, EnableHead (link up in LP-11, the 19-word
 *            sequencer program), IFTU0 planes enabled blank, panel supply on,
 *            120 ms, the panel's configuration over SPI2, StartDisplay
 *            (video), then sleep out, 13 frames, display on, brightness.
 *   disable: display off and sleep in, IFTU disabled and blanked, a blanking
 *            packet, 42 ms, DisableHead, Misc +0x140 = 1, gate off, reset
 *            on, PLL off, panel supply off.
 *
 * Vblank is the DSI's frame interrupt (+0x050 bit 1, GIC ID 213); IFTU banks
 * latch on it. Scanout is IFTU0 plane b over a blank plane a with merge
 * setting 2, as SceShell has it; flips write the plane's idle bank and select
 * it. Buffers come from a CDRAM carve-out (memory-region).
 *
 * If the boot loader left the head running (gate +0x80 set), the driver takes
 * the head down the same way first and brings it up from scratch.
 */

#include <linux/aperture.h>
#include <linux/backlight.h>
#include <linux/bitrev.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>

/* Pervasive (Misc 0xE3100000, Reset +0x1000, Gate +0x2000, BaseClk +0x3000) */
#define MISC_SOC_REV		0x000
#define MISC_DSI0		0x140	/* 0 while the head runs; meaning unknown */
#define PERV_DSI0		0x080	/* reset: 3 bits; gate: 4 bits */
#define PERV_IFTU		0x088
#define PERV_SPI2		0x10c
#define BASECLK_DSI0		0x180	/* PLL block; DSI1's is at +0x100 */

/* DSI */
#define DSI_START		0x000
#define DSI_VTOTAL		0x00c
#define DSI_BUSY		0x048
#define DSI_INT_STATUS		0x050
#define DSI_INT_ENABLE		0x054
#define DSI_INT_VBLANK		BIT(1)
#define DSI_SEQ_STATUS		0x414
#define DSI_SEQ_HS_DONE		BIT(24)
#define DSI_SEQ_FIFO		0x500
#define DSI_SEQ_MODE		0x504
#define DSI_SEQ_COMMIT		0x508
#define DSI_SEQ_ACK		0x50c
#define DSI_SEQ_CLEAR		0x514
#define DSI_SEQ_RESET		0x518
#define DSI_SEQ_RUN		0x51c
#define DSI_PHY_STARTCNTRL	0x804
#define DSI_PHY_TABLE		0x810	/* 9 words */
#define DSI_PHY_LANES		0x834
#define DSI_PHY_CLKCONT		0x838
#define DSI_PHY_83C		0x83c
#define DSI_LANE(n)		(0x940 + 4 * (n))

/* IFTU plane block */
#define IFTU_BANK_LIVE		0x004	/* bit 1: the bank being scanned; bit 0: settled */
#define IFTU_INT_STATUS		0x040
#define IFTU_KICK		0x050
#define IFTU_INT_ENABLE		0x058
#define IFTU_ALPHA		0x08c
#define IFTU_OPAQUE		0x0a0
#define IFTU_PERIOD		0x180
#define IFTU_BANK(n)		(0x200 + 0x100 * (n))
/* in a bank */
#define BANK_PADDR		0x00
#define BANK_FORMAT		0x40
#define BANK_WIDTH		0x44
#define BANK_HEIGHT		0x48
#define BANK_BLANK		0x4c
#define BANK_LEFTOVER		0x54	/* bytes past the width in each row */
#define BANK_OUT_FORMAT		0xa0
#define BANK_OUT_WIDTH		0xa4
#define BANK_OUT_HEIGHT		0xa8
#define BANK_SRC_W		0xc0
#define BANK_SRC_H		0xc4

/* IFTU control block */
#define IFTUC_START		0x00
#define IFTUC_CONTROL		0x04	/* bit 0: plane b merged */
#define IFTUC_SELECT(p)		(0x10 + 8 * (p))
#define IFTUC_MERGE		0x20

#define IFTU_FMT_RGBA		0x10	/* R,G,B,A bytes (the boot loader's A8B8G8R8) */
#define IFTU_FMT_BGRA		0x20	/* (inf.) the wiki's RGBX8888, 0x10's other order */
#define IFTU_OUT_FORMAT		0x2000	/* BGRX8888_MULT */
#define IFTU_MERGE_OFF		0x80

/* SPI2 */
#define SPI_RX			0x00
#define SPI_TX			0x04
#define SPI_MODE		0x08
#define SPI_CONFIG		0x0c
#define SPI_RUN			0x10
#define SPI_DIVIDER		0x14
#define SPI_IRQ_MASK		0x20
#define SPI_RX_LEVEL		0x28
#define SPI_TX_LEVEL		0x2c
#define SPI_TX_FULL		0x7f

#define OLED_WIDTH		960
#define OLED_HEIGHT		544
#define OLED_VTOTAL		594
#define OLED_FRAME_US		16683

enum { PLANE_A, PLANE_B };

struct vita_display {
	struct drm_device drm;
	struct drm_crtc crtc;
	struct drm_plane plane;
	struct drm_encoder encoder;
	struct drm_connector connector;

	void __iomem *dsi, *iftu[2], *iftuc, *spi;
	void __iomem *misc, *reset, *gate, *baseclk;
	struct gpio_desc *power;
	struct backlight_device *backlight;
	int irq;
	u32 soc_rev;

	spinlock_t perv_lock;		/* Pervasive read-modify-writes */
	struct mutex panel_lock;	/* SPI2 transfers and the panel state */

	/* the 9-bit frame packer (SceOled's accumulator) */
	u32 spi_bits;
	unsigned int spi_nbits;

	/* panel: SceOled's DDB-dependent tables */
	u8 ddb[5];
	const u8 *gamma;		/* 17 rows of 21 bytes, brightest first */
	const u8 *elvss;		/* 17 B2 values, or NULL */
	const u8 *init2;		/* the second configuration script */
	bool panel_prepared, panel_on, first_brightness;
	unsigned int row;		/* current gamma row */

	int last_bank[2];		/* SceIftu's last written bank per plane */
};

static inline struct vita_display *to_vd(struct drm_device *drm)
{
	return container_of(drm, struct vita_display, drm);
}

/* ------------------------------------------------------------------ */
/* Pervasive */

static void perv_rmw(struct vita_display *vd, void __iomem *reg, u32 clear, u32 set)
{
	unsigned long flags;

	spin_lock_irqsave(&vd->perv_lock, flags);
	writel((readl(reg) & ~clear) | set, reg);
	readl(reg);
	spin_unlock_irqrestore(&vd->perv_lock, flags);
}

/*
 * kscePervasiveDsiSetPixelClock(0, 2243100) (SceLowio):
 * the OLED's 224.31 MHz D-PHY HS clock.
 */
static void vita_dsi_pll_on(struct vita_display *vd)
{
	void __iomem *pll = vd->baseclk + BASECLK_DSI0;

	writel(1, pll + 0x08);
	writel(1, pll + 0x04);
	writel(0x10, pll + 0x24);
	writel(1, pll + 0x00);
	writel(0x1a, pll + 0x30);
	writel(0xc01, pll + 0x34);
	writel(0x08100301, pll + 0x38);
	writel(0, pll + 0x00);
	usleep_range(1000, 1100);
	writel(0x1001f, pll + 0x40);
	writel(0, pll + 0x44);
	writel(0x04180301, pll + 0x48);
	writel(2, pll + 0x20);
	writel(0, pll + 0x04);
	usleep_range(1000, 1100);
	writel(0, pll + 0x08);
}

static void vita_dsi_pll_off(struct vita_display *vd)
{
	void __iomem *pll = vd->baseclk + BASECLK_DSI0;

	writel(1, pll + 0x08);
	writel(1, pll + 0x04);
	writel(1, pll + 0x00);
}

/* ------------------------------------------------------------------ */
/* DSI0: SceDsi's head 0 with VIC 0, 2 lanes, 24 bpp, panel type 0 */

static const u32 oled_phy_rev0[9] = {
	0x2bcf, 0x0006, 0x0f05, 0x0001, 0x0806, 0x4906, 0x0016, 0x0003, 0x0020,
};

static const u32 oled_phy_rev1[9] = {
	0x2bcf, 0x0006, 0x1007, 0x0001, 0x0808, 0x4906, 0x0016, 0x0003, 0x0020,
};

/* The sequencer program ksceDsiEnableHead(0) writes */
static const u32 oled_program[19] = {
	0x0407ffff, 0xa3000082, 0x28000004, 0x10000001, 0x400b4019,
	0x0402002c, 0x28000001, 0x10000021, 0x400b4019, 0x0403021f,
	0x28000001, 0x10000021, 0x40000219, 0x400b403e, 0x04020003,
	0x28000001, 0x10000021, 0x400b4019, 0x0c000000,
};

/* ksceDsiStartDisplay(0, 0)'s timing registers, in its order (§5.2) */
static const struct { u16 reg; u32 val; } oled_timing[] = {
	{ 0x004, 0x00000000 }, { 0x008, 0x00000c4e }, { 0x00c, 0x00000252 },
	{ 0x010, 0x00010001 }, { 0x014, 0x00d20c13 }, { 0x01c, 0x002e024f },
	{ 0x024, 0x00c60001 }, { 0x02c, 0x00010001 }, { 0x030, 0x00040c4e },
	{ 0x03c, 0x00000001 }, { 0x040, 0x014f0026 }, { 0x05c, 0x00010001 },
	{ 0x060, 0x002e0001 }, { 0x06c, 0x41570001 }, { 0x070, 0x00000000 },
};

/* ksceDsiEnableHead(0): the link comes up in LP-11 */
static void vita_dsi_enable_head(struct vita_display *vd)
{
	const u32 *phy = (vd->soc_rev & 0x1ff00) > 0xff ? oled_phy_rev1 : oled_phy_rev0;
	void __iomem *dsi = vd->dsi;
	int i;

	writel(0, dsi + DSI_INT_ENABLE);
	writel(1, dsi + DSI_SEQ_RESET);
	if (vd->soc_rev & 0x1ff00)
		for (i = 0; i < 3; i++)
			writel((readl(dsi + 0x900 + 4 * i) & ~3) | 2, dsi + 0x900 + 4 * i);
	for (i = 0; i < 3; i++)
		writel(0x200, dsi + DSI_LANE(i));
	for (i = 0; i < 9; i++)
		writel(phy[i], dsi + DSI_PHY_TABLE + 4 * i);
	writel(phy[1] | phy[1] << 16, dsi + DSI_PHY_83C);
	writel(7, dsi + DSI_PHY_LANES);
	writel(0, dsi + DSI_PHY_CLKCONT);
	writel(1, dsi + DSI_PHY_STARTCNTRL);
	writel(0xffffffff, dsi + DSI_SEQ_CLEAR);
	for (i = 0; i < ARRAY_SIZE(oled_program); i++)
		writel(oled_program[i], dsi + DSI_SEQ_FIFO);
	writel(1, dsi + DSI_SEQ_RUN);
}

/* ksceDsiStartDisplay(0, 0): video on, the frame interrupt enabled */
static void vita_dsi_start_display(struct vita_display *vd)
{
	void __iomem *dsi = vd->dsi;
	int i;

	for (i = 0; i < ARRAY_SIZE(oled_timing); i++)
		writel(oled_timing[i].val, dsi + oled_timing[i].reg);
	writel(readl(dsi + DSI_INT_STATUS), dsi + DSI_INT_STATUS);
	writel(DSI_INT_VBLANK, dsi + DSI_INT_ENABLE);
	writel(0, dsi + DSI_PHY_CLKCONT);
	writel(0xffffffff, dsi + DSI_SEQ_COMMIT);
	writel(1, dsi + DSI_START);
}

/* ksceDsiSendBlankingPacket(0): ends video; the sequencer waits are bounded */
static void vita_dsi_send_blanking(struct vita_display *vd)
{
	void __iomem *dsi = vd->dsi;
	u32 v;

	writel(12, dsi + DSI_SEQ_MODE);
	readl(dsi + DSI_SEQ_MODE);
	writel(readl(dsi + DSI_SEQ_STATUS), dsi + DSI_SEQ_ACK);
	if (readl_poll_timeout(dsi + DSI_SEQ_STATUS, v, !(v & DSI_SEQ_HS_DONE), 10, 50000))
		drm_warn(&vd->drm, "DSI sequencer did not go idle (0x%08x)\n", v);
	writel(0xa30000a2, dsi + DSI_SEQ_FIFO);
	writel(0x40000019, dsi + DSI_SEQ_FIFO);
	writel(0x30000039, dsi + DSI_SEQ_FIFO);
	writel(0xa6002000, dsi + DSI_SEQ_FIFO);
	writel(0x18000001, dsi + DSI_SEQ_FIFO);
	writel(0xffffffff, dsi + DSI_SEQ_COMMIT);
	if (readl_poll_timeout(dsi + DSI_SEQ_STATUS, v, v & DSI_SEQ_HS_DONE, 10, 50000))
		drm_warn(&vd->drm, "DSI blanking packet not sent (0x%08x)\n", v);
	writel(0xffffffff, dsi + DSI_SEQ_CLEAR);
	writel(DSI_SEQ_HS_DONE, dsi + DSI_SEQ_ACK);
	writel(12, dsi + DSI_SEQ_MODE);
	readl(dsi + DSI_SEQ_MODE);
	writel(0, dsi + DSI_START);
	readl(dsi + DSI_START);
}

/* ksceDsiDisableHead(0) after its blanking packet */
static void vita_dsi_disable_head(struct vita_display *vd)
{
	void __iomem *dsi = vd->dsi;
	u32 v;
	int i;

	writel(0, dsi + DSI_INT_ENABLE);
	if (readl_poll_timeout(dsi + DSI_BUSY, v, !(v & 0xf), 10, 100000))
		drm_warn(&vd->drm, "DSI still busy (0x%x)\n", v);
	writel(0xffffffff, dsi + DSI_SEQ_CLEAR);
	for (i = 0; i < 3; i++)
		writel(1, dsi + DSI_LANE(i));
	writel(0, dsi + DSI_PHY_LANES);
	writel(1, dsi + DSI_PHY_STARTCNTRL);
	readl(dsi + 0x410);
}

/* ------------------------------------------------------------------ */
/* IFTU0 (SceLowio ksceIftuEnable, SetInputFrameBuffer, SetMergeSetting) */

/* SceDisplay's CSC tables for the OLED head (display.elf 0x810097d0, 0x810099ac) */
static const u32 oled_csc1[15] = {
	0, 0x202, 0x3ff, 0, 0, 0, 0x200, 0, 0x326, 0x200, 0xfa1, 0xf11, 0x200, 0x3b6, 0,
};

static const u32 oled_csc2[15] = {
	0, 0, 0x3ff, 0, 0x3ff, 0, 0x200, 0, 0, 0, 0x200, 0, 0, 0, 0x200,
};

static void vita_iftu_set_alpha(void __iomem *p, u32 alpha)
{
	if (alpha == 0x100) {
		writel(0, p + IFTU_ALPHA);
		writel(1, p + IFTU_OPAQUE);
	} else {
		writel(alpha, p + IFTU_ALPHA);
		writel(0, p + IFTU_OPAQUE);
	}
}

static void vita_iftu_blank_bank(void __iomem *bank)
{
	writel(1, bank + BANK_BLANK);
	writel(IFTU_FMT_RGBA, bank + BANK_FORMAT);
	writel(OLED_WIDTH, bank + BANK_WIDTH);
	writel(OLED_HEIGHT, bank + BANK_HEIGHT);
	writel(0, bank + 0x60);
	writel(0, bank + 0x64);
	writel(0, bank + 0x68);
	writel(0, bank + 0x6c);
	writel(0x10000, bank + BANK_SRC_W);
	writel(0x10000, bank + BANK_SRC_H);
}

/* ksceIftuEnable(plane) with no framebuffer yet: both banks blank */
static void vita_iftu_enable_plane(struct vita_display *vd, int plane)
{
	void __iomem *p = vd->iftu[plane];
	u32 ctl;
	int i, b;

	writel(0x108, p + IFTU_INT_ENABLE);
	writel(1, p + IFTU_PERIOD);
	writel(oled_csc1[0], p + 0x104);
	writel(oled_csc1[1], p + 0x108);
	for (i = 0; i < 9; i++)
		writel(oled_csc1[6 + i], p + 0x10c + 4 * i);
	writel(oled_csc2[0], p + 0x130);
	writel(oled_csc2[1], p + 0x134);
	for (i = 0; i < 9; i++)
		writel(oled_csc2[6 + i], p + 0x138 + 4 * i);
	for (i = 0; i < 4; i++)
		writel(oled_csc2[2 + i], p + 0x15c + 4 * i);
	writel(0, p + 0x80);
	writel(0, p + 0x84);
	writel(0, p + 0x88);
	vita_iftu_set_alpha(p, 0xff);
	for (b = 0; b < 2; b++) {
		void __iomem *bank = p + IFTU_BANK(b);

		writel(IFTU_OUT_FORMAT, bank + BANK_OUT_FORMAT);
		writel(OLED_WIDTH, bank + BANK_OUT_WIDTH);
		writel(OLED_HEIGHT, bank + BANK_OUT_HEIGHT);
		vita_iftu_blank_bank(bank);
	}

	ctl = readl(vd->iftuc + IFTUC_CONTROL);
	if (plane == PLANE_B) {
		/* merge setting 0x80 (off) until SetMergeSetting */
		writel(0, vd->iftuc + IFTUC_MERGE);
		ctl &= ~BIT(0);
		ctl &= ~BIT(4);
	} else {
		ctl &= ~BIT(2);
	}
	vd->last_bank[plane] = 0;
	writel(0, vd->iftuc + IFTUC_SELECT(plane));
	writel(ctl, vd->iftuc + IFTUC_CONTROL);
	writel(1, vd->iftuc + IFTUC_START);
	readl(vd->iftuc + IFTUC_START);
}

static void vita_iftu_disable_plane(struct vita_display *vd, int plane)
{
	void __iomem *p = vd->iftu[plane];

	writel(1, p + IFTU_BANK(0) + BANK_BLANK);
	writel(1, p + IFTU_BANK(1) + BANK_BLANK);
	writel(0, p + IFTU_KICK);
	writel(0, p + IFTU_INT_STATUS);
	readl(p + IFTU_INT_STATUS);
}

/* ksceDisplaySetMergeConf(0, 2, 0x100): plane b opaque, added to plane a */
static void vita_iftu_merge(struct vita_display *vd)
{
	writel(2, vd->iftuc + IFTUC_MERGE);
	writel(readl(vd->iftuc + IFTUC_CONTROL) | BIT(0), vd->iftuc + IFTUC_CONTROL);
	vita_iftu_set_alpha(vd->iftu[PLANE_A], 0);
	vita_iftu_set_alpha(vd->iftu[PLANE_B], 0x100);
}

/*
 * ksceIftuSetInputFrameBuffer: fill the bank that is not being scanned and
 * select it; the plane switches at the next frame. paddr 0 blanks the plane.
 */
static void vita_iftu_flip(struct vita_display *vd, int plane, u32 paddr, u32 format,
			   u32 leftover)
{
	void __iomem *p = vd->iftu[plane];
	u32 live_reg = readl(p + IFTU_BANK_LIVE);
	int live = (live_reg & 3) >> 1, bank;
	void __iomem *b;

	if (vd->last_bank[plane] == live) {
		bank = live ^ 1;
	} else if (live_reg & 1) {
		writel(live, vd->iftuc + IFTUC_SELECT(plane));
		vd->last_bank[plane] = live;
		bank = live ^ 1;
	} else {
		bank = live;
	}
	b = p + IFTU_BANK(bank);

	if (!paddr) {
		vita_iftu_blank_bank(b);
	} else {
		writel(format, b + BANK_FORMAT);
		writel(OLED_WIDTH, b + BANK_WIDTH);
		writel(OLED_HEIGHT, b + BANK_HEIGHT);
		writel(leftover, b + BANK_LEFTOVER);
		writel(0, b + 0x58);
		writel(paddr, b + BANK_PADDR);
		writel(0, b + 0x04);
		writel(0, b + 0x08);
		writel(0, b + 0x20);
		writel(0, b + 0x24);
		writel(0, b + 0x28);
		writel(0, b + 0x60);
		writel(0, b + 0x64);
		writel(0, b + 0x68);
		writel(0, b + 0x6c);
		writel(0x10000, b + BANK_SRC_W);
		writel(0x10000, b + BANK_SRC_H);
		writel(0, b + 0xc8);
		writel(0, b + 0xcc);
		writel(0, b + 0xd0);
		writel(0, b + 0xd4);
		writel(0, b + BANK_BLANK);
	}
	writel(bank, vd->iftuc + IFTUC_SELECT(plane));
	vd->last_bank[plane] = bank;
	if (readl(p + IFTU_PERIOD) != 1)
		writel(3, p + IFTU_KICK);
}

/* ------------------------------------------------------------------ */
/* The panel over SPI2 (oled.elf 3.65) */

static int vita_spi_wait(struct vita_display *vd, unsigned int reg, u32 *v, bool empty)
{
	int ret;

	if (empty)
		ret = readl_poll_timeout(vd->spi + reg, *v, !*v, 1, 20000);
	else
		ret = readl_poll_timeout(vd->spi + reg, *v, *v != SPI_TX_FULL, 1, 20000);
	if (ret)
		drm_warn_once(&vd->drm, "SPI2 +0x%02x stuck at 0x%x\n", reg, *v);
	return ret;
}

static void vita_spi_drain(struct vita_display *vd)
{
	int n = 256;

	while (readl(vd->spi + SPI_RX_LEVEL) && n--)
		readl(vd->spi + SPI_RX);
}

/* The start of an SPI2 transaction: gate on, RX drained, run */
static void vita_spi_begin(struct vita_display *vd)
{
	perv_rmw(vd, vd->gate + PERV_SPI2, 0, 1);
	vita_spi_drain(vd);
	writel(0x30001, vd->spi + SPI_MODE);
	writel(1, vd->spi + SPI_RUN);
	vd->spi_bits = 0;
	vd->spi_nbits = 0;
}

/* One 9-bit frame, LSB first, into 16-bit TX words */
static void vita_spi_frame(struct vita_display *vd, u32 frame)
{
	u32 v;

	vd->spi_bits |= frame << vd->spi_nbits;
	vd->spi_nbits += 9;
	while (vd->spi_nbits >= 16) {
		vita_spi_wait(vd, SPI_TX_LEVEL, &v, false);
		writel(vd->spi_bits & 0xffff, vd->spi + SPI_TX);
		vd->spi_bits >>= 16;
		vd->spi_nbits -= 16;
	}
}

/* Zero frames (DCS NOPs) up to a word boundary, and all of it sent */
static void vita_spi_flush(struct vita_display *vd)
{
	u32 v;

	while (vd->spi_nbits)
		vita_spi_frame(vd, 0);
	vita_spi_wait(vd, SPI_TX_LEVEL, &v, true);
}

static void vita_spi_stop(struct vita_display *vd)
{
	u32 v;

	writel(0, vd->spi + SPI_RUN);
	if (readl_poll_timeout(vd->spi + SPI_RUN, v, !(v & 1), 1, 20000))
		drm_warn_once(&vd->drm, "SPI2 did not stop\n");
}

/* SceOled's FUN_81000098: RX drained, stopped, gate off */
static void vita_spi_end(struct vita_display *vd)
{
	vita_spi_drain(vd);
	vita_spi_stop(vd);
	writel(0x30001, vd->spi + SPI_MODE);
	readl(vd->spi + SPI_MODE);
	perv_rmw(vd, vd->gate + PERV_SPI2, 1, 0);
}

/* SceOledForDriver_2F0C4B67: one DCS command and its parameters */
static void vita_oled_cmd(struct vita_display *vd, u8 cmd, const u8 *data, unsigned int len)
{
	vita_spi_begin(vd);
	vita_spi_frame(vd, bitrev8(cmd) << 1);
	while (len--)
		vita_spi_frame(vd, bitrev8(*data++) << 1 | 1);
	vita_spi_flush(vd);
	vita_spi_end(vd);
}

/*
 * SceOled's command scripts: {cmd, len, data...}; cmd 0x0D is "wait len
 * frames", 0xFF ends. SceOled waits frames on the vblank or 16 ms apart.
 */
#define OLED_DELAY	0x0d
#define OLED_END	0xff

static void vita_oled_script(struct vita_display *vd, const u8 *s)
{
	while (*s != OLED_END) {
		if (*s == OLED_DELAY)
			msleep(DIV_ROUND_UP(s[1] * OLED_FRAME_US, 1000));
		else
			vita_oled_cmd(vd, s[0], s + 2, s[1]);
		s += 2 + (*s == OLED_DELAY ? 0 : s[1]);
	}
}

/* oled.elf 0x81001dd0: the first configuration script */
static const u8 oled_init1[] = {
	0xf0, 2, 0x5a, 0x5a,
	0xf1, 2, 0x5a, 0x5a,
	0xb0, 1, 0x02,
	0xf3, 1, 0x3b,
	0xf4, 4, 0x33, 0x42, 0x00, 0x08,
	0xf5, 5, 0x00, 0x06, 0x26, 0x35, 0x03,
	0xf6, 1, 0x02,
	0xc6, 10, 0x0b, 0x00, 0x00, 0x3c, 0x00, 0x22, 0x00, 0x00, 0x00, 0x00,
	0xf7, 1, 0x20,
	OLED_END,
};

/* 0x81001a88: the second script for DDB ids other than 4 */
static const u8 oled_init2[] = {
	0xf5, 5, 0x00, 0x06, 0x27, 0x35, 0x03,
	0xb2, 4, 0x06, 0x06, 0x06, 0x06,
	0xb1, 3, 0x07, 0x00, 0x10,
	0xf8, 19, 0x7f, 0x7a, 0x89, 0x67, 0x26, 0x38, 0x00, 0x00, 0x09, 0x67,
		  0x70, 0x88, 0x7a, 0x76, 0x05, 0x09, 0x23, 0x23, 0x23,
	OLED_END,
};

/* 0x81001d88: the second script for DDB id 4 */
static const u8 oled_init2_id4[] = {
	0xb2, 4, 0x0b, 0x0b, 0x0b, 0x0b,
	0xb1, 3, 0x07, 0x00, 0x16,
	0xf8, 19, 0x7f, 0x7a, 0x89, 0x67, 0x26, 0x38, 0x00, 0x00, 0x09, 0x67,
		  0x70, 0x88, 0x7a, 0x76, 0x05, 0x09, 0x23, 0x23, 0x23,
	OLED_END,
};

/* ksceOledDisplayOn: 4 frames, sleep out, 13 frames, display on, 1 frame */
static const u8 oled_on[] = {
	OLED_DELAY, 4, 0x11, 0, OLED_DELAY, 13, 0x29, 0, OLED_DELAY, 1, OLED_END,
};

/* ksceOledDisplayOff (0x81001a70): display off, 1 frame, sleep in, 6 frames */
static const u8 oled_off[] = {
	0x28, 0, OLED_DELAY, 1, 0x10, 0, OLED_DELAY, 6, OLED_END,
};

/*
 * Gamma rows for ksceOledSetBrightness, brightest first (0x81001e00; DDB id 5:
 * 0x81001c20; id 4: 0x81001ab8), and the ELVSS (B2) value per row
 * (0x81001f88, id 5 0x81001db0; id 4 has none).
 */
static const u8 oled_gamma[17][21] = {
	{ 0x9f, 0x9f, 0xbe, 0xcf, 0xd7, 0xc9, 0xc2, 0xcb, 0xbb, 0xe1, 0xe3, 0xde, 0xd6, 0xd0, 0xd3, 0xfa, 0xed, 0xe6, 0x2f, 0x00, 0x2f },
	{ 0x9c, 0x9c, 0xba, 0xd0, 0xd8, 0xcb, 0xc3, 0xcb, 0xbb, 0xe2, 0xe4, 0xdf, 0xd6, 0xce, 0xd3, 0xfa, 0xed, 0xe6, 0x2f, 0x00, 0x2f },
	{ 0x99, 0x99, 0xb6, 0xd1, 0xd9, 0xcc, 0xc3, 0xcb, 0xbc, 0xe2, 0xe4, 0xdf, 0xd6, 0xcc, 0xd3, 0xfa, 0xed, 0xe6, 0x2f, 0x00, 0x2f },
	{ 0x95, 0x95, 0xb1, 0xd2, 0xd9, 0xcc, 0xc4, 0xcd, 0xbe, 0xe2, 0xe3, 0xdf, 0xd7, 0xcc, 0xd3, 0xfa, 0xed, 0xe6, 0x2f, 0x00, 0x2f },
	{ 0x93, 0x93, 0xaf, 0xd3, 0xda, 0xcd, 0xc5, 0xcd, 0xbe, 0xe2, 0xe3, 0xdf, 0xd6, 0xca, 0xd3, 0xfa, 0xed, 0xe6, 0x2f, 0x00, 0x2f },
	{ 0x91, 0x91, 0xac, 0xd3, 0xda, 0xce, 0xc5, 0xcd, 0xbe, 0xe3, 0xe3, 0xe0, 0xd7, 0xca, 0xd3, 0xfa, 0xed, 0xe6, 0x2f, 0x00, 0x2f },
	{ 0x8f, 0x8f, 0xaa, 0xd4, 0xdb, 0xce, 0xc6, 0xcd, 0xbf, 0xe3, 0xe3, 0xe1, 0xd7, 0xca, 0xd3, 0xfa, 0xed, 0xe6, 0x2f, 0x00, 0x2f },
	{ 0x8d, 0x8d, 0xa7, 0xd5, 0xdb, 0xcf, 0xc6, 0xce, 0xc0, 0xe4, 0xe4, 0xe1, 0xd7, 0xc8, 0xd3, 0xfa, 0xed, 0xe6, 0x2f, 0x00, 0x2f },
	{ 0x8b, 0x8b, 0xa5, 0xd5, 0xdb, 0xcf, 0xc7, 0xce, 0xc0, 0xe3, 0xe3, 0xe1, 0xd8, 0xc7, 0xd3, 0xfa, 0xed, 0xe6, 0x2f, 0x00, 0x2f },
	{ 0x89, 0x89, 0xa2, 0xd5, 0xdb, 0xcf, 0xc8, 0xcf, 0xc2, 0xe3, 0xe3, 0xe1, 0xd9, 0xc7, 0xd3, 0xfa, 0xed, 0xe6, 0x2f, 0x00, 0x2f },
	{ 0x87, 0x86, 0x9f, 0xd6, 0xdd, 0xd1, 0xc7, 0xce, 0xc1, 0xe4, 0xe3, 0xe2, 0xd9, 0xc6, 0xd3, 0xfa, 0xed, 0xe6, 0x2f, 0x00, 0x2f },
	{ 0x84, 0x83, 0x9b, 0xd7, 0xde, 0xd2, 0xc8, 0xce, 0xc2, 0xe4, 0xe3, 0xe2, 0xd9, 0xc3, 0xd3, 0xfa, 0xed, 0xe6, 0x2f, 0x00, 0x2f },
	{ 0x82, 0x81, 0x99, 0xd6, 0xdd, 0xd1, 0xca, 0xcf, 0xc3, 0xe4, 0xe3, 0xe3, 0xda, 0xc2, 0xd3, 0xfa, 0xed, 0xe6, 0x2f, 0x00, 0x2f },
	{ 0x7f, 0x7e, 0x95, 0xd7, 0xde, 0xd2, 0xcb, 0xcf, 0xc5, 0xe5, 0xe3, 0xe3, 0xda, 0xbf, 0xd3, 0xfa, 0xed, 0xe6, 0x2f, 0x00, 0x2f },
	{ 0x7d, 0x7c, 0x92, 0xd7, 0xdd, 0xd2, 0xcb, 0xd0, 0xc6, 0xe5, 0xe1, 0xe3, 0xda, 0xbd, 0xd3, 0xfa, 0xed, 0xe6, 0x2f, 0x00, 0x2f },
	{ 0x79, 0x78, 0x8d, 0xd9, 0xdf, 0xd5, 0xcb, 0xcf, 0xc5, 0xe5, 0xe0, 0xe4, 0xdc, 0xb8, 0xd4, 0xfa, 0xed, 0xe6, 0x2f, 0x00, 0x2f },
	{ 0x6a, 0x68, 0x7a, 0xdc, 0xe1, 0xd8, 0xd1, 0xcf, 0xcd, 0xe6, 0xd9, 0xe3, 0xcd, 0x8c, 0xcb, 0xf0, 0x9c, 0xf9, 0x2f, 0x00, 0x2f },
};

static const u8 oled_gamma_id5[17][21] = {
	{ 0x94, 0x93, 0xb6, 0xc9, 0xd2, 0xc5, 0xb8, 0xc2, 0xb1, 0xda, 0xdc, 0xd6, 0xd2, 0xcb, 0xce, 0xf4, 0xdf, 0xdf, 0x2f, 0x00, 0x2f },
	{ 0x91, 0x90, 0xb2, 0xcc, 0xd5, 0xc8, 0xb8, 0xc0, 0xb0, 0xdb, 0xdc, 0xd7, 0xd1, 0xca, 0xce, 0xf4, 0xda, 0xd0, 0x2f, 0x00, 0x2f },
	{ 0x8e, 0x8d, 0xae, 0xcd, 0xd6, 0xc9, 0xb8, 0xc0, 0xb0, 0xdc, 0xdc, 0xd9, 0xd1, 0xc9, 0xcd, 0xf4, 0xda, 0xd0, 0x2f, 0x00, 0x2f },
	{ 0x8a, 0x89, 0xa9, 0xcd, 0xd6, 0xc9, 0xba, 0xc1, 0xb2, 0xdd, 0xdc, 0xda, 0xd1, 0xc6, 0xcb, 0xf4, 0xda, 0xd0, 0x2f, 0x00, 0x2f },
	{ 0x88, 0x87, 0xa6, 0xcd, 0xd6, 0xc9, 0xbc, 0xc2, 0xb4, 0xdb, 0xdb, 0xd9, 0xd1, 0xc6, 0xcb, 0xf4, 0xda, 0xd0, 0x2f, 0x00, 0x2f },
	{ 0x86, 0x85, 0xa4, 0xce, 0xd7, 0xca, 0xbb, 0xc1, 0xb3, 0xdb, 0xdb, 0xd9, 0xd2, 0xc6, 0xcb, 0xf4, 0xda, 0xd0, 0x2f, 0x00, 0x2f },
	{ 0x84, 0x83, 0xa1, 0xd0, 0xd8, 0xcc, 0xbb, 0xc1, 0xb4, 0xdc, 0xdb, 0xd9, 0xd2, 0xc4, 0xca, 0xf4, 0xda, 0xd0, 0x2f, 0x00, 0x2f },
	{ 0x82, 0x81, 0x9f, 0xd0, 0xd8, 0xcc, 0xbb, 0xc1, 0xb3, 0xdc, 0xda, 0xda, 0xd2, 0xc2, 0xc9, 0xf4, 0xda, 0xd0, 0x2f, 0x00, 0x2f },
	{ 0x80, 0x7f, 0x9c, 0xd0, 0xd8, 0xcc, 0xbd, 0xc2, 0xb6, 0xdd, 0xda, 0xda, 0xd2, 0xc1, 0xc9, 0xf4, 0xda, 0xd0, 0x2f, 0x00, 0x2f },
	{ 0x7e, 0x7d, 0x9a, 0xd1, 0xd8, 0xcc, 0xbc, 0xc1, 0xb5, 0xdc, 0xd9, 0xda, 0xd2, 0xbc, 0xc7, 0xf4, 0xda, 0xd0, 0x2f, 0x00, 0x2f },
	{ 0x7c, 0x7b, 0x97, 0xd0, 0xd8, 0xcc, 0xbe, 0xc2, 0xb7, 0xdd, 0xd9, 0xdb, 0xd2, 0xbd, 0xc7, 0xf4, 0xda, 0xd0, 0x2f, 0x00, 0x2f },
	{ 0x78, 0x77, 0x93, 0xd2, 0xd8, 0xcc, 0xbf, 0xc1, 0xb8, 0xdc, 0xd8, 0xdb, 0xd2, 0xcf, 0xc7, 0xf4, 0xda, 0xd0, 0x2f, 0x00, 0x2f },
	{ 0x75, 0x74, 0x8f, 0xd4, 0xd9, 0xce, 0xbe, 0xc0, 0xb8, 0xde, 0xd8, 0xdc, 0xd2, 0xcf, 0xc7, 0xf4, 0xda, 0xd0, 0x2f, 0x00, 0x2f },
	{ 0x73, 0x71, 0x8c, 0xd4, 0xdb, 0xd0, 0xbe, 0xbf, 0xb7, 0xde, 0xd7, 0xdc, 0xd2, 0xcf, 0xc7, 0xf4, 0xda, 0xd0, 0x2f, 0x00, 0x2f },
	{ 0x71, 0x6f, 0x8a, 0xd3, 0xda, 0xce, 0xbf, 0xbe, 0xb8, 0xde, 0xd7, 0xdc, 0xd3, 0xcf, 0xc8, 0xf4, 0xda, 0xd0, 0x2f, 0x00, 0x2f },
	{ 0x6d, 0x6b, 0x85, 0xd5, 0xdb, 0xd0, 0xc0, 0xbf, 0xba, 0xdf, 0xd5, 0xdb, 0xd3, 0x95, 0xc8, 0xf4, 0xda, 0xd0, 0x2f, 0x00, 0x2f },
	{ 0x5c, 0x5e, 0x76, 0xdc, 0xe1, 0xda, 0xbc, 0xbe, 0xbd, 0xdd, 0xd6, 0xd5, 0xe3, 0x4a, 0xdc, 0xaf, 0xb1, 0xfc, 0x2f, 0x00, 0x2f },
};

static const u8 oled_gamma_id4[17][21] = {
	{ 0x66, 0x64, 0x8f, 0xc5, 0xc4, 0xb9, 0xae, 0xaa, 0x9f, 0xd2, 0xc8, 0xc8, 0xbc, 0x90, 0xb6, 0xef, 0x5a, 0xca, 0x07, 0x00, 0x1a },
	{ 0x63, 0x60, 0x8b, 0xc4, 0xc5, 0xb9, 0xaf, 0xab, 0xa0, 0xd3, 0xc7, 0xc9, 0xbc, 0x93, 0xb6, 0xef, 0x5e, 0xca, 0x07, 0x00, 0x1a },
	{ 0x5f, 0x5c, 0x86, 0xc5, 0xc6, 0xba, 0xb0, 0xab, 0xa1, 0xd3, 0xc7, 0xc9, 0xba, 0x8b, 0xb2, 0xef, 0x4e, 0xca, 0x07, 0x00, 0x1a },
	{ 0x5b, 0x58, 0x80, 0xc6, 0xc7, 0xbc, 0xb1, 0xac, 0xa2, 0xd3, 0xc5, 0xc9, 0xba, 0x84, 0xb2, 0xef, 0x2e, 0xca, 0x07, 0x00, 0x1a },
	{ 0x58, 0x56, 0x7d, 0xc7, 0xc7, 0xbc, 0xb2, 0xab, 0xa2, 0xd2, 0xc5, 0xc9, 0xbb, 0x7f, 0xb2, 0xef, 0x1e, 0xd0, 0x07, 0x00, 0x1a },
	{ 0x57, 0x54, 0x7b, 0xc7, 0xc7, 0xbc, 0xb1, 0xab, 0xa2, 0xd2, 0xc3, 0xc9, 0xbc, 0x7e, 0xb2, 0xef, 0x1e, 0xd5, 0x07, 0x00, 0x1a },
	{ 0x54, 0x51, 0x77, 0xc9, 0xc8, 0xbf, 0xb4, 0xab, 0xa5, 0xd5, 0xc4, 0xcd, 0xc1, 0x97, 0xc3, 0xaf, 0x10, 0xcf, 0x03, 0x00, 0x15 },
	{ 0x52, 0x4f, 0x74, 0xc9, 0xc8, 0xbf, 0xb4, 0xaa, 0xa6, 0xd4, 0xc2, 0xcc, 0xc1, 0x8d, 0xc1, 0xaf, 0x10, 0xcf, 0x03, 0x00, 0x15 },
	{ 0x4f, 0x4c, 0x71, 0xca, 0xc9, 0xc0, 0xb4, 0xa9, 0xa5, 0xd5, 0xc2, 0xcd, 0xc0, 0x8a, 0xc0, 0xaf, 0x10, 0xcf, 0x03, 0x00, 0x15 },
	{ 0x4d, 0x4a, 0x6e, 0xca, 0xc8, 0xc0, 0xb5, 0xaa, 0xa6, 0xd5, 0xc0, 0xce, 0xc0, 0x82, 0xbe, 0xaf, 0x10, 0xbf, 0x03, 0x00, 0x15 },
	{ 0x4a, 0x47, 0x6a, 0xcb, 0xc9, 0xc1, 0xb5, 0xa9, 0xa6, 0xd5, 0xbe, 0xce, 0xc0, 0x82, 0xbe, 0xaf, 0x10, 0xbf, 0x03, 0x00, 0x15 },
	{ 0x47, 0x44, 0x67, 0xcc, 0xca, 0xc1, 0xb6, 0xa9, 0xa7, 0xd6, 0xbe, 0xcf, 0xc1, 0x7e, 0xbe, 0xaf, 0x10, 0xbf, 0x03, 0x00, 0x15 },
	{ 0x45, 0x42, 0x64, 0xcc, 0xc9, 0xc1, 0xb6, 0xa8, 0xa7, 0xd5, 0xbb, 0xcf, 0xbd, 0x60, 0xb7, 0xaf, 0x10, 0xbf, 0x03, 0x00, 0x15 },
	{ 0x42, 0x3f, 0x60, 0xcc, 0xc9, 0xc1, 0xb6, 0xa6, 0xa7, 0xd5, 0xb7, 0xcf, 0xbf, 0x5a, 0xb7, 0xaf, 0x10, 0xbf, 0x03, 0x00, 0x15 },
	{ 0x3e, 0x3a, 0x5a, 0xcc, 0xca, 0xc3, 0xb9, 0xa8, 0xaa, 0xd5, 0xb6, 0xd0, 0xbf, 0x53, 0xb7, 0xb0, 0x10, 0xbf, 0x03, 0x00, 0x15 },
	{ 0x3b, 0x37, 0x56, 0xcd, 0xca, 0xc3, 0xb7, 0xa4, 0xa9, 0xd5, 0xb1, 0xd0, 0xbe, 0x10, 0xb1, 0xb0, 0x10, 0xbf, 0x03, 0x00, 0x15 },
	{ 0x2a, 0x24, 0x40, 0xcd, 0xca, 0xc3, 0xb7, 0xa4, 0xa9, 0xd5, 0xb1, 0xd0, 0xbe, 0x10, 0xb1, 0xb0, 0x10, 0xbf, 0x03, 0x00, 0x15 },
};

static const u8 oled_elvss[17] = {
	0x12, 0x13, 0x14, 0x14, 0x15, 0x15, 0x15, 0x15, 0x15,
	0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15,
};

#define OLED_ROWS		17
#define OLED_DEFAULT_ROW	4	/* B2 15, as the resume trace shows */

/*
 * SceOled's module start (FUN_8100012c): read_DDB_start (0xA1) with the panel
 * powered, five bytes after a command and a dummy frame. A panel answers
 * {0, 0, id, id_hi, 0xFF}. Returns 0 if it did.
 */
static int vita_oled_read_ddb(struct vita_display *vd)
{
	u32 acc = 0, v;
	unsigned int nbits = 0, n = 0, skip = 2;
	int i;

	vita_spi_begin(vd);
	vita_spi_frame(vd, bitrev8(0xa1) << 1);
	for (i = 0; i < 6; i++)
		vita_spi_frame(vd, 1);
	vita_spi_flush(vd);
	vita_spi_stop(vd);
	while (n < 5) {
		if (readl_poll_timeout(vd->spi + SPI_RX_LEVEL, v, v, 1, 20000))
			break;
		acc |= readl(vd->spi + SPI_RX) << nbits;
		for (nbits += 16; nbits > 8; nbits -= 9, acc >>= 9) {
			if (skip)
				skip--;
			else if (n < 5)
				vd->ddb[n++] = bitrev8(acc >> 1);
		}
	}
	vita_spi_end(vd);
	return n == 5 && vd->ddb[4] == 0xff && !vd->ddb[0] && !vd->ddb[1] ? 0 : -ENODEV;
}

/* SceOled's resume event 0x10003 and module start: SPI2 out of reset, set up */
static void vita_oled_spi_setup(struct vita_display *vd)
{
	perv_rmw(vd, vd->reset + PERV_SPI2, 1, 0);
	perv_rmw(vd, vd->gate + PERV_SPI2, 0, 1);
	writel(0x30001, vd->spi + SPI_MODE);
	writel(0xf, vd->spi + SPI_DIVIDER);
	writel(3, vd->spi + SPI_CONFIG);
	writel(0, vd->spi + SPI_IRQ_MASK);
	readl(vd->spi + SPI_IRQ_MASK);
	perv_rmw(vd, vd->gate + PERV_SPI2, 1, 0);
}

/* SceOled's suspend event 0x4003 */
static void vita_oled_spi_shutdown(struct vita_display *vd)
{
	perv_rmw(vd, vd->gate + PERV_SPI2, 1, 0);
	perv_rmw(vd, vd->reset + PERV_SPI2, 0, 1);
}

static void vita_oled_identify(struct vita_display *vd)
{
	int id;

	vd->gamma = &oled_gamma[0][0];
	vd->elvss = oled_elvss;
	vd->init2 = oled_init2;
	if (vita_oled_read_ddb(vd)) {
		drm_warn(&vd->drm, "panel DDB %*ph not recognised, using the default tables\n",
			 5, vd->ddb);
		return;
	}
	id = vd->ddb[2];
	if (id == 4) {
		vd->gamma = &oled_gamma_id4[0][0];
		vd->elvss = NULL;
		vd->init2 = oled_init2_id4;
	} else if (id == 5) {
		vd->gamma = &oled_gamma_id5[0][0];
	}
	drm_info(&vd->drm, "OLED panel DDB %*ph (id %d)\n", 5, vd->ddb, id);
}

/*
 * ksceOledSetBrightness: after the panel was off, B5/B4/B3 go first; the
 * gamma row, F9 00, 26 00 and the ELVSS level follow; after the panel was
 * off, sleep out and display on are sent again, as SceOled does.
 */
static void vita_oled_brightness(struct vita_display *vd, unsigned int row)
{
	static const u8 b5[] = {
		0xff, 0xef, 0x35, 0x42, 0x0d, 0xd7, 0xff, 0x07, 0xff, 0xff, 0xfd,
		0x00, 0x01, 0xff, 0x05, 0x12, 0x0f, 0xff, 0xff, 0xff, 0xff,
	};
	static const u8 b4 = 0x15, b3 = 0x00, zero = 0;
	u8 f9[22];

	if (vd->first_brightness) {
		vita_oled_cmd(vd, 0xb5, b5, sizeof(b5));
		vita_oled_cmd(vd, 0xb4, &b4, 1);
		vita_oled_cmd(vd, 0xb3, &b3, 1);
	}
	f9[0] = 1;
	memcpy(f9 + 1, vd->gamma + row * 21, 21);
	vita_oled_cmd(vd, 0xf9, f9, sizeof(f9));
	vita_oled_cmd(vd, 0xf9, &zero, 1);
	vita_oled_cmd(vd, 0x26, &zero, 1);
	if (vd->elvss)
		vita_oled_cmd(vd, 0xb2, &vd->elvss[row], 1);
	if (vd->first_brightness)
		vita_oled_script(vd, oled_on);
	vd->first_brightness = false;
	vd->row = row;
}

/* SceOledForDriver_DDB1412B: supply on, 120 ms, the configuration */
static void vita_oled_prepare(struct vita_display *vd)
{
	gpiod_direction_output(vd->power, 1);
	msleep(120);
	vita_oled_script(vd, oled_init1);
	vita_oled_script(vd, vd->init2);
	vd->panel_prepared = true;
}

static void vita_oled_unprepare(struct vita_display *vd)
{
	gpiod_direction_output(vd->power, 0);
	vd->panel_prepared = false;
}

static int vita_backlight_update(struct backlight_device *bl)
{
	struct vita_display *vd = bl_get_data(bl);
	unsigned int level = backlight_get_brightness(bl);

	mutex_lock(&vd->panel_lock);
	vd->row = OLED_ROWS - 1 - min(level, OLED_ROWS - 1U);
	if (vd->panel_on)
		vita_oled_brightness(vd, vd->row);
	mutex_unlock(&vd->panel_lock);
	return 0;
}

static const struct backlight_ops vita_backlight_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = vita_backlight_update,
};

/* ------------------------------------------------------------------ */
/* The head */

/* SceDisplay's display_head_enable (display.c FUN_81000c2c) and SceOled's resume */
static void vita_head_enable(struct vita_display *vd)
{
	vita_dsi_pll_on(vd);
	perv_rmw(vd, vd->gate + PERV_DSI0, 0, 0xf);
	perv_rmw(vd, vd->reset + PERV_DSI0, 7, 0);
	perv_rmw(vd, vd->gate + PERV_IFTU, 0, 1);
	perv_rmw(vd, vd->reset + PERV_IFTU, 1, 0);
	writel(0, vd->misc + MISC_DSI0);
	vita_dsi_enable_head(vd);
	vita_iftu_enable_plane(vd, PLANE_A);
	vita_iftu_enable_plane(vd, PLANE_B);
	vita_iftu_merge(vd);

	mutex_lock(&vd->panel_lock);
	vita_oled_spi_setup(vd);
	vita_oled_prepare(vd);
	mutex_unlock(&vd->panel_lock);

	vita_dsi_start_display(vd);
}

/*
 * SceDisplay's head disable (display.c FUN_81001360) and SceOled's suspend:
 * the teardown the OS gives DSI1; DSI0 gets the same so that a later enable
 * starts from the state the OS resumes from.
 */
static void vita_head_disable(struct vita_display *vd)
{
	vita_iftu_disable_plane(vd, PLANE_A);
	vita_iftu_disable_plane(vd, PLANE_B);
	vita_dsi_send_blanking(vd);
	msleep(42);
	vita_dsi_disable_head(vd);
	writel(1, vd->misc + MISC_DSI0);
	perv_rmw(vd, vd->gate + PERV_DSI0, 0xf, 0);
	perv_rmw(vd, vd->reset + PERV_DSI0, 0, 7);
	vita_dsi_pll_off(vd);

	mutex_lock(&vd->panel_lock);
	vita_oled_unprepare(vd);
	vita_oled_spi_shutdown(vd);
	mutex_unlock(&vd->panel_lock);
}

static void vita_panel_on(struct vita_display *vd)
{
	mutex_lock(&vd->panel_lock);
	vita_oled_script(vd, oled_on);
	vd->first_brightness = true;
	vd->panel_on = true;
	vita_oled_brightness(vd, vd->row);
	mutex_unlock(&vd->panel_lock);
}

static void vita_panel_off(struct vita_display *vd)
{
	mutex_lock(&vd->panel_lock);
	if (vd->panel_prepared)
		vita_oled_script(vd, oled_off);
	vd->panel_on = false;
	mutex_unlock(&vd->panel_lock);
}

static irqreturn_t vita_dsi_irq(int irq, void *arg)
{
	struct vita_display *vd = arg;
	u32 status = readl(vd->dsi + DSI_INT_STATUS);

	if (!status)
		return IRQ_NONE;
	writel(status, vd->dsi + DSI_INT_STATUS);
	readl(vd->dsi + DSI_INT_STATUS);
	if (status & DSI_INT_VBLANK) {
		writel(OLED_VTOTAL, vd->dsi + DSI_VTOTAL);
		drm_crtc_handle_vblank(&vd->crtc);
	}
	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* KMS */

/* VIC 0: hfp 20, hsync 66, hbp 4, vfp 4, vsync 4, vbp 42 */
static const struct drm_display_mode vita_oled_mode = {
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
	.clock = 37385,		/* 1050 x 594 at 59.94 Hz */
	.hdisplay = OLED_WIDTH,
	.hsync_start = OLED_WIDTH + 20,
	.hsync_end = OLED_WIDTH + 20 + 66,
	.htotal = 1050,
	.vdisplay = OLED_HEIGHT,
	.vsync_start = OLED_HEIGHT + 4,
	.vsync_end = OLED_HEIGHT + 4 + 4,
	.vtotal = OLED_VTOTAL,
	.width_mm = 111,	/* 5" 960x544 */
	.height_mm = 63,
};

/*
 * The plane is merged over the blank plane a with a constant alpha (merge
 * setting 2, alpha b 0x100), so pixel alpha is not used and only the X
 * formats are offered.
 */
static const u32 vita_plane_formats[] = {
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_XRGB8888,
};

static int vita_plane_atomic_check(struct drm_plane *plane, struct drm_atomic_commit *state)
{
	struct drm_plane_state *new = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *crtc_state = NULL;
	int ret;

	if (new->crtc)
		crtc_state = drm_atomic_get_new_crtc_state(state, new->crtc);
	ret = drm_atomic_helper_check_plane_state(new, crtc_state, DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING, false, true);
	if (ret || !new->visible)
		return ret;
	if (new->src_x || new->src_y || new->fb->pitches[0] < OLED_WIDTH * 4 ||
	    new->fb->pitches[0] % 256)
		return -EINVAL;
	return 0;
}

static void vita_plane_atomic_update(struct drm_plane *plane, struct drm_atomic_commit *state)
{
	struct vita_display *vd = to_vd(plane->dev);
	struct drm_plane_state *new = drm_atomic_get_new_plane_state(state, plane);
	struct drm_framebuffer *fb = new->fb;
	u32 format;

	if (!new->visible || !fb) {
		vita_iftu_flip(vd, PLANE_B, 0, 0, 0);
		return;
	}
	switch (fb->format->format) {
	case DRM_FORMAT_XRGB8888:
		format = IFTU_FMT_BGRA;
		break;
	default:
		format = IFTU_FMT_RGBA;
		break;
	}
	vita_iftu_flip(vd, PLANE_B, drm_fb_dma_get_gem_addr(fb, new, 0), format,
		       fb->pitches[0] - OLED_WIDTH * 4);
}

static void vita_plane_atomic_disable(struct drm_plane *plane, struct drm_atomic_commit *state)
{
	struct vita_display *vd = to_vd(plane->dev);
	struct drm_plane_state *old = drm_atomic_get_old_plane_state(state, plane);
	struct drm_crtc_state *crtc_state;

	if (!old->crtc)
		return;
	crtc_state = drm_atomic_get_new_crtc_state(state, old->crtc);
	if (crtc_state && crtc_state->active)
		vita_iftu_flip(vd, PLANE_B, 0, 0, 0);
}


static const struct drm_plane_helper_funcs vita_plane_helper_funcs = {
	.atomic_check = vita_plane_atomic_check,
	.atomic_update = vita_plane_atomic_update,
	.atomic_disable = vita_plane_atomic_disable,
};

static const struct drm_plane_funcs vita_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static enum drm_mode_status vita_crtc_mode_valid(struct drm_crtc *crtc,
						 const struct drm_display_mode *mode)
{
	if (mode->hdisplay != OLED_WIDTH || mode->vdisplay != OLED_HEIGHT ||
	    mode->htotal != vita_oled_mode.htotal || mode->vtotal != OLED_VTOTAL)
		return MODE_BAD;
	return MODE_OK;
}

static void vita_crtc_atomic_enable(struct drm_crtc *crtc, struct drm_atomic_commit *state)
{
	struct vita_display *vd = to_vd(crtc->dev);

	vita_head_enable(vd);
	drm_crtc_vblank_on(crtc);
}

static void vita_crtc_atomic_disable(struct drm_crtc *crtc, struct drm_atomic_commit *state)
{
	struct vita_display *vd = to_vd(crtc->dev);

	drm_crtc_vblank_off(crtc);
	vita_head_disable(vd);

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event && !crtc->state->active) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);
}

static void vita_crtc_atomic_flush(struct drm_crtc *crtc, struct drm_atomic_commit *state)
{
	struct drm_pending_vblank_event *event = crtc->state->event;

	if (!event)
		return;
	crtc->state->event = NULL;
	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->active && drm_crtc_vblank_get(crtc) == 0)
		drm_crtc_arm_vblank_event(crtc, event);
	else
		drm_crtc_send_vblank_event(crtc, event);
	spin_unlock_irq(&crtc->dev->event_lock);
}

static int vita_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct vita_display *vd = to_vd(crtc->dev);

	writel(DSI_INT_VBLANK, vd->dsi + DSI_INT_ENABLE);
	return 0;
}

static void vita_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct vita_display *vd = to_vd(crtc->dev);

	writel(0, vd->dsi + DSI_INT_ENABLE);
}

static const struct drm_crtc_helper_funcs vita_crtc_helper_funcs = {
	.mode_valid = vita_crtc_mode_valid,
	.atomic_enable = vita_crtc_atomic_enable,
	.atomic_disable = vita_crtc_atomic_disable,
	.atomic_flush = vita_crtc_atomic_flush,
};

static const struct drm_crtc_funcs vita_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = vita_crtc_enable_vblank,
	.disable_vblank = vita_crtc_disable_vblank,
};

/* The encoder turns the panel on and off; the CRTC owns the link */
static void vita_encoder_atomic_enable(struct drm_encoder *encoder,
				       struct drm_atomic_commit *state)
{
	vita_panel_on(to_vd(encoder->dev));
}

static void vita_encoder_atomic_disable(struct drm_encoder *encoder,
					struct drm_atomic_commit *state)
{
	vita_panel_off(to_vd(encoder->dev));
}

static const struct drm_encoder_helper_funcs vita_encoder_helper_funcs = {
	.atomic_enable = vita_encoder_atomic_enable,
	.atomic_disable = vita_encoder_atomic_disable,
};

static int vita_connector_get_modes(struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &vita_oled_mode);
}

static const struct drm_connector_helper_funcs vita_connector_helper_funcs = {
	.get_modes = vita_connector_get_modes,
};

static const struct drm_connector_funcs vita_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_mode_config_funcs vita_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

/* Enables before planes: a flip needs the IFTU running */
static const struct drm_mode_config_helper_funcs vita_mode_config_helpers = {
	.atomic_commit_tail = drm_atomic_helper_commit_tail_rpm,
};

static int vita_modeset_init(struct vita_display *vd)
{
	struct drm_device *drm = &vd->drm;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;
	drm->mode_config.min_width = OLED_WIDTH;
	drm->mode_config.min_height = OLED_HEIGHT;
	drm->mode_config.max_width = 4096;
	drm->mode_config.max_height = 4096;
	drm->mode_config.preferred_depth = 24;
	drm->mode_config.funcs = &vita_mode_config_funcs;
	drm->mode_config.helper_private = &vita_mode_config_helpers;

	ret = drm_universal_plane_init(drm, &vd->plane, 0, &vita_plane_funcs,
				       vita_plane_formats, ARRAY_SIZE(vita_plane_formats),
				       NULL, DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&vd->plane, &vita_plane_helper_funcs);

	ret = drm_crtc_init_with_planes(drm, &vd->crtc, &vd->plane, NULL,
					&vita_crtc_funcs, "oled");
	if (ret)
		return ret;
	drm_crtc_helper_add(&vd->crtc, &vita_crtc_helper_funcs);

	ret = drm_simple_encoder_init(drm, &vd->encoder, DRM_MODE_ENCODER_DSI);
	if (ret)
		return ret;
	vd->encoder.possible_crtcs = drm_crtc_mask(&vd->crtc);
	drm_encoder_helper_add(&vd->encoder, &vita_encoder_helper_funcs);

	ret = drm_connector_init(drm, &vd->connector, &vita_connector_funcs,
				 DRM_MODE_CONNECTOR_DSI);
	if (ret)
		return ret;
	drm_connector_helper_add(&vd->connector, &vita_connector_helper_funcs);
	vd->connector.display_info.width_mm = vita_oled_mode.width_mm;
	vd->connector.display_info.height_mm = vita_oled_mode.height_mm;
	ret = drm_connector_attach_encoder(&vd->connector, &vd->encoder);
	if (ret)
		return ret;

	ret = drm_vblank_init(drm, 1);
	if (ret)
		return ret;
	drm_mode_config_reset(drm);
	return 0;
}

DEFINE_DRM_GEM_DMA_FOPS(vita_display_fops);

static const struct drm_driver vita_display_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.fops = &vita_display_fops,
	.name = "vita-display",
	.desc = "PlayStation Vita display",
	.major = 1,
	.minor = 0,
	DRM_GEM_DMA_DRIVER_OPS,
	DRM_FBDEV_DMA_DRIVER_OPS,
};

static void __iomem *vita_map_shared(struct platform_device *pdev, const char *name)
{
	struct resource *res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);

	if (!res)
		return IOMEM_ERR_PTR(-EINVAL);
	/* Pervasive is shared with the reset, clock and USB drivers */
	return devm_ioremap(&pdev->dev, res->start, resource_size(res)) ?:
	       IOMEM_ERR_PTR(-ENOMEM);
}

static void vita_release_reserved(void *dev)
{
	of_reserved_mem_device_release(dev);
}

/*
 * The panel's DDB, read as SceOled does at module start, with its supply on
 * for the read only if it was off. A head the boot loader left running is
 * then taken down (display off first).
 */
static void vita_display_take_over(struct vita_display *vd)
{
	bool running = readl(vd->gate + PERV_DSI0) & 0xf;
	bool powered = gpiod_get_value(vd->power) > 0;

	mutex_lock(&vd->panel_lock);
	vita_oled_spi_setup(vd);
	if (!powered) {
		gpiod_direction_output(vd->power, 1);
		msleep(20);
	}
	vita_oled_identify(vd);
	if (!running) {
		if (!powered)
			gpiod_direction_output(vd->power, 0);
		vita_oled_spi_shutdown(vd);
	}
	vd->panel_prepared = running;
	mutex_unlock(&vd->panel_lock);

	if (running) {
		drm_info(&vd->drm, "taking over the head the boot loader started\n");
		vita_panel_off(vd);
		vita_head_disable(vd);
		/*
		 * The boot loader enables the plane CSC (+0x100 = 1) with its own
		 * matrices; SceIftu never writes +0x100, so it is left as the
		 * OS finds it after a reset (inferred to be 0).
		 */
		writel(0, vd->iftu[PLANE_A] + 0x100);
		writel(0, vd->iftu[PLANE_B] + 0x100);
	}
}

static int vita_display_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.max_brightness = OLED_ROWS - 1,
		.brightness = OLED_ROWS - 1 - OLED_DEFAULT_ROW,
	};
	struct vita_display *vd;
	int ret;

	vd = devm_drm_dev_alloc(dev, &vita_display_driver, struct vita_display, drm);
	if (IS_ERR(vd))
		return PTR_ERR(vd);
	platform_set_drvdata(pdev, vd);
	spin_lock_init(&vd->perv_lock);
	ret = drmm_mutex_init(&vd->drm, &vd->panel_lock);
	if (ret)
		return ret;
	vd->row = OLED_DEFAULT_ROW;

	vd->dsi = devm_platform_ioremap_resource_byname(pdev, "dsi");
	vd->iftu[PLANE_A] = devm_platform_ioremap_resource_byname(pdev, "iftu-a");
	vd->iftu[PLANE_B] = devm_platform_ioremap_resource_byname(pdev, "iftu-b");
	vd->iftuc = devm_platform_ioremap_resource_byname(pdev, "iftu-ctl");
	vd->spi = devm_platform_ioremap_resource_byname(pdev, "spi");
	vd->misc = vita_map_shared(pdev, "misc");
	vd->reset = vita_map_shared(pdev, "reset");
	vd->gate = vita_map_shared(pdev, "gate");
	vd->baseclk = vita_map_shared(pdev, "baseclk");
	if (IS_ERR(vd->dsi) || IS_ERR(vd->iftu[PLANE_A]) || IS_ERR(vd->iftu[PLANE_B]) ||
	    IS_ERR(vd->iftuc) || IS_ERR(vd->spi) || IS_ERR(vd->misc) || IS_ERR(vd->reset) ||
	    IS_ERR(vd->gate) || IS_ERR(vd->baseclk))
		return dev_err_probe(dev, -EINVAL, "missing registers\n");

	vd->power = devm_gpiod_get(dev, "power", GPIOD_ASIS);
	if (IS_ERR(vd->power))
		return dev_err_probe(dev, PTR_ERR(vd->power), "no panel supply GPIO\n");
	vd->irq = platform_get_irq(pdev, 0);
	if (vd->irq < 0)
		return vd->irq;

	ret = of_reserved_mem_device_init(dev);
	if (ret)
		return dev_err_probe(dev, ret, "no scanout memory\n");
	ret = devm_add_action_or_reset(dev, vita_release_reserved, dev);
	if (ret)
		return ret;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	/* the boot loader's simple-framebuffer, if any */
	ret = aperture_remove_all_conflicting_devices(vita_display_driver.name);
	if (ret)
		return ret;

	vd->soc_rev = readl(vd->misc + MISC_SOC_REV);
	vita_display_take_over(vd);

	ret = devm_request_irq(dev, vd->irq, vita_dsi_irq, 0, "vita-dsi0", vd);
	if (ret)
		return ret;

	vd->backlight = devm_backlight_device_register(dev, "vita-oled", dev, vd,
						       &vita_backlight_ops, &props);
	if (IS_ERR(vd->backlight))
		return PTR_ERR(vd->backlight);

	ret = vita_modeset_init(vd);
	if (ret)
		return ret;
	ret = drm_dev_register(&vd->drm, 0);
	if (ret)
		return ret;
	drm_info(&vd->drm, "OLED head: DSI0, IFTU0, SoC revision 0x%x\n", vd->soc_rev);
	drm_client_setup(&vd->drm, NULL);
	return 0;
}

static void vita_display_remove(struct platform_device *pdev)
{
	struct vita_display *vd = platform_get_drvdata(pdev);

	drm_dev_unplug(&vd->drm);
	drm_atomic_helper_shutdown(&vd->drm);
}

static void vita_display_shutdown(struct platform_device *pdev)
{
	struct vita_display *vd = platform_get_drvdata(pdev);

	drm_atomic_helper_shutdown(&vd->drm);
}

static int vita_display_suspend(struct device *dev)
{
	struct vita_display *vd = dev_get_drvdata(dev);

	return drm_mode_config_helper_suspend(&vd->drm);
}

static int vita_display_resume(struct device *dev)
{
	struct vita_display *vd = dev_get_drvdata(dev);

	return drm_mode_config_helper_resume(&vd->drm);
}

static DEFINE_SIMPLE_DEV_PM_OPS(vita_display_pm, vita_display_suspend, vita_display_resume);

static const struct of_device_id vita_display_of_match[] = {
	{ .compatible = "sony,vita-display" },
	{ }
};
MODULE_DEVICE_TABLE(of, vita_display_of_match);

static struct platform_driver vita_display_platform_driver = {
	.probe = vita_display_probe,
	.remove = vita_display_remove,
	.shutdown = vita_display_shutdown,
	.driver = {
		.name = "vita-display",
		.of_match_table = vita_display_of_match,
		.pm = pm_sleep_ptr(&vita_display_pm),
	},
};
module_platform_driver(vita_display_platform_driver);

MODULE_DESCRIPTION("PlayStation Vita display (DSI0, IFTU0, OLED panel)");
MODULE_LICENSE("GPL");
