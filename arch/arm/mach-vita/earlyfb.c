// SPDX-License-Identifier: GPL-2.0
/*
 * "earlyfb" on the command line: the kernel log on the boot loader's
 * framebuffer from paging_init on, for a console with no UART to read.
 * The framebuffer console only takes over once simplefb probes; a kernel
 * that stops before then leaves the boot loader's screen as it was.
 *
 * The framebuffer is mapped statically (.map_io) and the console registers
 * right there, in the middle of paging_init, so an oops or panic from then on
 * is on the screen; it replays the log from the first line.  Until the device
 * tree is unflattened (init_early) it assumes the Vita's 960x544 panel.
 * Lines wrap from the bottom back to the top rather than scroll, which would
 * move 2 MiB of uncached memory per line.  It is a boot console: it stops
 * when ttyS0 registers, after fbcon has taken the screen over.
 */

#include <linux/console.h>
#include <linux/font.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/sizes.h>

#include <asm/mach/map.h>

#include "vita.h"

/* The boot loader's framebuffer, as the DTs' simple-framebuffer nodes give it. */
#define EARLYFB_PHYS		0x20000000
/*
 * 1280x720x4 fits (PS TV). On a Vita the upper 2 MiB are the start of the
 * GPU's CDRAM region (the DTs' cdram_gpu), which userspace maps with the
 * same write-combined memory type.
 */
#define EARLYFB_SIZE		SZ_4M
#define EARLYFB_VIRT		0xfc000000

#define EARLYFB_FG		0xffffffff	/* a8b8g8r8 */
#define EARLYFB_BG		0xff000000

static bool earlyfb_enabled __initdata;
static u32 __iomem *earlyfb_base;
static unsigned int earlyfb_width = 960, earlyfb_height = 544, earlyfb_stride = 960;
static unsigned int earlyfb_cols, earlyfb_rows, earlyfb_x, earlyfb_y;
static const struct font_desc *earlyfb_font = &font_vga_8x16;

static int __init earlyfb_setup(char *arg)
{
	earlyfb_enabled = true;
	return 0;
}
early_param("earlyfb", earlyfb_setup);

static void earlyfb_fill_row(unsigned int row, u32 color)
{
	unsigned int h = earlyfb_font->height, y, x;

	for (y = row * h; y < (row + 1) * h; y++)
		for (x = 0; x < earlyfb_width; x++)
			writel_relaxed(color, earlyfb_base + y * earlyfb_stride + x);
}

static void earlyfb_newline(void)
{
	earlyfb_x = 0;
	earlyfb_y = (earlyfb_y + 1) % earlyfb_rows;
	earlyfb_fill_row(earlyfb_y, EARLYFB_BG);
	/* Mark where the newest line is while the screen wraps. */
	earlyfb_fill_row((earlyfb_y + 1) % earlyfb_rows, EARLYFB_BG);
}

static void earlyfb_putc(unsigned char c)
{
	const unsigned char *glyph;
	unsigned int w = earlyfb_font->width, h = earlyfb_font->height;
	unsigned int px = earlyfb_x * w, py = earlyfb_y * h, y, x;

	if (c == '\n') {
		earlyfb_newline();
		return;
	}
	if (c == '\r')
		return;

	glyph = (const unsigned char *)earlyfb_font->data + c * h;
	for (y = 0; y < h; y++)
		for (x = 0; x < w; x++)
			writel_relaxed(glyph[y] & (0x80 >> x) ? EARLYFB_FG : EARLYFB_BG,
				       earlyfb_base + (py + y) * earlyfb_stride + px + x);

	if (++earlyfb_x == earlyfb_cols)
		earlyfb_newline();
}

static void earlyfb_write(struct console *con, const char *s, unsigned int n)
{
	while (n--)
		earlyfb_putc(*s++);
	wmb();
}

static struct console earlyfb_console = {
	.name	= "earlyfb",
	.write	= earlyfb_write,
	.flags	= CON_PRINTBUFFER | CON_BOOT,
	.index	= -1,
};

void __init vita_earlyfb_map_io(void)
{
	struct map_desc desc = {
		.virtual	= EARLYFB_VIRT,
		.pfn		= __phys_to_pfn(EARLYFB_PHYS),
		.length		= EARLYFB_SIZE,
		.type		= MT_DEVICE_WC,
	};

	if (!earlyfb_enabled)
		return;

	iotable_init(&desc, 1);
	earlyfb_base = (u32 __iomem *)EARLYFB_VIRT;
	earlyfb_cols = earlyfb_width / earlyfb_font->width;
	earlyfb_rows = earlyfb_height / earlyfb_font->height;
	for (earlyfb_y = 0; earlyfb_y < earlyfb_rows; earlyfb_y++)
		earlyfb_fill_row(earlyfb_y, EARLYFB_BG);
	earlyfb_y = 0;

	register_console(&earlyfb_console);
}

void __init vita_earlyfb_init(void)
{
	struct device_node *np;
	u32 width, height, stride;

	if (!earlyfb_enabled)
		return;

	/* Running since .map_io with the panel's size: take the DT's (one CPU, IRQs off) */
	np = of_find_compatible_node(NULL, NULL, "simple-framebuffer");
	if (np && !of_property_read_u32(np, "width", &width) &&
	    !of_property_read_u32(np, "height", &height) &&
	    !of_property_read_u32(np, "stride", &stride) &&
	    stride * height <= EARLYFB_SIZE) {
		earlyfb_width = width;
		earlyfb_height = height;
		earlyfb_stride = stride / 4;
	}
	of_node_put(np);

	earlyfb_cols = earlyfb_width / earlyfb_font->width;
	earlyfb_rows = earlyfb_height / earlyfb_font->height;
	if (earlyfb_y >= earlyfb_rows)
		earlyfb_y = 0;
}
