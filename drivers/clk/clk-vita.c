// SPDX-License-Identifier: GPL-2.0
/*
 * PlayStation Vita ScePervasiveBaseClk (0xE3103000): the SoC's clock
 * multipliers and dividers. Only the ARM clock is provided so far.
 *
 * The ARM clock is the reference clock times the multiplier that BaseClk
 * +0x00 bits 3:0 select, less 1/16 per step of the divider at +0x04 (0 to 8;
 * 9 and up act as 0) [WIKI Pervasive, ARM Clocks]:
 *
 *   selector    1    3  4  5  6  7   8   9  10   12  13  14  15
 *   multiplier  1.5  3  4  6  8  12  16  18  12  4.5  9  16  18
 *
 * 0, 2 and 11 are undefined and act as 1. Selectors 8, 9 and 10 run the L2
 * cache at half the core clock; the others run it at the core clock. The
 * clock switches only between the eight of those that ScePower uses (1, 3 to
 * 7 and 14) or that the boot loader sets (15), always with divider 0, as
 * ScePower does.
 *
 * The Vita OS does not jump from one selector to another: SceLowio
 * (ScePervasiveForDriver_E9D95643, lowio.skprx 3.65 0x81000f4c) looks up a
 * route of up to four selector writes in a table at 0x810082b0 and writes
 * them with interrupts off on its own CPU while the other cores run on. The
 * routes here are that table's.
 */

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <dt-bindings/clock/sony,vita-baseclk.h>

#define BASECLK_ARM_SEL		0x00
#define BASECLK_ARM_DIV		0x04
#define ARM_SEL_MASK		0xf
#define ARM_DIV_MAX		8

/* Reference multiplier of each selector, doubled */
static const u8 vita_arm_mul2[16] = {
	3, 3, 3, 6, 8, 12, 16, 24, 32, 36, 24, 3, 9, 18, 32, 36,
};

/* The selectors this driver switches between, lowest rate first */
static const u8 vita_arm_sels[] = { 1, 3, 4, 5, 6, 7, 14, 15 };

/* Row and column of each defined selector in vita_arm_route */
static const s8 vita_arm_route_idx[16] = {
	-1, 0, -1, 1, 2, 3, 4, 5, 6, 7, 8, -1, 9, 10, 11, 12,
};

/*
 * vita_arm_route[from][to]: the selectors to write in turn, the first in the
 * low nibble; the last is the target. Used only when from != to. Rows and
 * columns run 1, 3 to 10, 12 to 15.
 */
static const u16 vita_arm_route[13][13] = {
	/* from 1 */
	{ 0x0000, 0x0003, 0x04c3, 0x0005, 0x06d5, 0x0007, 0x86d5,
	  0x009a, 0x000a, 0x00c3, 0x00d5, 0xe6d5, 0x00f7 },
	/* from 3 */
	{ 0x0001, 0x0000, 0x004c, 0x0005, 0x064c, 0x0007, 0x86d5,
	  0x009a, 0x000a, 0x000c, 0x00d5, 0xe6d5, 0x00f7 },
	/* from 4 */
	{ 0x013c, 0x003c, 0x0000, 0x053c, 0x0006, 0x073c, 0x0008,
	  0x0098, 0x0a3c, 0x000c, 0x00dc, 0x000e, 0x00fe },
	/* from 5 */
	{ 0x0001, 0x0003, 0x04c3, 0x0000, 0x006d, 0x0007, 0x086d,
	  0x009a, 0x000a, 0x00c3, 0x000d, 0x0e6d, 0x00f7 },
	/* from 6 */
	{ 0x015d, 0x035d, 0x0004, 0x005d, 0x0000, 0x075d, 0x0008,
	  0x0098, 0x0a5d, 0x00cd, 0x000d, 0x000e, 0x00fe },
	/* from 7 */
	{ 0x0001, 0x0003, 0x04c3, 0x0005, 0x06d5, 0x0000, 0x86d5,
	  0x009a, 0x000a, 0x0cd5, 0x00d5, 0xe6d5, 0x000f },
	/* from 8 */
	{ 0x15d6, 0x35d6, 0x0004, 0x05d6, 0x0006, 0x75d6, 0x0000,
	  0x0009, 0xa5d6, 0x0cd6, 0x00d6, 0x000e, 0x00f9 },
	/* from 9 */
	{ 0x001a, 0x003a, 0x004c, 0x005d, 0x006d, 0x007a, 0x0008,
	  0x0000, 0x000a, 0x000c, 0x000d, 0x00e8, 0x000f },
	/* from 10 */
	{ 0x0001, 0x0003, 0x0043, 0x0005, 0x06d5, 0x0007, 0x86d5,
	  0x0009, 0x0000, 0x0cd5, 0x00d5, 0xe6d5, 0x00f7 },
	/* from 12 */
	{ 0x0013, 0x0003, 0x0004, 0x0053, 0x0064, 0x0073, 0x086d,
	  0x0009, 0x00a3, 0x0000, 0x000d, 0x0e6d, 0x000f },
	/* from 13 */
	{ 0x0015, 0x0035, 0x004c, 0x0005, 0x0006, 0x0075, 0x0086,
	  0x0009, 0x00a5, 0x000c, 0x0000, 0x00e6, 0x000f },
	/* from 14 */
	{ 0x15d6, 0x35d6, 0x0004, 0x05d6, 0x0006, 0x75d6, 0x0008,
	  0x0098, 0xa5d6, 0x0cd6, 0x00d6, 0x000e, 0x000f },
	/* from 15 */
	{ 0x0017, 0x0037, 0x004c, 0x005d, 0x006d, 0x007a, 0x008e,
	  0x0009, 0x000a, 0x000c, 0x000d, 0x000e, 0x0000 },
};

struct vita_arm_clk {
	struct clk_hw hw;
	void __iomem *base;
	spinlock_t lock;	/* serialises the selector route */
};

#define to_vita_arm_clk(_hw) container_of(_hw, struct vita_arm_clk, hw)

static unsigned long vita_arm_rate(unsigned long parent_rate, unsigned int sel,
				   unsigned int div)
{
	return div_u64((u64)parent_rate * vita_arm_mul2[sel] * (16 - div), 32);
}

static unsigned long vita_arm_recalc_rate(struct clk_hw *hw,
					  unsigned long parent_rate)
{
	struct vita_arm_clk *arm = to_vita_arm_clk(hw);
	u32 sel = readl(arm->base + BASECLK_ARM_SEL) & ARM_SEL_MASK;
	u32 div = readl(arm->base + BASECLK_ARM_DIV);

	return vita_arm_rate(parent_rate, sel, div > ARM_DIV_MAX ? 0 : div);
}

/* The highest rate not above the request, or the lowest rate there is */
static int vita_arm_determine_rate(struct clk_hw *hw,
				   struct clk_rate_request *req)
{
	unsigned long rate = vita_arm_rate(req->best_parent_rate,
					   vita_arm_sels[0], 0);
	int i;

	for (i = 1; i < ARRAY_SIZE(vita_arm_sels); i++) {
		unsigned long r = vita_arm_rate(req->best_parent_rate,
						vita_arm_sels[i], 0);

		if (r <= req->rate)
			rate = r;
	}
	req->rate = rate;
	return 0;
}

static int vita_arm_set_rate(struct clk_hw *hw, unsigned long rate,
			     unsigned long parent_rate)
{
	struct vita_arm_clk *arm = to_vita_arm_clk(hw);
	unsigned int to = 0, from, route, i;
	unsigned long flags;
	u32 val;

	for (i = 0; i < ARRAY_SIZE(vita_arm_sels); i++)
		if (vita_arm_rate(parent_rate, vita_arm_sels[i], 0) == rate)
			to = vita_arm_sels[i];
	if (!to)
		return -EINVAL;

	spin_lock_irqsave(&arm->lock, flags);

	val = readl(arm->base + BASECLK_ARM_SEL);
	from = val & ARM_SEL_MASK;
	if (from != to) {
		if (vita_arm_route_idx[from] < 0) {
			spin_unlock_irqrestore(&arm->lock, flags);
			pr_err("vita-baseclk: no route from ARM selector %u\n", from);
			return -EBUSY;
		}
		route = vita_arm_route[vita_arm_route_idx[from]][vita_arm_route_idx[to]];
		val &= ~ARM_SEL_MASK;
		for (; route; route >>= 4)
			writel(val | (route & ARM_SEL_MASK), arm->base + BASECLK_ARM_SEL);
	}
	writel(0, arm->base + BASECLK_ARM_DIV);

	spin_unlock_irqrestore(&arm->lock, flags);
	return 0;
}

static const struct clk_ops vita_arm_clk_ops = {
	.recalc_rate = vita_arm_recalc_rate,
	.determine_rate = vita_arm_determine_rate,
	.set_rate = vita_arm_set_rate,
};

/*
 * A CLK_OF_DECLARE provider: the Cortex-A9 private timers count PERIPHCLK,
 * the ARM clock divided by two, and need its rate at time_init().
 */
static void __init vita_baseclk_init(struct device_node *np)
{
	struct clk_parent_data parent = { .fw_name = "ref", .index = 0 };
	struct clk_init_data init = {
		.name = "armclk",
		.ops = &vita_arm_clk_ops,
		.parent_data = &parent,
		.num_parents = 1,
	};
	struct clk_hw_onecell_data *data;
	struct vita_arm_clk *arm;
	int ret;

	data = kzalloc_flex(*data, hws, VITA_CLK_ARM + 1);
	arm = kzalloc_obj(*arm);
	if (!data || !arm)
		goto err_free;

	arm->base = of_iomap(np, 0);
	if (!arm->base) {
		pr_err("%pOF: cannot map registers\n", np);
		goto err_free;
	}
	spin_lock_init(&arm->lock);
	arm->hw.init = &init;

	ret = of_clk_hw_register(np, &arm->hw);
	if (ret) {
		pr_err("%pOF: cannot register the ARM clock: %d\n", np, ret);
		goto err_unmap;
	}

	data->num = VITA_CLK_ARM + 1;
	data->hws[VITA_CLK_ARM] = &arm->hw;
	ret = of_clk_add_hw_provider(np, of_clk_hw_onecell_get, data);
	if (ret) {
		pr_err("%pOF: cannot add the clock provider: %d\n", np, ret);
		clk_hw_unregister(&arm->hw);
		goto err_unmap;
	}
	return;

err_unmap:
	iounmap(arm->base);
err_free:
	kfree(arm);
	kfree(data);
}
CLK_OF_DECLARE(vita_baseclk, "sony,vita-baseclk", vita_baseclk_init);
