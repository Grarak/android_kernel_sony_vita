// SPDX-License-Identifier: GPL-2.0
/*
 * PlayStation Vita system timer (SceSystimer) as the clocksource.
 *
 * The Cortex-A9 global and private timers count PERIPHCLK, which is the ARM
 * clock divided by two, so they change rate with it. This uses one of the
 * SoC's long-range timers instead: a 64-bit up-counter on a fixed 48 MHz
 * source, from which the Vita OS derives its own 1 MHz system time.
 *
 * Each long-range timer is a 0x1000-byte page (SceSystimer):
 *   +0x00/+0x04  count, low and high word
 *   +0x08/+0x0c  compare, low and high word
 *   +0x18        status, write 1 to clear: bit 0 overflow, bit 1 match
 *   +0x1c        control: bit 0 run, bits 2-4 match interrupt and mode,
 *                bits 23:20 clock source, bits 31:24 prescaler (source / (n + 1))
 * Units 0 to 4 belong to the Vita OS's SceSystimer and are free once Linux
 * runs; unit 5 is the Vita OS's system time, and the unit to use is given
 * by the device tree. Clock source 3 is the 48 MHz clock.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/clocksource.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/sched_clock.h>

#define LT_COUNT_LO		0x00
#define LT_COUNT_HI		0x04
#define LT_COMPARE_LO		0x08
#define LT_COMPARE_HI		0x0c
#define LT_STATUS		0x18
#define LT_CTRL			0x1c

#define LT_CTRL_RUN		BIT(0)
#define LT_CTRL_CLKSRC(n)	((n) << 20)
#define LT_CTRL_PRESCALE(n)	((n) << 24)
#define LT_CLKSRC_48MHZ		3
#define LT_STATUS_ALL		(BIT(0) | BIT(1))

static void __iomem *vita_lt;

/* The high word is read on both sides of the low word to catch a carry. */
static u64 notrace vita_lt_read(void)
{
	u32 hi, lo, again;

	hi = readl_relaxed(vita_lt + LT_COUNT_HI);
	do {
		again = hi;
		lo = readl_relaxed(vita_lt + LT_COUNT_LO);
		hi = readl_relaxed(vita_lt + LT_COUNT_HI);
	} while (hi != again);
	return (u64)hi << 32 | lo;
}

static u64 vita_cs_read(struct clocksource *cs)
{
	return vita_lt_read();
}

static struct clocksource vita_cs = {
	.name	= "vita-systimer",
	.rating	= 350,
	.read	= vita_cs_read,
	.mask	= CLOCKSOURCE_MASK(64),
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

static unsigned long vita_delay_read(void)
{
	return readl_relaxed(vita_lt + LT_COUNT_LO);
}

static struct delay_timer vita_delay = {
	.read_current_timer = vita_delay_read,
};

static int __init vita_systimer_init(struct device_node *np)
{
	unsigned long rate;
	struct clk *clk;
	u32 start;
	int i, ret;

	vita_lt = of_iomap(np, 0);
	if (!vita_lt)
		return -ENXIO;

	clk = of_clk_get(np, 0);
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		goto unmap;
	}
	ret = clk_prepare_enable(clk);
	if (ret)
		goto put;
	rate = clk_get_rate(clk);

	/* stopped, from zero, a compare it never reaches, no status, then run */
	writel_relaxed(0, vita_lt + LT_CTRL);
	writel_relaxed(0, vita_lt + LT_COUNT_LO);
	writel_relaxed(0, vita_lt + LT_COUNT_HI);
	writel_relaxed(~0u, vita_lt + LT_COMPARE_HI);
	writel_relaxed(~0u, vita_lt + LT_COMPARE_LO);
	writel_relaxed(LT_STATUS_ALL, vita_lt + LT_STATUS);
	writel(LT_CTRL_PRESCALE(0) | LT_CTRL_CLKSRC(LT_CLKSRC_48MHZ) | LT_CTRL_RUN,
	       vita_lt + LT_CTRL);

	/* a gated or unclocked unit would stop time */
	start = readl_relaxed(vita_lt + LT_COUNT_LO);
	for (i = 0; i < 1000000; i++)
		if (readl_relaxed(vita_lt + LT_COUNT_LO) != start)
			break;
	if (i == 1000000) {
		pr_err("%pOF: counter does not move (control %08x)\n", np,
		       readl_relaxed(vita_lt + LT_CTRL));
		ret = -ENODEV;
		goto disable;
	}

	sched_clock_register(vita_lt_read, 64, rate);
	vita_delay.freq = rate;
	register_current_timer_delay(&vita_delay);
	ret = clocksource_register_hz(&vita_cs, rate);
	if (ret)
		goto disable;
	pr_info("%pOF: clocksource at %lu Hz\n", np, rate);
	return 0;

disable:
	writel_relaxed(0, vita_lt + LT_CTRL);
	clk_disable_unprepare(clk);
put:
	clk_put(clk);
unmap:
	iounmap(vita_lt);
	return ret;
}
TIMER_OF_DECLARE(vita_systimer, "sony,vita-systimer", vita_systimer_init);
