// SPDX-License-Identifier: GPL-2.0
/*
 * Power sequence for the PlayStation Vita's WLAN/BT module
 *
 * The module (a Marvell 88W8787 on SceSdif2, "Robin") has no GPIOs of its
 * own: its supply and its reset line belong to the Syscon (Ernie). SceWlanBt
 * (wlanbt.skprx, 3.65) brings it up as follows (FUN_81000168):
 *
 *	sceClockgenWlanBtClkEnableForDriver();		27 MHz reference clock
 *	ksceKernelDelayThread(5000);
 *	sceSysconCtrlWirelessPower2ForDriver(1);	Syscon command 0x88A
 *	sceSysconCtrlDeviceResetForDriver(0x10, 0);	Syscon command 0x88F
 *	ksceKernelDelayThread(10000);
 *	sceSysconCtrlDeviceResetForDriver(0x10, 1);
 *	ksceKernelDelayThread(10000);
 *
 * and powers it down in the reverse order, followed by 500 ms
 * (FUN_810001c4). Device 0x10 of command 0x88F is the WLAN/BT module and
 * mode 0 holds it in reset (wiki SceSyscon, sceSysconCtrlDeviceResetForDriver).
 *
 * The reference clock comes from the I2C clock generator (SceClockgen: bit 7
 * of register 1 of the P1P40167 derivative at 0xD2 on I2C bus 0). Linux has
 * no driver for the Vita's I2C controllers, so the Android boot loader turns
 * it on; an optional "ext_clock" is enabled here once there is a clock
 * driver to provide it.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/mfd/vita-syscon.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "pwrseq.h"

#define SYSCON_CMD_WIRELESS_POWER	0x88A
#define SYSCON_CMD_DEVICE_RESET		0x88F
#define SYSCON_DEVICE_WLANBT		0x10

struct mmc_pwrseq_vita_wlanbt {
	struct mmc_pwrseq pwrseq;
	struct vita_syscon *syscon;
	struct clk *ext_clk;
	bool clk_enabled;
};

#define to_pwrseq_vita(p) container_of(p, struct mmc_pwrseq_vita_wlanbt, pwrseq)

static int vita_wlanbt_power(struct mmc_pwrseq_vita_wlanbt *pwrseq, bool on)
{
	/* One data byte (length 2 includes the checksum) */
	return pwrseq->syscon->short_command_write(pwrseq->syscon,
						  SYSCON_CMD_WIRELESS_POWER,
						  on, 2);
}

static int vita_wlanbt_reset_release(struct mmc_pwrseq_vita_wlanbt *pwrseq,
				     bool release)
{
	/* Two data bytes: the mode, then the device */
	return pwrseq->syscon->short_command_write(pwrseq->syscon,
						  SYSCON_CMD_DEVICE_RESET,
						  release | (SYSCON_DEVICE_WLANBT << 8),
						  3);
}

static void mmc_pwrseq_vita_wlanbt_pre_power_on(struct mmc_host *host)
{
	struct mmc_pwrseq_vita_wlanbt *pwrseq = to_pwrseq_vita(host->pwrseq);
	struct device *dev = pwrseq->pwrseq.dev;
	int ret;

	if (!IS_ERR(pwrseq->ext_clk) && !pwrseq->clk_enabled) {
		if (!clk_prepare_enable(pwrseq->ext_clk))
			pwrseq->clk_enabled = true;
		usleep_range(5000, 6000);
	}

	ret = vita_wlanbt_power(pwrseq, true);
	if (ret < 0)
		dev_err(dev, "Syscon wireless power on failed: %d\n", ret);

	ret = vita_wlanbt_reset_release(pwrseq, false);
	if (ret < 0)
		dev_err(dev, "Syscon WLAN/BT reset assert failed: %d\n", ret);

	usleep_range(10000, 11000);
}

static void mmc_pwrseq_vita_wlanbt_post_power_on(struct mmc_host *host)
{
	struct mmc_pwrseq_vita_wlanbt *pwrseq = to_pwrseq_vita(host->pwrseq);
	int ret;

	ret = vita_wlanbt_reset_release(pwrseq, true);
	if (ret < 0)
		dev_err(pwrseq->pwrseq.dev,
			"Syscon WLAN/BT reset release failed: %d\n", ret);

	usleep_range(10000, 11000);
}

static void mmc_pwrseq_vita_wlanbt_power_off(struct mmc_host *host)
{
	struct mmc_pwrseq_vita_wlanbt *pwrseq = to_pwrseq_vita(host->pwrseq);

	vita_wlanbt_reset_release(pwrseq, false);
	vita_wlanbt_power(pwrseq, false);

	if (pwrseq->clk_enabled) {
		clk_disable_unprepare(pwrseq->ext_clk);
		pwrseq->clk_enabled = false;
	}
}

static const struct mmc_pwrseq_ops mmc_pwrseq_vita_wlanbt_ops = {
	.pre_power_on = mmc_pwrseq_vita_wlanbt_pre_power_on,
	.post_power_on = mmc_pwrseq_vita_wlanbt_post_power_on,
	.power_off = mmc_pwrseq_vita_wlanbt_power_off,
};

static int mmc_pwrseq_vita_wlanbt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mmc_pwrseq_vita_wlanbt *pwrseq;

	/* The Syscon adds this cell for every model; only a DT node is used */
	if (!dev->of_node)
		return -ENODEV;

	pwrseq = devm_kzalloc(dev, sizeof(*pwrseq), GFP_KERNEL);
	if (!pwrseq)
		return -ENOMEM;

	pwrseq->syscon = dev_get_drvdata(dev->parent);
	if (!pwrseq->syscon)
		return -EPROBE_DEFER;

	pwrseq->ext_clk = devm_clk_get_optional(dev, "ext_clock");
	if (IS_ERR(pwrseq->ext_clk))
		return dev_err_probe(dev, PTR_ERR(pwrseq->ext_clk),
				     "failed to get ext_clock\n");
	if (!pwrseq->ext_clk)
		pwrseq->ext_clk = ERR_PTR(-ENOENT);

	pwrseq->pwrseq.dev = dev;
	pwrseq->pwrseq.ops = &mmc_pwrseq_vita_wlanbt_ops;
	pwrseq->pwrseq.owner = THIS_MODULE;
	platform_set_drvdata(pdev, pwrseq);

	return mmc_pwrseq_register(&pwrseq->pwrseq);
}

static void mmc_pwrseq_vita_wlanbt_remove(struct platform_device *pdev)
{
	struct mmc_pwrseq_vita_wlanbt *pwrseq = platform_get_drvdata(pdev);

	mmc_pwrseq_unregister(&pwrseq->pwrseq);
}

static const struct of_device_id mmc_pwrseq_vita_wlanbt_of_match[] = {
	{ .compatible = "vita,syscon-wlanbt-pwrseq" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mmc_pwrseq_vita_wlanbt_of_match);

static struct platform_driver mmc_pwrseq_vita_wlanbt_driver = {
	.probe = mmc_pwrseq_vita_wlanbt_probe,
	.remove = mmc_pwrseq_vita_wlanbt_remove,
	.driver = {
		.name = "pwrseq_vita_wlanbt",
		.of_match_table = mmc_pwrseq_vita_wlanbt_of_match,
	},
};
module_platform_driver(mmc_pwrseq_vita_wlanbt_driver);

MODULE_DESCRIPTION("PlayStation Vita WLAN/BT module power sequence (Syscon)");
MODULE_LICENSE("GPL");
