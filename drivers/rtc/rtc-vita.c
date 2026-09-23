// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 Sergi Granell

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/mfd/vita-syscon.h>
#include <linux/unaligned.h>

#define SECONDS_FROM_YEAR_1_TO_1970 62135596800

/*
 * Syscon command 0x82 takes an alarm in the half-second units of the 0x11 counter,
 * 0xffffffff for none, and a flag byte from Baryon 0x90907 on (SceRtc passes 0).
 * Command 0x12 reads it back.  Syscon powers the Vita on when it expires, so the
 * alarm only matters while suspended or off: Linux's alarm timers run on hrtimers
 * while awake and program the RTC on suspend.  Syscon's alarm event is not handled,
 * so an alarm that expires while Linux runs raises no RTC interrupt.
 */
#define SYSCON_ALARM_NONE	0xffffffff

struct vita_rtc {
	struct rtc_device *rtc;
	struct vita_syscon *syscon;
	bool alarm_enabled;
	u32 alarm_ticks;
};

/* Microseconds since year 1 at which the 0x11 counter was 0, as SceRtc keeps it. */
static int vita_rtc_base_usec(struct vita_syscon *syscon, u64 *base)
{
	u32 tick[2];
	int ret;

	ret = syscon->scratchpad_read(syscon, 0x10, tick, 8);
	if (ret < 0)
		return ret;

	*base = ((u64)tick[0] << 19) | (((u64)tick[1] & 0xFF) << 51);

	return 0;
}

static int vita_rtc_read_ticks(struct vita_syscon *syscon, u16 cmd, u32 *ticks)
{
	u8 rx[SYSCON_RX_HEADER_SIZE + 5 + 1];
	int ret;

	ret = syscon->command_read(syscon, cmd, rx, sizeof(rx));
	if (ret < 0)
		return ret;

	*ticks = get_unaligned_le32(&rx[SYSCON_RX_DATA]);

	return 0;
}

static void vita_rtc_ticks_to_tm(u64 base, u32 ticks, struct rtc_time *tm)
{
	time64_t time = div_u64(base + (u64)ticks * 500000, 1000000);

	rtc_time64_to_tm(time - SECONDS_FROM_YEAR_1_TO_1970, tm);
}

static int vita_syscon_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct vita_rtc *vrtc = dev_get_drvdata(dev);
	struct vita_syscon *syscon = vrtc->syscon;
	u32 syscon_on_hsec;
	u64 base;
	int ret;

	ret = vita_rtc_base_usec(syscon, &base);
	if (ret < 0)
		return ret;

	ret = vita_rtc_read_ticks(syscon, 0x11, &syscon_on_hsec);
	if (ret < 0)
		return ret;

	if (div_u64(base + (u64)syscon_on_hsec * 500000, 1000000) <
	    SECONDS_FROM_YEAR_1_TO_1970)
		return -EINVAL;

	vita_rtc_ticks_to_tm(base, syscon_on_hsec, tm);

	return 0;
}

static int vita_rtc_write_alarm(struct vita_rtc *vrtc, u32 ticks)
{
	struct vita_syscon *syscon = vrtc->syscon;
	u8 tx[SYSCON_TX_HEADER_SIZE + 5 + 1];
	u8 rx[16];

	if (syscon->baryon_version < 0x90907)
		return syscon->short_command_write(syscon, 0x82, ticks, 5);

	tx[SYSCON_TX_CMD_LO] = 0x82;
	tx[SYSCON_TX_CMD_HI] = 0;
	tx[SYSCON_TX_LENGTH] = 6;
	put_unaligned_le32(ticks, &tx[SYSCON_TX_DATA(0)]);
	tx[SYSCON_TX_DATA(4)] = 0;

	return syscon->transfer(syscon, tx, rx, sizeof(rx));
}

static int vita_syscon_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct vita_rtc *vrtc = dev_get_drvdata(dev);
	struct vita_syscon *syscon = vrtc->syscon;
	u32 ticks;
	u64 base;
	int ret;

	ret = vita_rtc_base_usec(syscon, &base);
	if (ret < 0)
		return ret;

	ret = vita_rtc_read_ticks(syscon, 0x12, &ticks);
	if (ret < 0)
		return ret;

	if (ticks == SYSCON_ALARM_NONE)
		ticks = vrtc->alarm_ticks;
	alrm->enabled = vrtc->alarm_enabled;
	vita_rtc_ticks_to_tm(base, ticks, &alrm->time);

	return 0;
}

static int vita_syscon_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct vita_rtc *vrtc = dev_get_drvdata(dev);
	u64 base, usec;
	u64 ticks;
	int ret;

	ret = vita_rtc_base_usec(vrtc->syscon, &base);
	if (ret < 0)
		return ret;

	usec = (u64)(rtc_tm_to_time64(&alrm->time) + SECONDS_FROM_YEAR_1_TO_1970) *
	       1000000;
	if (usec < base)
		return -EINVAL;

	/* SceRtc rounds up by one half second, so the alarm never fires early. */
	ticks = div_u64(usec - base, 500000) + 1;
	if (ticks >= SYSCON_ALARM_NONE)
		return -ERANGE;

	ret = vita_rtc_write_alarm(vrtc, alrm->enabled ? ticks : SYSCON_ALARM_NONE);
	if (ret < 0)
		return ret;

	vrtc->alarm_ticks = ticks;
	vrtc->alarm_enabled = alrm->enabled;

	return 0;
}

static int vita_syscon_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct vita_rtc *vrtc = dev_get_drvdata(dev);
	int ret;

	if (enabled && vrtc->alarm_ticks == 0)
		return -EINVAL;

	ret = vita_rtc_write_alarm(vrtc, enabled ? vrtc->alarm_ticks : SYSCON_ALARM_NONE);
	if (ret < 0)
		return ret;

	vrtc->alarm_enabled = enabled;

	return 0;
}

static const struct rtc_class_ops vita_rtc_ops = {
	.read_time = vita_syscon_rtc_read_time,
	.read_alarm = vita_syscon_rtc_read_alarm,
	.set_alarm = vita_syscon_rtc_set_alarm,
	.alarm_irq_enable = vita_syscon_rtc_alarm_irq_enable,
};

static int vita_syscon_rtc_probe(struct platform_device *pdev)
{
	struct vita_rtc *vrtc;
	struct device *dev = &pdev->dev;
	struct vita_syscon *syscon = dev_get_drvdata(dev->parent);

	vrtc = devm_kzalloc(&pdev->dev, sizeof(*vrtc), GFP_KERNEL);
	if (!vrtc)
		return -ENOMEM;

	vrtc->rtc = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(vrtc->rtc))
		return PTR_ERR(vrtc->rtc);

	vrtc->syscon = syscon;
	vrtc->rtc->ops = &vita_rtc_ops;
	vrtc->rtc->range_max = U64_MAX;

	platform_set_drvdata(pdev, vrtc);

	/*
	 * Syscon's alarm powers the Vita on, which is what the alarm timers
	 * (CLOCK_BOOTTIME_ALARM, used by Bluetooth and AlarmManager) require of the RTC.
	 */
	device_init_wakeup(dev, true);
	clear_bit(RTC_FEATURE_UPDATE_INTERRUPT, vrtc->rtc->features);

	return devm_rtc_register_device(vrtc->rtc);
}

static const struct of_device_id vita_syscon_rtc_of_match[] = {
	{ .compatible = "vita,syscon-rtc" },
	{ },
};
MODULE_DEVICE_TABLE(of, vita_syscon_rtc_of_match);

static struct platform_driver vita_syscon_rtc_driver = {
	.driver = {
		.name = "vita-syscon-rtc",
		.of_match_table = vita_syscon_rtc_of_match,
	},
	.probe = vita_syscon_rtc_probe,
};
module_platform_driver(vita_syscon_rtc_driver);

MODULE_AUTHOR("Sergi Granell");
MODULE_DESCRIPTION("PlayStation Vita Syscon RTC driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:vita-syscon-rtc");
