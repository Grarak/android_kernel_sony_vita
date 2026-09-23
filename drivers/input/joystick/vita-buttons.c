// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 Sergi Granell

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/mfd/vita-syscon.h>

/*
 * Syscon command 0x101 answers four button bytes and four analog bytes (left
 * stick x/y, right stick x/y, 0x80 at rest). The bits below are positions in
 * the little-endian word of the button bytes. SceCtrl (ctrl.skprx 3.65,
 * CtrlSysconPollDone at 0x81002314) moves them into its own layout and then
 * XORs that with 0x4037fcf9: every button is active low except the bits of
 * the last byte (headphone detect and two unknown ones). ACTIVE_LOW_MASK is
 * the same inversion in this layout.
 */
#define CTRL_UP		BIT(0)
#define CTRL_RIGHT	BIT(1)
#define CTRL_DOWN	BIT(2)
#define CTRL_LEFT	BIT(3)
#define CTRL_TRIANGLE	BIT(4)
#define CTRL_CIRCLE	BIT(5)
#define CTRL_CROSS	BIT(6)
#define CTRL_SQUARE	BIT(7)
#define CTRL_SELECT	BIT(8)
#define CTRL_LTRIGGER	BIT(9)
#define CTRL_RTRIGGER	BIT(10)
#define CTRL_START	BIT(11)
#define CTRL_PSBUTTON	BIT(12)
#define CTRL_POWER	BIT(14)
#define CTRL_VOLUP	BIT(16)
#define CTRL_VOLDOWN	BIT(17)
#define CTRL_HEADPHONE	BIT(27)

#define ACTIVE_LOW_MASK	0x00037fff

#define STICK_FUZZ	2
#define STICK_FLAT	12

static const struct {
	u32 bit;
	unsigned int code;
} vita_buttons_map[] = {
	{ CTRL_UP,		BTN_DPAD_UP },
	{ CTRL_DOWN,		BTN_DPAD_DOWN },
	{ CTRL_LEFT,		BTN_DPAD_LEFT },
	{ CTRL_RIGHT,		BTN_DPAD_RIGHT },
	{ CTRL_CROSS,		BTN_SOUTH },	/* BTN_A */
	{ CTRL_CIRCLE,		BTN_EAST },	/* BTN_B */
	{ CTRL_SQUARE,		BTN_WEST },	/* BTN_Y (BTN_X is north) */
	{ CTRL_TRIANGLE,	BTN_NORTH },
	{ CTRL_LTRIGGER,	BTN_TL },
	{ CTRL_RTRIGGER,	BTN_TR },
	{ CTRL_SELECT,		BTN_SELECT },
	{ CTRL_START,		BTN_START },
	{ CTRL_PSBUTTON,	KEY_HOMEPAGE },
	{ CTRL_VOLUP,		KEY_VOLUMEUP },
	{ CTRL_VOLDOWN,		KEY_VOLUMEDOWN },
	{ CTRL_POWER,		KEY_POWER },
};

struct vita_syscon_buttons {
	struct vita_syscon *syscon;
	struct input_dev *input;
	struct device *dev;
};

static void vita_syscon_buttons_poll(struct input_dev *input)
{
	struct vita_syscon_buttons *pad = input_get_drvdata(input);
	u8 data[SYSCON_RX_HEADER_SIZE + 8 + 1];
	u8 *payload = &data[SYSCON_RX_DATA];
	u32 buttons;
	int i, ret;

	memset(data, 0, sizeof(data));

	/* 0x104 would add the analog pressure of the arrow keys */
	ret = pad->syscon->command_read(pad->syscon, 0x101, data, sizeof(data));
	if (ret < 0 || data[SYSCON_RX_LENGTH] < 8 + 2)
		return;

	buttons = (payload[0] | (payload[1] << 8) | (payload[2] << 16) |
		   (payload[3] << 24)) ^ ACTIVE_LOW_MASK;

	input_report_abs(input, ABS_X, payload[4]);
	input_report_abs(input, ABS_Y, payload[5]);
	input_report_abs(input, ABS_RX, payload[6]);
	input_report_abs(input, ABS_RY, payload[7]);

	for (i = 0; i < ARRAY_SIZE(vita_buttons_map); i++)
		input_report_key(input, vita_buttons_map[i].code,
				 !!(buttons & vita_buttons_map[i].bit));

	input_sync(input);
}

static int vita_syscon_buttons_probe(struct platform_device *pdev)
{
	int err, i;
	struct vita_syscon_buttons *pad;
	struct input_dev *idev;
	struct device *dev = &pdev->dev;
	struct vita_syscon *syscon = dev_get_drvdata(dev->parent);

	pad = devm_kzalloc(dev, sizeof(struct vita_syscon_buttons), GFP_KERNEL);
	if (!pad)
		return -ENOMEM;

	pad->syscon = syscon;
	pad->dev = dev;

	idev = devm_input_allocate_device(dev);
	if (!idev)
		return -ENOMEM;

	pad->input = idev;
	input_set_drvdata(idev, pad);
	idev->name = "Vita Buttons";
	idev->phys = "vita_syscon_buttons";
	idev->dev.parent = pad->dev;
	idev->id.bustype = BUS_SPI;

	input_set_abs_params(idev, ABS_X, 0, 255, STICK_FUZZ, STICK_FLAT);
	input_set_abs_params(idev, ABS_Y, 0, 255, STICK_FUZZ, STICK_FLAT);
	input_set_abs_params(idev, ABS_RX, 0, 255, STICK_FUZZ, STICK_FLAT);
	input_set_abs_params(idev, ABS_RY, 0, 255, STICK_FUZZ, STICK_FLAT);

	for (i = 0; i < ARRAY_SIZE(vita_buttons_map); i++)
		input_set_capability(idev, EV_KEY, vita_buttons_map[i].code);

	err = input_setup_polling(idev, vita_syscon_buttons_poll);
	if (err) {
		dev_err(pad->dev, "failed to set up polling: %d\n", err);
		return err;
	}

	input_set_poll_interval(idev, 16);

	err = input_register_device(idev);
	if (err) {
		dev_err(pad->dev,
			"failed to register input device: %d\n", err);
		return err;
	}

	return 0;
}

static const struct of_device_id vita_syscon_buttons_of_match[] = {
	{ .compatible = "vita,syscon-buttons" },
	{ },
};
MODULE_DEVICE_TABLE(of, vita_syscon_buttons_of_match);

static struct platform_driver vita_syscon_buttons_driver = {
	.driver = {
		.name = "vita-syscon-buttons",
		.of_match_table = vita_syscon_buttons_of_match,
	},
	.probe = vita_syscon_buttons_probe,
};
module_platform_driver(vita_syscon_buttons_driver);

MODULE_AUTHOR("Sergi Granell");
MODULE_DESCRIPTION("PlayStation Vita buttons driver");
MODULE_LICENSE("GPL");
