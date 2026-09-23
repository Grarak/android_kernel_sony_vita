// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 Sergi Granell
/*
 * PlayStation Vita front touch screen and rear touch pad, read through the
 * Syscon (Ernie).
 *
 * Syscon command 0x300 + port (1 front, 2 rear) returns one panel's report:
 * 0x3c bytes of six-byte contacts (id, whose bit 7 marks an empty slot;
 * 11-bit x; 11-bit y; force). SceTouch (touch.skprx 3.65, the report decoder
 * at 0x81004510) clamps the front panel to 0..1919 x 0..1087, twice the
 * 960x544 screen, and the rear one to x 0..1919 and raw y 0..781, which it
 * shifts by 108 into the front panel's frame.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/vita-syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/input/touchscreen.h>

#define TOUCH_PORT_FRONT (1 << 0)
#define TOUCH_PORT_BACK  (1 << 1)

#define TOUCH_MAX_REPORT_FRONT 6
#define TOUCH_MAX_REPORT_BACK  4

#define TOUCH_REPORT_DATA_SIZE      (0x7a - 2)

#define TOUCH_FRONT_MAX_X	1919
#define TOUCH_FRONT_MAX_Y	1087
#define TOUCH_BACK_MAX_X	1919
#define TOUCH_BACK_MAX_Y	781
#define TOUCH_FORCE_MAX		255

static bool rear = true;
module_param(rear, bool, 0444);
MODULE_PARM_DESC(rear, "Report the rear touch pad as a second input device");

struct syscon_touchpanel_device_info {
	u16 front_vendor_id;
	u16 front_fw_version;
	u16 back_vendor_id;
	u16 back_fw_version;
};

struct syscon_touchpanel_device_info_ext {
	u16 front_vendor_id;
	u16 front_fw_version;
	u16 front_unk1;
	u8 front_unk2;
	u8 front_unk3;
	u16 unused1;
	u16 back_vendor_id;
	u16 back_fw_version;
	u16 back_unk1;
	u8 back_unk2;
	u8 back_unk3;
	u16 unused2;
};

struct vita_ts_vendor_data {
	u16 vendor_id;
	u16 fw_version;
	u16 unk1;
	u8 unk2;
	u8 unk3;
};

struct vita_vendor_data {
	struct vita_ts_vendor_data front;
	struct vita_ts_vendor_data back;
	u8 front_vendor_id_non_dependant;
	u8 back_vendor_id_dependant;
	u32 syscon_hw_version_dependant;
};

struct vita_syscon_ts {
	struct vita_syscon *syscon;
	struct input_dev *input;	/* front touch screen */
	struct input_dev *rear_input;	/* rear touch pad, if enabled */
	struct device *dev;
	struct vita_vendor_data vendor;
	struct touchscreen_properties prop;
	struct mutex lock;		/* serialises open/close */
	unsigned int users;
	int port_mask;
};

struct touch_finger {
	u8 id; // bit 8: invalid
	u8 force;
	u16 x; // 11 bits
	u16 y; // 11 bits
};

static int syscon_ctrl_device_reset(struct vita_syscon *syscon, unsigned int param_1,
				    unsigned int param_2)
{
	return syscon->short_command_write(syscon, 0x88F, param_2 | (param_1 << 8), 3);
}

static int syscon_get_touchpanel_device_info(struct vita_syscon *syscon,
					     struct syscon_touchpanel_device_info *info)
{
	unsigned char rx[SYSCON_RX_HEADER_SIZE + sizeof(*info) + 1];
	u8 *data = &rx[SYSCON_RX_DATA];
	int ret;

	ret = syscon->command_read(syscon, 0x380, rx, sizeof(rx));
	if (ret < 0)
		return ret;

	info->front_vendor_id  = (data[0] << 8) | data[1];
	info->front_fw_version = (data[2] << 8) | data[3];
	info->back_vendor_id   = (data[4] << 8) | data[5];
	info->back_fw_version  = (data[6] << 8) | data[7];

	return 0;
}

static int syscon_get_touchpanel_device_info_ext(struct vita_syscon *syscon,
						 struct syscon_touchpanel_device_info_ext *info)
{
	u8 rx[SYSCON_RX_HEADER_SIZE + sizeof(*info) + 1];
	u8 *data = &rx[SYSCON_RX_DATA];
	int ret;

	ret = syscon->command_read(syscon, 0x390, rx, sizeof(rx));
	if (ret < 0)
		return ret;

	info->front_vendor_id  = (data[0] << 8) | data[1];
	info->front_fw_version = (data[2] << 8) | data[3];
	info->front_unk1       = (data[4] << 8) | data[5];
	info->front_unk2       = data[6];
	info->front_unk3       = data[7];
	info->unused1          = (data[9] << 8) | data[8];
	info->back_vendor_id   = (data[10] << 8) | data[11];
	info->back_fw_version  = (data[12] << 8) | data[13];
	info->back_unk1        = (data[14] << 8) | data[15];
	info->back_unk2        = data[16];
	info->back_unk3        = data[17];
	info->unused2          = (data[19] << 8) | data[18];

	return 0;
}

static int syscon_get_touchpanel_unk_info_front(struct vita_syscon *syscon, u16 *data)
{
	int ret;
	u8 rx[SYSCON_RX_HEADER_SIZE + sizeof(u16) + 1];

	ret = syscon->command_read(syscon, 0x3a7, rx, sizeof(rx));

	*data = rx[SYSCON_RX_DATA] | ((u16)rx[SYSCON_RX_DATA + 1] << 8);

	return ret;
}

static int syscon_get_touchpanel_unk_info_back(struct vita_syscon *syscon, u16 *data)
{
	int ret;
	u8 rx[SYSCON_RX_HEADER_SIZE + sizeof(u16) + 1];

	ret = syscon->command_read(syscon, 0x3b7, rx, sizeof(rx));

	*data = rx[SYSCON_RX_DATA] | ((u16)rx[SYSCON_RX_DATA + 1] << 8);

	return ret;
}

static int syscon_touch_set_sampling_cycle(struct vita_syscon *syscon, int cycles_front,
					   int cycles_back)
{
	u8 tx[SYSCON_TX_HEADER_SIZE + 8 + 1];
	u8 rx[16];

	tx[SYSCON_TX_CMD_LO] = 0x87;
	tx[SYSCON_TX_CMD_HI] = 3;
	tx[SYSCON_TX_LENGTH] = 9;
	tx[SYSCON_TX_DATA(0)] = (cycles_front >= 0) ? 1 : 0;
	tx[SYSCON_TX_DATA(1)] = (cycles_front >= 0) ? (cycles_front & 0xFF) : 0;
	tx[SYSCON_TX_DATA(2)] = 0;
	tx[SYSCON_TX_DATA(3)] = 0;
	tx[SYSCON_TX_DATA(4)] = (cycles_back >= 0) ? 1 : 0;
	tx[SYSCON_TX_DATA(5)] = (cycles_back >= 0) ? (cycles_back & 0xFF) : 0;
	tx[SYSCON_TX_DATA(6)] = 0;
	tx[SYSCON_TX_DATA(7)] = 0;

	return syscon->transfer(syscon, tx, rx, sizeof(rx));
}

static int touch_read_panel_info_1(struct vita_syscon *syscon,
				   struct vita_ts_vendor_data *front,
				   struct vita_ts_vendor_data *back)
{
	struct syscon_touchpanel_device_info info;
	u16 data;
	int ret;

	ret = syscon_get_touchpanel_device_info(syscon, &info);
	if (ret < 0)
		return ret;

	front->vendor_id = info.front_vendor_id;
	front->fw_version = info.front_fw_version;
	back->vendor_id = info.back_vendor_id;
	back->fw_version = info.back_fw_version;

	if (syscon->baryon_version > 0x90002) {
		ret = syscon_get_touchpanel_unk_info_front(syscon, &data);
		if (ret < 0)
			return ret;

		front->unk1 = (data >> 8) | (data << 8);

		ret = syscon_get_touchpanel_unk_info_back(syscon, &data);
		if (ret < 0)
			return ret;

		back->unk1 = (data >> 8) | (data << 8);
	}

	return 0;
}

static int touch_read_panel_info_2(struct vita_syscon *syscon,
				   struct vita_ts_vendor_data *front,
				   struct vita_ts_vendor_data *back)
{
	struct syscon_touchpanel_device_info_ext info;
	int ret;

	ret = syscon_get_touchpanel_device_info_ext(syscon, &info);
	if (ret < 0)
		return ret;

	front->vendor_id = info.front_vendor_id;
	front->fw_version = info.front_fw_version;
	front->unk1 = info.front_unk1;
	front->unk2 = info.front_unk2;
	front->unk3 = info.front_unk3;
	back->vendor_id = info.back_vendor_id;
	back->fw_version = info.back_fw_version;
	back->unk1 = info.back_unk1;
	back->unk2 = info.back_unk2;
	back->unk3 = info.back_unk3;

	return 0;
}

static int touch_init(struct vita_syscon_ts *ts)
{
	struct vita_syscon *syscon = ts->syscon;
	uint32_t baryon_version = syscon->baryon_version;
	uint32_t hw_info = syscon->hardware_info;
	int ret;

	ret = syscon_ctrl_device_reset(syscon, 0xC, 0);
	if (ret < 0)
		return ret;

	usleep_range(1000, 2000);

	ret = syscon_ctrl_device_reset(syscon, 0xC, 1);
	if (ret < 0)
		return ret;

	msleep(100);

	if (baryon_version < 0x1000600) {
		ret = touch_read_panel_info_1(syscon, &ts->vendor.front, &ts->vendor.back);
	} else {
		ret = touch_read_panel_info_2(syscon, &ts->vendor.front, &ts->vendor.back);
	}
	if (ret < 0)
		return ret;

	ts->vendor.front_vendor_id_non_dependant = 0xe0;
	if (ts->vendor.back.vendor_id == 0x800a)
		ts->vendor.back_vendor_id_dependant = 0xd0;
	else
		ts->vendor.back_vendor_id_dependant = 0xb0;

	if (baryon_version > 0x90002) {
		ret = syscon_touch_set_sampling_cycle(syscon, 0, 0);
		if (ret < 0)
			return ret;
	}

	if (hw_info & 0x10000)
		ts->vendor.syscon_hw_version_dependant = 0;
	else
		ts->vendor.syscon_hw_version_dependant = 1;

	return 0;
}

static int touch_configure(struct vita_syscon_ts *ts, int port_mask,
			   u8 max_report_front, u8 max_report_back)
{
	struct vita_syscon *syscon = ts->syscon;
	u8 tx[SYSCON_TX_HEADER_SIZE + 4 + 1];
	u8 rx[16];
	u8 front_cfg, back_cfg;

	port_mask &= 3;

	if (port_mask == 0) {
		front_cfg = 0;
		back_cfg = 0;
	} else {
		if ((port_mask == 2) || (port_mask == 3)) {
			front_cfg = 0;
			back_cfg = ts->vendor.back_vendor_id_dependant;
		} else if (port_mask == 1) {
			front_cfg = ts->vendor.front_vendor_id_non_dependant;
			back_cfg = 0;
		} else {
			front_cfg = 0;
			back_cfg = 0;
		}
	}

	if (max_report_front > TOUCH_MAX_REPORT_FRONT)
		max_report_front = TOUCH_MAX_REPORT_FRONT;

	if (max_report_back > TOUCH_MAX_REPORT_BACK)
		max_report_back = TOUCH_MAX_REPORT_BACK;

	tx[SYSCON_TX_CMD_LO] = 0x81;
	tx[SYSCON_TX_CMD_HI] = 3;
	tx[SYSCON_TX_LENGTH] = 5;
	tx[SYSCON_TX_DATA(0)] = port_mask & 1;
	tx[SYSCON_TX_DATA(1)] = front_cfg | max_report_front;
	tx[SYSCON_TX_DATA(2)] = (port_mask >> 1) & 1;
	tx[SYSCON_TX_DATA(3)] = back_cfg | max_report_back;

	return syscon->transfer(syscon, tx, rx, sizeof(rx));
}

static int touch_set_sampling_cycle(struct vita_syscon_ts *ts, int port_mask,
				    u8 cycles_front, u8 cycles_back)
{
	struct vita_syscon *syscon = ts->syscon;
	u8 tx[SYSCON_TX_HEADER_SIZE + 8 + 1];
	u8 rx[16];

	tx[SYSCON_TX_CMD_LO] = 0x87;
	tx[SYSCON_TX_CMD_HI] = 3;
	tx[SYSCON_TX_LENGTH] = 9;
	tx[SYSCON_TX_DATA(0)] = port_mask & 1;
	tx[SYSCON_TX_DATA(1)] = cycles_front;
	tx[SYSCON_TX_DATA(2)] = 0;
	tx[SYSCON_TX_DATA(3)] = 0;
	tx[SYSCON_TX_DATA(4)] = (port_mask >> 1) & 1;
	tx[SYSCON_TX_DATA(5)] = cycles_back;
	tx[SYSCON_TX_DATA(6)] = 0;
	tx[SYSCON_TX_DATA(7)] = 0;

	return syscon->transfer(syscon, tx, rx, sizeof(rx));
}

static inline bool touch_parse_finger(struct touch_finger *finger, const u8 *data)
{
	u8 id = data[0];
	if (id & 0x80)
		return false;

	finger->id = id;
	finger->force = data[5];
	finger->x = data[1] | (((u16)data[2] & 7) << 8);
	finger->y = data[3] | (((u16)data[4] & 7) << 8);

	return true;
}

/*
 * Reads one panel's contacts (port TOUCH_PORT_FRONT or TOUCH_PORT_BACK) with
 * command 0x301 or 0x302. Only the bytes the reply actually carries (its
 * length field counts the result byte, the data and the checksum) are
 * parsed, so a short reply means no contacts rather than stale buffer data.
 */
static int touch_read_port(struct vita_syscon_ts *ts, int port,
			   struct touch_finger *fingers, int max_fingers)
{
	struct vita_syscon *syscon = ts->syscon;
	u8 buffer[SYSCON_RX_HEADER_SIZE + TOUCH_REPORT_DATA_SIZE + 1];
	u8 *raw_data = &buffer[SYSCON_RX_DATA];
	int data_len, offset, count = 0;
	int i, ret;

	memset(buffer, 0, sizeof(buffer));

	ret = syscon->command_read(syscon, 0x300 + port, buffer, sizeof(buffer));
	if (ret < 0)
		return ret;

	if (buffer[SYSCON_RX_RESULT] & 0x20)	/* command error */
		return 0;

	data_len = (int)buffer[SYSCON_RX_LENGTH] - 2;
	data_len = clamp(data_len, 0, TOUCH_REPORT_DATA_SIZE);

	for (i = 0, offset = 0; i < max_fingers && offset + 6 <= data_len;
	     i++, offset += 6) {
		if (touch_parse_finger(&fingers[count], &raw_data[offset]))
			count++;
	}

	return count;
}

static void vita_syscon_ts_report(struct input_dev *input_dev,
				  const struct touch_finger *fingers, int count,
				  int max_x, int max_y)
{
	int i, slot;

	for (i = 0; i < count; i++) {
		/* The panel's contact id tracks a finger across reports */
		slot = input_mt_get_slot_by_key(input_dev, fingers[i].id);
		if (slot < 0)
			continue;

		input_mt_slot(input_dev, slot);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);
		input_report_abs(input_dev, ABS_MT_POSITION_X,
				 min_t(int, fingers[i].x, max_x));
		input_report_abs(input_dev, ABS_MT_POSITION_Y,
				 min_t(int, fingers[i].y, max_y));
		input_report_abs(input_dev, ABS_MT_PRESSURE, fingers[i].force);
	}

	input_mt_sync_frame(input_dev);
	input_sync(input_dev);
}

static void vita_syscon_ts_poll(struct input_dev *input_dev)
{
	struct vita_syscon_ts *ts = input_get_drvdata(input_dev);
	struct touch_finger fingers[TOUCH_MAX_REPORT_FRONT];
	int count;

	count = touch_read_port(ts, TOUCH_PORT_FRONT, fingers,
				TOUCH_MAX_REPORT_FRONT);
	if (count < 0)
		return;

	vita_syscon_ts_report(input_dev, fingers, count,
			      TOUCH_FRONT_MAX_X, TOUCH_FRONT_MAX_Y);
}

static void vita_syscon_ts_rear_poll(struct input_dev *input_dev)
{
	struct vita_syscon_ts *ts = input_get_drvdata(input_dev);
	struct touch_finger fingers[TOUCH_MAX_REPORT_BACK];
	int count;

	count = touch_read_port(ts, TOUCH_PORT_BACK, fingers,
				TOUCH_MAX_REPORT_BACK);
	if (count < 0)
		return;

	vita_syscon_ts_report(input_dev, fingers, count,
			      TOUCH_BACK_MAX_X, TOUCH_BACK_MAX_Y);
}

static int vita_syscon_ts_start(struct vita_syscon_ts *ts)
{
	int ret;

	ret = touch_configure(ts, ts->port_mask, TOUCH_MAX_REPORT_FRONT,
			      (ts->port_mask & TOUCH_PORT_BACK) ?
			      TOUCH_MAX_REPORT_BACK : 0);
	if (ret < 0)
		return ret;

	return touch_set_sampling_cycle(ts, ts->port_mask, 0xFF,
					(ts->port_mask & TOUCH_PORT_BACK) ?
					0xFF : 0);
}

/*
 * Both input devices share the panels' Syscon device (reset line 0xC): it is
 * released when the first of them is opened and held in reset after the last
 * one is closed.
 */
static int vita_syscon_ts_open(struct input_dev *input)
{
	struct vita_syscon_ts *ts = input_get_drvdata(input);
	int ret = 0;

	mutex_lock(&ts->lock);
	if (ts->users++ == 0) {
		syscon_ctrl_device_reset(ts->syscon, 0xC, 1);
		msleep(100);
		ret = vita_syscon_ts_start(ts);
		if (ret < 0)
			ts->users--;
	}
	mutex_unlock(&ts->lock);

	return ret < 0 ? ret : 0;
}

static void vita_syscon_ts_close(struct input_dev *input)
{
	struct vita_syscon_ts *ts = input_get_drvdata(input);

	mutex_lock(&ts->lock);
	if (--ts->users == 0) {
		syscon_ctrl_device_reset(ts->syscon, 0xC, 0);
		usleep_range(1000, 2000);
	}
	mutex_unlock(&ts->lock);
}

static struct input_dev *vita_syscon_ts_alloc(struct vita_syscon_ts *ts,
					      const char *name, const char *phys,
					      int max_x, int max_y,
					      int max_contacts, unsigned int mt_flags,
					      void (*poll)(struct input_dev *))
{
	struct input_dev *input_dev;
	int ret;

	input_dev = devm_input_allocate_device(ts->dev);
	if (!input_dev)
		return ERR_PTR(-ENOMEM);

	input_set_drvdata(input_dev, ts);

	input_dev->name = name;
	input_dev->phys = phys;
	input_dev->dev.parent = ts->dev;
	input_dev->id.bustype = BUS_SPI;
	input_dev->open = vita_syscon_ts_open;
	input_dev->close = vita_syscon_ts_close;

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, TOUCH_FORCE_MAX, 0, 0);

	ret = input_mt_init_slots(input_dev, max_contacts, mt_flags);
	if (ret) {
		dev_err(ts->dev, "Failed to initialize MT slots: %d\n", ret);
		return ERR_PTR(ret);
	}

	ret = input_setup_polling(input_dev, poll);
	if (ret)
		return ERR_PTR(ret);

	input_set_poll_interval(input_dev, 16);

	return input_dev;
}

static int vita_syscon_ts_probe(struct platform_device *pdev)
{
	int ret;
	struct vita_syscon_ts *ts;
	struct input_dev *input_dev;
	struct device *dev = &pdev->dev;
	struct vita_syscon *syscon = dev_get_drvdata(dev->parent);

	ts = devm_kzalloc(dev, sizeof(struct vita_syscon_ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->syscon = syscon;
	ts->dev = dev;
	mutex_init(&ts->lock);
	ts->port_mask = TOUCH_PORT_FRONT | (rear ? TOUCH_PORT_BACK : 0);

	/* Front: a direct touch screen over the display */
	input_dev = vita_syscon_ts_alloc(ts, "Vita Front Touchscreen",
					 "vita_syscon_ts/input0",
					 TOUCH_FRONT_MAX_X, TOUCH_FRONT_MAX_Y,
					 TOUCH_MAX_REPORT_FRONT,
					 INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED,
					 vita_syscon_ts_poll);
	if (IS_ERR(input_dev))
		return PTR_ERR(input_dev);
	ts->input = input_dev;

	touchscreen_parse_properties(input_dev, true, &ts->prop);

	/* Rear: an indirect multi-touch pad */
	if (rear) {
		input_dev = vita_syscon_ts_alloc(ts, "Vita Rear Touchpad",
						 "vita_syscon_ts/input1",
						 TOUCH_BACK_MAX_X, TOUCH_BACK_MAX_Y,
						 TOUCH_MAX_REPORT_BACK,
						 INPUT_MT_POINTER | INPUT_MT_DROP_UNUSED,
						 vita_syscon_ts_rear_poll);
		if (IS_ERR(input_dev))
			return PTR_ERR(input_dev);
		ts->rear_input = input_dev;
	}

	ret = touch_init(ts);
	if (ret < 0)
		return ret;

	ret = vita_syscon_ts_start(ts);
	if (ret < 0)
		return ret;

	ret = input_register_device(ts->input);
	if (ret)
		return ret;

	if (ts->rear_input) {
		ret = input_register_device(ts->rear_input);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct of_device_id vita_syscon_ts_of_match[] = {
	{ .compatible = "vita,syscon-ts" },
	{ },
};
MODULE_DEVICE_TABLE(of, vita_syscon_ts_of_match);

static struct platform_driver vita_syscon_ts_driver = {
	.driver = {
		.name = "vita-syscon-ts",
		.of_match_table = vita_syscon_ts_of_match,
	},
	.probe = vita_syscon_ts_probe,
};
module_platform_driver(vita_syscon_ts_driver);

MODULE_AUTHOR("Sergi Granell");
MODULE_DESCRIPTION("PlayStation Vita Touchscreen Driver (Syscon)");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:vita-syscon-ts");
