// SPDX-License-Identifier: GPL-2.0
/*
 * PlayStation Vita core rails, set by Syscon (Ernie) command 0x088E with
 * [rail, value] as sceSysconCtrlVoltageForDriver (NID 0x7F198FA2) sends it.
 * The value is the voltage in 10 mV units:
 * ScePower asks for 0x8A - vid, 1.38 V less the chip's voltage ID in 10 mV
 * steps [WIKI Pervasive, ScePervasiveVid].
 *
 * On boards whose Syscon hardware info byte 2 is below 0x60, or is 0x70,
 * SceSyscon sends values from 0x40 up as 0x8C - value and accepts only
 * 0x4D to 0x8C there, as the Android boot loader's set_voltage() does too.
 * This driver offers 0x4D (0.77 V) to 0x8A (1.38 V) on every board.
 *
 * No Syscon command is known to read a rail back. The boot loader sets the
 * rails from the chip's voltage IDs, so a rail node names the ID it used
 * ("boot-vid"); without one, the rail's voltage is unknown until it is first
 * set, and the regulator core sets it to the rail's minimum when it registers.
 */

#include <linux/mfd/vita-syscon.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/slab.h>

#define SYSCON_CMD_SET_VOLTAGE	0x088e

#define VITA_RAIL_MIN_VALUE	0x4d
#define VITA_RAIL_MAX_VALUE	0x8a
#define VITA_RAIL_STEP_UV	10000
#define VITA_VID_REF_VALUE	0x8a	/* 1.38 V, less the voltage ID */

enum vita_rail {
	VITA_RAIL_VDD = 1,	/* IFTU, DMAC, internal and camera buses */
	VITA_RAIL_VDDA,		/* ARM cores and L2 cache */
	VITA_RAIL_VDDC,		/* codec engine, Venezia */
	VITA_RAIL_VDDG,		/* GPU, GPU crossbar */
	VITA_RAIL_COUNT,
};

struct vita_syscon_regulator {
	struct vita_syscon *syscon;
	unsigned int value[VITA_RAIL_COUNT];	/* 0: not set yet */
};

static unsigned int vita_rail_encode(struct vita_syscon *syscon,
				     unsigned int value)
{
	u32 hw = syscon->hardware_info & 0xff0000;

	if (value >= 0x40 && (hw < 0x600000 || hw == 0x700000))
		return 0x8c - value;
	return value;
}

/* Selectors are the Syscon values themselves */
static int vita_rail_set_voltage_sel(struct regulator_dev *rdev,
				     unsigned int sel)
{
	struct vita_syscon_regulator *vreg = rdev_get_drvdata(rdev);
	struct vita_syscon *syscon = vreg->syscon;
	int rail = rdev_get_id(rdev);
	int ret;

	ret = syscon->short_command_write(syscon, SYSCON_CMD_SET_VOLTAGE,
					  rail | vita_rail_encode(syscon, sel) << 8, 2);
	if (ret)
		return ret;

	vreg->value[rail] = sel;
	return 0;
}

static int vita_rail_get_voltage_sel(struct regulator_dev *rdev)
{
	struct vita_syscon_regulator *vreg = rdev_get_drvdata(rdev);

	return vreg->value[rdev_get_id(rdev)] ?: -ENOTRECOVERABLE;
}

static const struct regulator_ops vita_rail_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.set_voltage_sel = vita_rail_set_voltage_sel,
	.get_voltage_sel = vita_rail_get_voltage_sel,
};

#define VITA_RAIL(_id, _name)						\
	{								\
		.name = _name,						\
		.of_match = _name,					\
		.id = _id,						\
		.ops = &vita_rail_ops,					\
		.type = REGULATOR_VOLTAGE,				\
		.owner = THIS_MODULE,					\
		.n_voltages = VITA_RAIL_MAX_VALUE + 1,			\
		.linear_min_sel = VITA_RAIL_MIN_VALUE,			\
		.min_uV = VITA_RAIL_MIN_VALUE * VITA_RAIL_STEP_UV,	\
		.uV_step = VITA_RAIL_STEP_UV,				\
	}

static const struct regulator_desc vita_rails[] = {
	VITA_RAIL(VITA_RAIL_VDD, "vdd"),
	VITA_RAIL(VITA_RAIL_VDDA, "vdda"),
	VITA_RAIL(VITA_RAIL_VDDC, "vddc"),
	VITA_RAIL(VITA_RAIL_VDDG, "vddg"),
};

/* The value the boot loader set a rail to, or 0 if the node does not say */
static int vita_rail_boot_value(struct device *dev, const char *name,
				unsigned int *value)
{
	struct device_node *np __free(device_node) =
		of_get_child_by_name(dev->of_node, name);
	struct nvmem_cell *cell;
	size_t len;
	u32 *vid;

	*value = 0;
	if (!np)
		return 0;

	cell = of_nvmem_cell_get(np, "boot-vid");
	if (IS_ERR(cell)) {
		if (PTR_ERR(cell) == -ENOENT)
			return 0;
		return dev_err_probe(dev, PTR_ERR(cell),
				     "%s: cannot get the boot voltage ID\n", name);
	}

	vid = nvmem_cell_read(cell, &len);
	nvmem_cell_put(cell);
	if (IS_ERR(vid))
		return dev_err_probe(dev, PTR_ERR(vid),
				     "%s: cannot read the boot voltage ID\n", name);

	if (len == sizeof(*vid) && *vid > 0 &&
	    VITA_VID_REF_VALUE - *vid >= VITA_RAIL_MIN_VALUE)
		*value = VITA_VID_REF_VALUE - *vid;
	else
		dev_warn(dev, "%s: boot voltage ID 0x%x ignored\n", name,
			 len == sizeof(*vid) ? *vid : 0);
	kfree(vid);

	return 0;
}

static int vita_syscon_regulator_probe(struct platform_device *pdev)
{
	struct regulator_config config = { .dev = &pdev->dev };
	struct vita_syscon_regulator *vreg;
	int i, ret;

	vreg = devm_kzalloc(&pdev->dev, sizeof(*vreg), GFP_KERNEL);
	if (!vreg)
		return -ENOMEM;

	vreg->syscon = dev_get_drvdata(pdev->dev.parent);
	config.driver_data = vreg;

	for (i = 0; i < ARRAY_SIZE(vita_rails); i++) {
		struct regulator_dev *rdev;

		ret = vita_rail_boot_value(&pdev->dev, vita_rails[i].name,
					   &vreg->value[vita_rails[i].id]);
		if (ret)
			return ret;

		rdev = devm_regulator_register(&pdev->dev, &vita_rails[i], &config);
		if (IS_ERR(rdev))
			return dev_err_probe(&pdev->dev, PTR_ERR(rdev),
					     "cannot register %s\n",
					     vita_rails[i].name);
	}

	return 0;
}

static struct platform_driver vita_syscon_regulator_driver = {
	.probe = vita_syscon_regulator_probe,
	.driver = {
		.name = "vita-syscon-regulator",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};
module_platform_driver(vita_syscon_regulator_driver);

MODULE_DESCRIPTION("PlayStation Vita Syscon core rail regulators");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:vita-syscon-regulator");
