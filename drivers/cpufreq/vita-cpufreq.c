// SPDX-License-Identifier: GPL-2.0
/*
 * PlayStation Vita CPU frequency scaling
 *
 * The four Cortex-A9 cores share one clock, the ARM clock of
 * ScePervasiveBaseClk (drivers/clk/clk-vita.c), and one rail, VDDA, which the
 * Syscon sets (drivers/regulator/vita-syscon-regulator.c). The voltage a clock
 * level needs differs from chip to chip: each OPP in the device tree names the
 * ScePervasiveVid cell ("vid") ScePower takes for that level, and this driver
 * sets the OPP's voltage to 1.38 V less vid in 10 mV steps, the voltage
 * ScePower asks the Syscon for, before any OPP is used. The OPP core then
 * raises the voltage before the clock and lowers it after, as ScePower does.
 *
 * cpufreq-dt cannot do this: it attaches the regulator itself, which the OPP
 * core allows only before the table is parsed, and the voltages can only be
 * set after.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>

#define VITA_VID_REF_UV		1380000
#define VITA_VID_STEP_UV	10000

static struct {
	struct device *cpu_dev;
	struct cpufreq_frequency_table *freq_table;
	cpumask_var_t cpus;
	unsigned int latency;
	int opp_token;
} vita_cpufreq;

static struct platform_device *vita_cpufreq_pdev;

static int vita_cpufreq_set_target(struct cpufreq_policy *policy,
				   unsigned int index)
{
	return dev_pm_opp_set_rate(vita_cpufreq.cpu_dev,
				   policy->freq_table[index].frequency * 1000UL);
}

static int vita_cpufreq_init(struct cpufreq_policy *policy)
{
	policy->clk = clk_get(vita_cpufreq.cpu_dev, NULL);
	if (IS_ERR(policy->clk))
		return PTR_ERR(policy->clk);

	cpumask_copy(policy->cpus, vita_cpufreq.cpus);
	policy->freq_table = vita_cpufreq.freq_table;
	policy->cpuinfo.transition_latency = vita_cpufreq.latency;
	policy->dvfs_possible_from_any_cpu = true;

	return 0;
}

static void vita_cpufreq_exit(struct cpufreq_policy *policy)
{
	clk_put(policy->clk);
}

static struct cpufreq_driver vita_cpufreq_driver = {
	.flags = CPUFREQ_NEED_INITIAL_FREQ_CHECK,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = vita_cpufreq_set_target,
	.get = cpufreq_generic_get,
	.init = vita_cpufreq_init,
	.exit = vita_cpufreq_exit,
	.name = "vita-cpufreq",
};

static int vita_cpufreq_read_vid(struct device_node *np, u32 *vid)
{
	struct nvmem_cell *cell;
	size_t len;
	void *buf;

	cell = of_nvmem_cell_get(np, "vid");
	if (IS_ERR(cell))
		return PTR_ERR(cell);

	buf = nvmem_cell_read(cell, &len);
	nvmem_cell_put(cell);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	if (len == sizeof(*vid))
		memcpy(vid, buf, sizeof(*vid));
	kfree(buf);

	return len == sizeof(*vid) ? 0 : -EINVAL;
}

static int vita_cpufreq_set_voltages(struct device *cpu_dev)
{
	unsigned long freq;
	int ret;

	for (freq = 0; ; freq++) {
		struct dev_pm_opp *opp = dev_pm_opp_find_freq_ceil(cpu_dev, &freq);
		struct device_node *np;
		unsigned long uv;
		u32 vid;

		if (IS_ERR(opp))
			return 0;

		np = dev_pm_opp_get_of_node(opp);
		dev_pm_opp_put(opp);
		ret = vita_cpufreq_read_vid(np, &vid);
		of_node_put(np);
		if (ret)
			return dev_err_probe(cpu_dev, ret,
					     "%lu kHz: no voltage ID\n", freq / 1000);

		/* An ID the chip does not have reads 0 */
		if (!vid || vid >= VITA_VID_REF_UV / VITA_VID_STEP_UV) {
			dev_warn(cpu_dev, "%lu kHz: voltage ID 0x%x, OPP disabled\n",
				 freq / 1000, vid);
			dev_pm_opp_disable(cpu_dev, freq);
			continue;
		}

		uv = VITA_VID_REF_UV - vid * VITA_VID_STEP_UV;
		ret = dev_pm_opp_adjust_voltage(cpu_dev, freq, uv, uv, uv);
		if (ret)
			return ret;

		dev_info(cpu_dev, "%lu kHz at %lu mV (voltage ID 0x%x)\n",
			 freq / 1000, uv / 1000, vid);
	}
}

static int vita_cpufreq_probe(struct platform_device *pdev)
{
	static const char * const supplies[] = { "cpu", NULL };
	struct device *cpu_dev = get_cpu_device(0);
	int ret;

	if (!cpu_dev)
		return -ENODEV;

	if (!zalloc_cpumask_var(&vita_cpufreq.cpus, GFP_KERNEL))
		return -ENOMEM;

	ret = dev_pm_opp_of_get_sharing_cpus(cpu_dev, vita_cpufreq.cpus);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "no shared OPP table\n");
		goto free_cpus;
	}

	/* The supply has to be attached before the table is parsed */
	ret = dev_pm_opp_set_regulators(cpu_dev, supplies);
	if (ret < 0) {
		dev_err_probe(&pdev->dev, ret, "no CPU supply\n");
		goto free_cpus;
	}
	vita_cpufreq.opp_token = ret;

	ret = dev_pm_opp_of_add_table(cpu_dev);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "no OPP table\n");
		goto put_regulators;
	}

	ret = vita_cpufreq_set_voltages(cpu_dev);
	if (ret)
		goto remove_table;

	ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &vita_cpufreq.freq_table);
	if (ret)
		goto remove_table;

	vita_cpufreq.cpu_dev = cpu_dev;
	vita_cpufreq.latency = dev_pm_opp_get_max_transition_latency(cpu_dev) ?:
			       CPUFREQ_DEFAULT_TRANSITION_LATENCY_NS;

	ret = cpufreq_register_driver(&vita_cpufreq_driver);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "cannot register the driver\n");
		goto free_table;
	}

	return 0;

free_table:
	dev_pm_opp_free_cpufreq_table(cpu_dev, &vita_cpufreq.freq_table);
remove_table:
	dev_pm_opp_of_remove_table(cpu_dev);
put_regulators:
	dev_pm_opp_put_regulators(vita_cpufreq.opp_token);
free_cpus:
	free_cpumask_var(vita_cpufreq.cpus);
	return ret;
}

static void vita_cpufreq_remove(struct platform_device *pdev)
{
	struct device *cpu_dev = vita_cpufreq.cpu_dev;

	cpufreq_unregister_driver(&vita_cpufreq_driver);
	dev_pm_opp_free_cpufreq_table(cpu_dev, &vita_cpufreq.freq_table);
	dev_pm_opp_of_remove_table(cpu_dev);
	dev_pm_opp_put_regulators(vita_cpufreq.opp_token);
	free_cpumask_var(vita_cpufreq.cpus);
}

static struct platform_driver vita_cpufreq_platdrv = {
	.probe = vita_cpufreq_probe,
	.remove = vita_cpufreq_remove,
	.driver = {
		.name = "vita-cpufreq",
	},
};

static const struct of_device_id vita_cpufreq_machines[] = {
	{ .compatible = "sony,vita" },
	{ }
};
MODULE_DEVICE_TABLE(of, vita_cpufreq_machines);

/*
 * The work is done in probe, which waits for the clock, the regulator and
 * the voltage IDs; this only registers the driver and its device.
 */
static int __init vita_cpufreq_module_init(void)
{
	int ret;

	if (!of_machine_device_match(vita_cpufreq_machines))
		return -ENODEV;

	ret = platform_driver_register(&vita_cpufreq_platdrv);
	if (ret)
		return ret;

	vita_cpufreq_pdev = platform_device_register_simple("vita-cpufreq", -1,
							    NULL, 0);
	ret = PTR_ERR_OR_ZERO(vita_cpufreq_pdev);
	if (ret)
		platform_driver_unregister(&vita_cpufreq_platdrv);

	return ret;
}
module_init(vita_cpufreq_module_init);

static void __exit vita_cpufreq_module_exit(void)
{
	platform_device_unregister(vita_cpufreq_pdev);
	platform_driver_unregister(&vita_cpufreq_platdrv);
}
module_exit(vita_cpufreq_module_exit);

MODULE_DESCRIPTION("PlayStation Vita CPU frequency scaling");
MODULE_LICENSE("GPL");
