// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 Sergi Granell

#include <linux/gpio/driver.h>
#include <linux/gpio/generic.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/pm_runtime.h>
#include <linux/spinlock.h>

#define VITA_GPIO_DIRECTION		0x00
/* Port mode: 1 = output, 0 = input (wiki GPIO_Registers, "Port mode") */
#define  VITA_GPIO_DIRECTION_OUT	1
#define  VITA_GPIO_DIRECTION_IN		0
#define VITA_GPIO_READ			0x04
#define VITA_GPIO_SET			0x08
#define VITA_GPIO_CLEAR			0x0C
#define VITA_GPIO_INT_MODE_0_15		0x14
#define VITA_GPIO_INT_MODE_16_31	0x18
#define  VITA_GPIO_INT_MODE_HLEVEL_SENS	0
#define  VITA_GPIO_INT_MODE_LLEVEL_SENS	1
#define  VITA_GPIO_INT_MODE_REDGE	2
#define  VITA_GPIO_INT_MODE_FEDGE	3
#define VITA_GPIO_INT_MASK_GATE0	0x1C
#define VITA_GPIO_INT_MASK_GATE1	0x20
#define VITA_GPIO_INT_MASK_GATE2	0x24
#define VITA_GPIO_INT_MASK_GATE3	0x28
#define VITA_GPIO_INT_MASK_GATE4	0x2C
#define VITA_GPIO_READ_LATCH		0x34
#define VITA_GPIO_INT_STATUS_GATE0	0x38
#define VITA_GPIO_INT_STATUS_GATE1	0x3C
#define VITA_GPIO_INT_STATUS_GATE2	0x40
#define VITA_GPIO_INT_STATUS_GATE3	0x44
#define VITA_GPIO_INT_STATUS_GATE4	0x48

struct vita_gpio_chip {
	struct gpio_generic_chip	gen_gc;
	void __iomem		*regs;
	struct clk 		*clk;
	struct reset_control    *rst;
};

static void vita_gpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct vita_gpio_chip *vgpio = gpiochip_get_data(chip);
	unsigned long flags;
	u32 val;

	gpio_generic_chip_lock_irqsave(&vgpio->gen_gc, flags);

	val = ioread32(vgpio->regs + VITA_GPIO_INT_MASK_GATE0);
	val |= BIT(d->hwirq);
	iowrite32(val, vgpio->regs + VITA_GPIO_INT_MASK_GATE0);

	gpio_generic_chip_unlock_irqrestore(&vgpio->gen_gc, flags);

	gpiochip_disable_irq(chip, irqd_to_hwirq(d));
}

static void vita_gpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct vita_gpio_chip *vgpio = gpiochip_get_data(chip);
	unsigned long flags;
	u32 val;

	gpiochip_enable_irq(chip, irqd_to_hwirq(d));

	gpio_generic_chip_lock_irqsave(&vgpio->gen_gc, flags);

	val = ioread32(vgpio->regs + VITA_GPIO_INT_MASK_GATE0);
	val &= ~BIT(d->hwirq);
	iowrite32(val, vgpio->regs + VITA_GPIO_INT_MASK_GATE0);

	gpio_generic_chip_unlock_irqrestore(&vgpio->gen_gc, flags);
}

static int vita_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct vita_gpio_chip *vgpio = gpiochip_get_data(chip);
	unsigned long flags;
	int mode;
	u32 val;
	u32 reg;
	u32 shift;

	if (d->hwirq < 16) {
		reg = VITA_GPIO_INT_MODE_0_15;
		shift = d->hwirq * 2;
	} else {
		reg = VITA_GPIO_INT_MODE_16_31;
		shift = (d->hwirq - 16) * 2;
	}

	if (type == IRQ_TYPE_EDGE_RISING) {
		mode = VITA_GPIO_INT_MODE_REDGE;
	} else if (type == IRQ_TYPE_EDGE_FALLING) {
		mode = VITA_GPIO_INT_MODE_FEDGE;
	} else if (type == IRQ_TYPE_LEVEL_HIGH) {
		mode = VITA_GPIO_INT_MODE_HLEVEL_SENS;
	} else if (type == IRQ_TYPE_LEVEL_LOW) {
		mode = VITA_GPIO_INT_MODE_LLEVEL_SENS;
	} else {
		/* No both-edges mode in the INT_MODE field */
		return -EINVAL;
	}

	if (type & IRQ_TYPE_LEVEL_MASK)
		irq_set_handler_locked(d, handle_level_irq);
	else
		irq_set_handler_locked(d, handle_edge_irq);

	gpio_generic_chip_lock_irqsave(&vgpio->gen_gc, flags);

	val = ioread32(vgpio->regs + reg);
	val &= ~(3 << shift);
	val |= mode << shift;
	iowrite32(val, vgpio->regs + reg);

	gpio_generic_chip_unlock_irqrestore(&vgpio->gen_gc, flags);

	return 0;
}

static void vita_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *chip = irq_desc_get_handler_data(desc);
	struct vita_gpio_chip *vgpio = gpiochip_get_data(chip);
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	unsigned long masked, status, valid;
	int hwirq;

	masked = ioread32(vgpio->regs + VITA_GPIO_INT_MASK_GATE0);
	status = ioread32(vgpio->regs + VITA_GPIO_INT_STATUS_GATE0);
	valid = status & ~masked;

	//pr_info("vita_gpio_irq_handler, int status: 0x%08X\n", status);

	chained_irq_enter(irqchip, desc);

	for_each_set_bit(hwirq, &valid, chip->ngpio) {
		generic_handle_domain_irq(chip->irq.domain, hwirq);
		iowrite32(BIT(hwirq), vgpio->regs + VITA_GPIO_INT_STATUS_GATE0);
	}

	chained_irq_exit(irqchip, desc);
}

static const struct irq_chip vita_gpio_irqchip = {
	.name		= "vita-gpio",
	.irq_mask	= vita_gpio_irq_mask,
	.irq_unmask	= vita_gpio_irq_unmask,
	.irq_set_type	= vita_gpio_irq_set_type,
	.flags		= IRQCHIP_IMMUTABLE,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static int vita_gpio_probe(struct platform_device *pdev)
{
	struct vita_gpio_chip *vgpio;
	struct gpio_irq_chip *girq;
	struct gpio_generic_chip_config config;
	struct device *dev = &pdev->dev;
	int ret, irq;
	u32 num_gpios = 32;

	vgpio = devm_kzalloc(dev, sizeof(*vgpio), GFP_KERNEL);
	if (!vgpio)
		return -ENOMEM;

	vgpio->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vgpio->regs))
		return PTR_ERR(vgpio->regs);

	of_property_read_u32(pdev->dev.of_node, "ngpios", &num_gpios);
	if (num_gpios > 32) {
		dev_err(dev, "ngpios must be less or equal 32\n");
		return -EINVAL;
	}

	/* Reset GPIO status here? */

	/* DIRECTION: a set bit makes the port an output ("dirout") */
	config = (struct gpio_generic_chip_config) {
		.dev = dev,
		.sz = 4,
		.dat = vgpio->regs + VITA_GPIO_READ,
		.set = vgpio->regs + VITA_GPIO_SET,
		.clr = vgpio->regs + VITA_GPIO_CLEAR,
		.dirout = vgpio->regs + VITA_GPIO_DIRECTION,
	};

	ret = gpio_generic_chip_init(&vgpio->gen_gc, &config);
	if (ret) {
		dev_err(dev, "Failed to register generic gpio, %d\n",
			ret);
		return ret;
	}

	vgpio->gen_gc.gc.label = dev_name(dev);
	vgpio->gen_gc.gc.ngpio = num_gpios;
	vgpio->gen_gc.gc.parent = dev;
	vgpio->gen_gc.gc.base = -1;
	vgpio->gen_gc.gc.owner = THIS_MODULE;

	vgpio->clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(vgpio->clk)) {
		ret = PTR_ERR(vgpio->clk);
		dev_err(dev,
			"Failed to retrieve peripheral clock, %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(vgpio->clk);
	if (ret) {
		dev_err(dev,
			"Failed to enable the peripheral clock, %d\n", ret);
		return ret;
	}

	vgpio->rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(vgpio->rst)) {
		ret = PTR_ERR(vgpio->rst);
		goto err_disable_clk;
	}
	reset_control_deassert(vgpio->rst);

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		return irq ? irq : -ENODEV;

	girq = &vgpio->gen_gc.gc.irq;
	gpio_irq_chip_set_chip(girq, &vita_gpio_irqchip);
	girq->parent_handler = vita_gpio_irq_handler;
	girq->num_parents = 1;
	girq->parents = devm_kcalloc(dev, 1,
				     sizeof(*girq->parents),
				     GFP_KERNEL);
	if (!girq->parents) {
		ret = -ENOMEM;
		goto err_reset;
	}
	girq->parents[0] = irq;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_level_irq;

	ret = devm_gpiochip_add_data(dev, &vgpio->gen_gc.gc, vgpio);
	if (ret < 0) {
		dev_err(dev, "Could not register gpiochip, %d\n", ret);
		goto err_reset;
	}

	platform_set_drvdata(pdev, vgpio);

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return 0;

err_reset:
	reset_control_assert(vgpio->rst);

err_disable_clk:
	clk_disable_unprepare(vgpio->clk);

	return ret;
}

static void vita_gpio_remove(struct platform_device *pdev)
{
	struct vita_gpio_chip *vgpio = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	pm_runtime_get_sync(dev);

	reset_control_assert(vgpio->rst);

	clk_disable_unprepare(vgpio->clk);

	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
}

static const struct of_device_id vita_of_ids[] = {
	{ .compatible = "vita,gpio" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, vita_of_ids);

static struct platform_driver vita_gpio_driver = {
	.driver = {
		.name = "vita-gpio",
		.of_match_table = vita_of_ids,
	},
	.probe = vita_gpio_probe,
	.remove = vita_gpio_remove,
};
module_platform_driver(vita_gpio_driver);

MODULE_AUTHOR("Sergi Granell");
MODULE_DESCRIPTION("PlayStation Vita GPIO driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:vita-gpio");
