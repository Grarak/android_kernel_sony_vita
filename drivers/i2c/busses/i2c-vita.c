// SPDX-License-Identifier: GPL-2.0-only
/*
 * PlayStation Vita I2C controllers (SceI2c0Reg 0xE0500000, SceI2c1Reg
 * 0xE0510000).
 *
 * The programming model is the one SceLowio's own driver uses (ksceI2cInit and
 * the ksceI2cTransfer* exports).  Each transfer runs as a sequence of commands
 * written to +0x14, each of which completes with a status bit in +0x28 and an
 * interrupt:
 *
 *   data phase   (len << 8) | 0x02 (write), | 0x13 (read)   status bit 0
 *   stop         0x04                                       status bit 2
 *   restart      0x05                                       status bit 3
 *   reset        0x07       (completes without an interrupt; +0x1C reads 0)
 *
 * The 7-bit address goes to +0x10, write data to the FIFO at +0x00 and read
 * data comes from the FIFO at +0x04, 64 bytes at most per phase.  The
 * interrupt handler writes the status it read back to +0x28.  lowio treats
 * status bit 1 and bits 8, 9 and 11..15 as errors; which one is a NAK is not
 * established, so all of them are reported as -EIO.
 *
 * The Pervasive gate clock of each controller (+0x110 + 4 * bus) is expected
 * to be on: the Vita OS has it on when the boot loader takes over.
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#define VITA_I2C_TXFIFO		0x00
#define VITA_I2C_RXFIFO		0x04
#define VITA_I2C_CTRL0		0x08	/* lowio writes 1 before every transfer */
#define VITA_I2C_CTRL1		0x0c	/* likewise */
#define VITA_I2C_ADDR		0x10	/* 7-bit target address */
#define VITA_I2C_CMD		0x14
#define VITA_I2C_SPEED		0x18
#define VITA_I2C_BUSY		0x1c
#define VITA_I2C_STATUS		0x28	/* write the bits back to clear them */
#define VITA_I2C_INTMASK	0x2c

#define VITA_I2C_CMD_WRITE	0x02
#define VITA_I2C_CMD_READ	0x13
#define VITA_I2C_CMD_STOP	0x04
#define VITA_I2C_CMD_RESTART	0x05
#define VITA_I2C_CMD_RESET	0x07
#define VITA_I2C_CMD_LEN(n)	((n) << 8)

#define VITA_I2C_ST_DATA	BIT(0)
#define VITA_I2C_ST_STOP	BIT(2)
#define VITA_I2C_ST_RESTART	BIT(3)
#define VITA_I2C_ST_WAITED	0xfb0f	/* the bits lowio's event flag waits on */
#define VITA_I2C_ST_ERRORS	(VITA_I2C_ST_WAITED & ~(VITA_I2C_ST_DATA | \
				 VITA_I2C_ST_STOP | VITA_I2C_ST_RESTART))

/* +0x2C during the reset command, and afterwards (SceLowio ksceI2cInit) */
#define VITA_I2C_INTMASK_RESET	0x0100f70f
#define VITA_I2C_INTMASK_RUN	0x01000000

#define VITA_I2C_FIFO_SIZE	64
#define VITA_I2C_TIMEOUT	msecs_to_jiffies(100)

struct vita_i2c {
	struct i2c_adapter adap;
	struct device *dev;
	void __iomem *regs;
	struct reset_control *rst;
	struct completion done;
	u32 status;
	u32 speed;
};

static irqreturn_t vita_i2c_irq(int irq, void *data)
{
	struct vita_i2c *i2c = data;
	u32 status = readl(i2c->regs + VITA_I2C_STATUS);

	if (!status)
		return IRQ_NONE;
	writel(status, i2c->regs + VITA_I2C_STATUS);
	i2c->status |= status;
	complete(&i2c->done);
	return IRQ_HANDLED;
}

/* Run one command and wait for the status bit it completes with. */
static int vita_i2c_command(struct vita_i2c *i2c, u32 cmd, u32 expect)
{
	u32 status;

	i2c->status = 0;
	reinit_completion(&i2c->done);
	writel(cmd, i2c->regs + VITA_I2C_CMD);
	if (!wait_for_completion_timeout(&i2c->done, VITA_I2C_TIMEOUT)) {
		dev_dbg(i2c->dev, "command 0x%x timed out\n", cmd);
		return -ETIMEDOUT;
	}
	status = i2c->status & VITA_I2C_ST_WAITED;
	if (status != expect) {
		dev_dbg(i2c->dev, "command 0x%x: status 0x%x\n", cmd, status);
		return -EIO;
	}
	return 0;
}

static int vita_i2c_msg(struct vita_i2c *i2c, struct i2c_msg *msg)
{
	int ret;
	u16 i;

	writel(1, i2c->regs + VITA_I2C_CTRL0);
	writel(1, i2c->regs + VITA_I2C_CTRL1);
	writel(msg->addr, i2c->regs + VITA_I2C_ADDR);

	if (!(msg->flags & I2C_M_RD)) {
		for (i = 0; i < msg->len; i++)
			writel(msg->buf[i], i2c->regs + VITA_I2C_TXFIFO);
		return vita_i2c_command(i2c, VITA_I2C_CMD_LEN(msg->len) | VITA_I2C_CMD_WRITE,
					VITA_I2C_ST_DATA);
	}

	ret = vita_i2c_command(i2c, VITA_I2C_CMD_LEN(msg->len) | VITA_I2C_CMD_READ,
			       VITA_I2C_ST_DATA);
	if (ret)
		return ret;
	for (i = 0; i < msg->len; i++)
		msg->buf[i] = readl(i2c->regs + VITA_I2C_RXFIFO);
	return 0;
}

static int vita_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct vita_i2c *i2c = i2c_get_adapdata(adap);
	int ret = 0, stop;
	int i;

	for (i = 0; i < num && !ret; i++) {
		if (i)
			ret = vita_i2c_command(i2c, VITA_I2C_CMD_RESTART, VITA_I2C_ST_RESTART);
		if (!ret)
			ret = vita_i2c_msg(i2c, &msgs[i]);
	}
	/* lowio ends every transfer with a stop, whatever the data phase returned. */
	stop = vita_i2c_command(i2c, VITA_I2C_CMD_STOP, VITA_I2C_ST_STOP);
	if (ret)
		return ret;
	return stop ? stop : num;
}

static u32 vita_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

static const struct i2c_algorithm vita_i2c_algo = {
	.xfer = vita_i2c_xfer,
	.functionality = vita_i2c_func,
};

static const struct i2c_adapter_quirks vita_i2c_quirks = {
	.flags = I2C_AQ_NO_ZERO_LEN,
	.max_write_len = VITA_I2C_FIFO_SIZE,
	.max_read_len = VITA_I2C_FIFO_SIZE,
};

static int vita_i2c_hw_init(struct vita_i2c *i2c)
{
	u32 busy;
	int ret;

	writel(VITA_I2C_INTMASK_RESET, i2c->regs + VITA_I2C_INTMASK);
	writel(1, i2c->regs + VITA_I2C_CTRL0);
	writel(1, i2c->regs + VITA_I2C_CTRL1);
	writel(VITA_I2C_CMD_RESET, i2c->regs + VITA_I2C_CMD);
	ret = readl_poll_timeout(i2c->regs + VITA_I2C_BUSY, busy, !busy, 10, 100000);
	if (ret)
		return ret;
	writel(readl(i2c->regs + VITA_I2C_STATUS), i2c->regs + VITA_I2C_STATUS);
	writel(i2c->speed, i2c->regs + VITA_I2C_SPEED);
	writel(VITA_I2C_INTMASK_RUN, i2c->regs + VITA_I2C_INTMASK);
	return 0;
}

static int vita_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vita_i2c *i2c;
	int irq, ret;

	i2c = devm_kzalloc(dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;
	i2c->dev = dev;
	init_completion(&i2c->done);

	i2c->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(i2c->regs))
		return PTR_ERR(i2c->regs);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	if (of_property_read_u32(dev->of_node, "sony,speed-mode", &i2c->speed))
		i2c->speed = 5;

	i2c->rst = devm_reset_control_get_optional_exclusive_deasserted(dev, NULL);
	if (IS_ERR(i2c->rst))
		return dev_err_probe(dev, PTR_ERR(i2c->rst), "reset\n");

	ret = vita_i2c_hw_init(i2c);
	if (ret)
		return dev_err_probe(dev, ret, "controller stays busy after reset\n");

	ret = devm_request_irq(dev, irq, vita_i2c_irq, 0, dev_name(dev), i2c);
	if (ret)
		return ret;

	i2c->adap.owner = THIS_MODULE;
	i2c->adap.algo = &vita_i2c_algo;
	i2c->adap.quirks = &vita_i2c_quirks;
	i2c->adap.dev.parent = dev;
	i2c->adap.dev.of_node = dev->of_node;
	strscpy(i2c->adap.name, dev_name(dev), sizeof(i2c->adap.name));
	i2c_set_adapdata(&i2c->adap, i2c);
	platform_set_drvdata(pdev, i2c);

	return devm_i2c_add_adapter(dev, &i2c->adap);
}

static int vita_i2c_resume(struct device *dev)
{
	return vita_i2c_hw_init(dev_get_drvdata(dev));
}

static DEFINE_SIMPLE_DEV_PM_OPS(vita_i2c_pm_ops, NULL, vita_i2c_resume);

static const struct of_device_id vita_i2c_of_match[] = {
	{ .compatible = "sony,vita-i2c" },
	{ }
};
MODULE_DEVICE_TABLE(of, vita_i2c_of_match);

static struct platform_driver vita_i2c_driver = {
	.probe = vita_i2c_probe,
	.driver = {
		.name = "vita-i2c",
		.of_match_table = vita_i2c_of_match,
		.pm = pm_sleep_ptr(&vita_i2c_pm_ops),
	},
};
module_platform_driver(vita_i2c_driver);

MODULE_DESCRIPTION("PlayStation Vita I2C controller");
MODULE_LICENSE("GPL");
