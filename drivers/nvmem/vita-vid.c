// SPDX-License-Identifier: GPL-2.0
/*
 * PlayStation Vita ScePervasiveVid (0xE3104000): the chip's own voltage IDs,
 * one 32-bit word per rail and clock level, read-only and different from
 * chip to chip [WIKI Pervasive, ScePervasiveVid]. ScePower reads them once
 * and asks Syscon for 0x8A - vid, in 10 mV units, for each level it sets.
 * The device tree names the words as nvmem cells.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/platform_device.h>

static int vita_vid_read(void *context, unsigned int offset, void *val,
			 size_t bytes)
{
	void __iomem *base = context;
	u32 *buf = val;

	for (; bytes >= sizeof(u32); bytes -= sizeof(u32), offset += sizeof(u32))
		*buf++ = readl(base + offset);

	return 0;
}

static int vita_vid_probe(struct platform_device *pdev)
{
	struct nvmem_config config = {
		.dev = &pdev->dev,
		.name = "vita-vid",
		.id = NVMEM_DEVID_NONE,
		.owner = THIS_MODULE,
		.read_only = true,
		.word_size = sizeof(u32),
		.stride = sizeof(u32),
		.reg_read = vita_vid_read,
	};
	struct resource *res;
	void __iomem *base;

	base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	config.size = resource_size(res);
	config.priv = base;

	return PTR_ERR_OR_ZERO(devm_nvmem_register(&pdev->dev, &config));
}

static const struct of_device_id vita_vid_of_match[] = {
	{ .compatible = "sony,vita-vid" },
	{ }
};
MODULE_DEVICE_TABLE(of, vita_vid_of_match);

static struct platform_driver vita_vid_driver = {
	.probe = vita_vid_probe,
	.driver = {
		.name = "vita-vid",
		.of_match_table = vita_vid_of_match,
	},
};
module_platform_driver(vita_vid_driver);

MODULE_DESCRIPTION("PlayStation Vita voltage ID (ScePervasiveVid) driver");
MODULE_LICENSE("GPL");
