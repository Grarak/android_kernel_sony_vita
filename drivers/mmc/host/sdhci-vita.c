// SPDX-License-Identifier: GPL-2.0
/*
 * SDHCI glue for the PlayStation Vita's SceSdif hosts
 *
 * SceSdif0-3 are SD Host Controller 2.0 blocks (version register 0x8901,
 * vendor 0x89), mirrored every 0x100 bytes (wiki.henkaku.xyz SDIF_Registers).
 * Around the standard register file the SoC adds:
 *  - a reset bit per host in ScePervasiveReset (+0xA0 + 4 * n), handled here
 *    through the reset controller;
 *  - a clock gate per host in ScePervasiveGate (same offsets), which the
 *    boot loader leaves enabled;
 *  - the 1.8 V signalling select in ScePervasiveMisc +0x124 (TODO).
 *
 * The capabilities register's base clock field is not known to be valid,
 * so "clock-frequency" from the DT is used when it reads zero.
 */

#include <linux/io.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include "sdhci-pltfm.h"

struct sdhci_vita {
	struct reset_control *rst;
	u32 base_clock;
};

static unsigned int sdhci_vita_get_max_clock(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_vita *vita = sdhci_pltfm_priv(pltfm_host);

	return vita->base_clock;
}

static const struct sdhci_ops sdhci_vita_ops = {
	.set_clock		= sdhci_set_clock,
	.get_max_clock		= sdhci_vita_get_max_clock,
	.set_bus_width		= sdhci_set_bus_width,
	.reset			= sdhci_reset,
	.set_uhs_signaling	= sdhci_set_uhs_signaling,
};

static const struct sdhci_pltfm_data sdhci_vita_pdata = {
	.ops	= &sdhci_vita_ops,
	/*
	 * Nothing is known about the DMA engines yet: start in PIO.
	 * TODO: try SDMA/ADMA2 once PIO works on hardware.
	 */
	.quirks	= SDHCI_QUIRK_BROKEN_DMA | SDHCI_QUIRK_BROKEN_ADMA |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.quirks2 = SDHCI_QUIRK2_NO_1_8_V,
};

static int sdhci_vita_probe(struct platform_device *pdev)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_host *host;
	struct sdhci_vita *vita;
	int ret;

	host = sdhci_pltfm_init(pdev, &sdhci_vita_pdata, sizeof(*vita));
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);
	vita = sdhci_pltfm_priv(pltfm_host);

	if (of_property_read_u32(pdev->dev.of_node, "clock-frequency",
				 &vita->base_clock) || !vita->base_clock) {
		dev_err(&pdev->dev, "no clock-frequency for the SD base clock\n");
		return -EINVAL;
	}

	vita->rst = devm_reset_control_get_optional_exclusive(&pdev->dev, NULL);
	if (IS_ERR(vita->rst))
		return dev_err_probe(&pdev->dev, PTR_ERR(vita->rst),
				     "failed to get reset\n");

	ret = reset_control_deassert(vita->rst);
	if (ret)
		return ret;

	ret = mmc_of_parse(host->mmc);
	if (ret)
		goto err_reset;

	sdhci_get_of_property(pdev);

	ret = sdhci_add_host(host);
	if (ret)
		goto err_reset;

	return 0;

err_reset:
	reset_control_assert(vita->rst);
	return ret;
}

static void sdhci_vita_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_vita *vita = sdhci_pltfm_priv(pltfm_host);

	sdhci_pltfm_remove(pdev);
	reset_control_assert(vita->rst);
}

static const struct of_device_id sdhci_vita_of_match[] = {
	{ .compatible = "vita,sdhci" },
	{ }
};
MODULE_DEVICE_TABLE(of, sdhci_vita_of_match);

static struct platform_driver sdhci_vita_driver = {
	.driver = {
		.name = "sdhci-vita",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = sdhci_vita_of_match,
	},
	.probe = sdhci_vita_probe,
	.remove = sdhci_vita_remove,
};
module_platform_driver(sdhci_vita_driver);

MODULE_DESCRIPTION("SDHCI glue for the PlayStation Vita SceSdif hosts");
MODULE_LICENSE("GPL");
