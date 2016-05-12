/*
 * Driver for Broadcom BCM2835 soc sdram controller
 *
 * Copyright (C) 2016 Martin Sperl
 *
 * inspired by: atmel-sdramc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

struct bcm2835_sdram_data {
	void __iomem *regs;
	struct clk *clk;
};

static int bcm2835_sdram_probe(struct platform_device *pdev)
{
	struct bcm2835_sdram_data *data;
	struct resource *res;
	int err;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	platform_set_drvdata(pdev, data);

	/* get registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->regs)) {
		err = PTR_ERR(data->regs);
		dev_err(&pdev->dev,
			"Could not get register range: %d\n", err);
			return err;
	}
	/* get clock */
	data->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(data->clk))
		return PTR_ERR(data->clk);
	clk_prepare_enable(data->clk);

	return 0;
}

static const struct of_device_id bcm2835_sdram_of_match_table[] = {
	{ .compatible = "brcm,bcm2835-sdram", },
	{},
};
MODULE_DEVICE_TABLE(of, bcm2835_sdram_of_match_table);

static struct platform_driver bcm2835_sdram_driver = {
	.probe = bcm2835_sdram_probe,
	.driver = {
		.name = "bcm2835_sdram",
		.of_match_table = bcm2835_sdram_of_match_table,
	},
};
module_platform_driver(bcm2835_sdram_driver);

MODULE_AUTHOR("Martin Sperl");
MODULE_DESCRIPTION("sdram driver for bcm2835 chip");
MODULE_LICENSE("GPL");
