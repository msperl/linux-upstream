/*
 * Copyright (C) 2015 Broadcom
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

#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/clk/bcm2835.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <dt-bindings/clock/bcm2835-aux.h>

static int bcm2835_aux_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_onecell_data *onecell;
	const char *parent;
	void __iomem *reg;

	parent = of_clk_get_parent_name(dev->of_node, 0);
	if (!parent) {
		dev_err(dev, "Couldn't find parent clock\n");
		return -ENODEV;
	}

	reg = of_iomap(dev->of_node, 0);
	if (!reg)
		return -ENODEV;

	onecell = kmalloc(sizeof(*onecell), GFP_KERNEL);
	if (!onecell)
		return -ENOMEM;
	onecell->clk_num = BCM2835_AUX_CLOCK_COUNT;
	onecell->clks = kzalloc(sizeof(*onecell->clks), GFP_KERNEL);
	if (!onecell->clks)
		return -ENOMEM;

	onecell->clks[BCM2835_AUX_CLOCK_UART] =
		clk_register_gate(dev, "aux_uart", parent,
				  CLK_IGNORE_UNUSED | CLK_SET_RATE_GATE,
				  reg, BIT(0),
				  0, NULL);

	onecell->clks[BCM2835_AUX_CLOCK_SPI1] =
		clk_register_gate(dev, "aux_spi1", parent,
				  CLK_IGNORE_UNUSED | CLK_SET_RATE_GATE,
				  reg, BIT(1),
				  0, NULL);

	onecell->clks[BCM2835_AUX_CLOCK_SPI2] =
		clk_register_gate(dev, "aux_spi2", parent,
				  CLK_IGNORE_UNUSED | CLK_SET_RATE_GATE,
				  reg, BIT(2),
				  0, NULL);

	return of_clk_add_provider(pdev->dev.of_node, of_clk_src_onecell_get,
				   onecell);
}

static const struct of_device_id bcm2835_aux_clk_of_match[] = {
	{ .compatible = "brcm,bcm2835-aux-clock", },
	{},
};
MODULE_DEVICE_TABLE(of, bcm2835_aux_clk_of_match);

static struct platform_driver bcm2835_aux_clk_driver = {
	.driver = {
		.name = "bcm2835-aux-clk",
		.of_match_table = bcm2835_aux_clk_of_match,
	},
	.probe          = bcm2835_aux_clk_probe,
};
builtin_platform_driver(bcm2835_aux_clk_driver);

MODULE_AUTHOR("Eric Anholt <eric@anholt.net>");
MODULE_DESCRIPTION("BCM2835 auxiliary peripheral clock driver");
MODULE_LICENSE("GPL v2");
