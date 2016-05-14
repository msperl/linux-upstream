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

/* the set of registers */
#define BCM2835_SD_CS		0x000
#define BCM2835_SD_SA		0x004
#define BCM2835_SD_SB		0x008
#define BCM2835_SD_SC		0x00c
#define BCM2835_SD_PT2		0x010
#define BCM2835_SD_PT1		0x014
#define BCM2835_SD_SECSRT0	0x03c
#define BCM2835_SD_SECEND0	0x040
#define BCM2835_SD_SECSRT1	0x044
#define BCM2835_SD_SECEND1	0x048
#define BCM2835_SD_SECSRT2	0x04c
#define BCM2835_SD_SECEND2	0x050
#define BCM2835_SD_SECSRT3	0x054
#define BCM2835_SD_SECEND3	0x058
#define BCM2835_SD_PHYC		0x060
#define BCM2835_SD_MRT		0x064
#define BCM2835_SD_TMC		0x07c
#define BCM2835_SD_RWC		0x080
#define BCM2835_SD_VIN		0x088
#define BCM2835_SD_MR		0x090
#define BCM2835_SD_SD		0x094
#define BCM2835_SD_SE		0x098
#define BCM2835_SD_STALL	0x0a0
#define BCM2835_SD_SF		0x0b4
#define BCM2835_SD_CARCRC	0x100
#define BCM2835_SD_DMRCRC0	0x104
#define BCM2835_SD_DMRCRC1	0x108
#define BCM2835_SD_DQRCRC0	0x10c
#define BCM2835_SD_DQRCRC1	0x110
#define BCM2835_SD_DQRCRC2	0x114
#define BCM2835_SD_DQRCRC3	0x118
#define BCM2835_SD_DQRCRC4	0x11c
#define BCM2835_SD_DQRCRC5	0x120
#define BCM2835_SD_DQRCRC6	0x124
#define BCM2835_SD_DQRCRC7	0x128
#define BCM2835_SD_DQRCRC8	0x12c
#define BCM2835_SD_DQRCRC9	0x130
#define BCM2835_SD_DQRCRC10	0x134
#define BCM2835_SD_DQRCRC11	0x138
#define BCM2835_SD_DQRCRC12	0x13c
#define BCM2835_SD_DQRCRC13	0x140
#define BCM2835_SD_DQRCRC14	0x144
#define BCM2835_SD_DQRCRC15	0x148
#define BCM2835_SD_DQLCRC0	0x14c
#define BCM2835_SD_DQLCRC1	0x150
#define BCM2835_SD_DQLCRC2	0x154
#define BCM2835_SD_DQLCRC3	0x158
#define BCM2835_SD_DQLCRC4	0x15c
#define BCM2835_SD_DQLCRC5	0x160
#define BCM2835_SD_DQLCRC6	0x164
#define BCM2835_SD_DQLCRC7	0x168
#define BCM2835_SD_DQLCRC8	0x16c
#define BCM2835_SD_DQLCRC9	0x170
#define BCM2835_SD_DQLCRC10	0x174
#define BCM2835_SD_DQLCRC11	0x178
#define BCM2835_SD_DQLCRC12	0x17c
#define BCM2835_SD_DQLCRC13	0x180
#define BCM2835_SD_DQLCRC14	0x184
#define BCM2835_SD_DQLCRC15	0x188

struct bcm2835_sdram_data {
	void __iomem *regs;
	struct dentry *debugfsdir;
	struct clk *clk;
};

#define R(n, o) { .name = n, .offset = o }
static const struct debugfs_reg32 bcm2835_sdram_regs[] = {
	R("cs",		BCM2835_SD_CS),
	R("sa",		BCM2835_SD_SA),
	R("sb",		BCM2835_SD_SB),
	R("sc",		BCM2835_SD_SC),
	R("pt2",	BCM2835_SD_PT2),
	R("pt1",	BCM2835_SD_PT1),
	R("secsrt0",	BCM2835_SD_SECSRT0),
	R("secend0",	BCM2835_SD_SECEND0),
	R("secsrt1",	BCM2835_SD_SECSRT1),
	R("secend1",	BCM2835_SD_SECEND1),
	R("secsrt2",	BCM2835_SD_SECSRT2),
	R("secend2",	BCM2835_SD_SECEND2),
	R("secsrt3",	BCM2835_SD_SECSRT3),
	R("secend3",	BCM2835_SD_SECEND3),
	R("phyc",	BCM2835_SD_PHYC),
	R("mrt",	BCM2835_SD_MRT),
	R("tmc",	BCM2835_SD_TMC),
	R("rwc",	BCM2835_SD_RWC),
	R("vin",	BCM2835_SD_VIN),
	R("mr",		BCM2835_SD_MR),
	R("sd",		BCM2835_SD_SD),
	R("se",		BCM2835_SD_SE),
	R("stall",	BCM2835_SD_STALL),
	R("sf",		BCM2835_SD_SF),
	R("carcrc",	BCM2835_SD_CARCRC),
	R("dmrcrc0",	BCM2835_SD_DMRCRC0),
	R("dmrcrc1",	BCM2835_SD_DMRCRC1),
	R("dqrcrc0",	BCM2835_SD_DQRCRC0),
	R("dqrcrc1",	BCM2835_SD_DQRCRC1),
	R("dqrcrc2",	BCM2835_SD_DQRCRC2),
	R("dqrcrc3",	BCM2835_SD_DQRCRC3),
	R("dqrcrc4",	BCM2835_SD_DQRCRC4),
	R("dqrcrc5",	BCM2835_SD_DQRCRC5),
	R("dqrcrc6",	BCM2835_SD_DQRCRC6),
	R("dqrcrc7",	BCM2835_SD_DQRCRC7),
	R("dqrcrc8",	BCM2835_SD_DQRCRC8),
	R("dqrcrc9",	BCM2835_SD_DQRCRC9),
	R("dqrcrc10",	BCM2835_SD_DQRCRC10),
	R("dqrcrc11",	BCM2835_SD_DQRCRC11),
	R("dqrcrc12",	BCM2835_SD_DQRCRC12),
	R("dqrcrc13",	BCM2835_SD_DQRCRC13),
	R("dqrcrc14",	BCM2835_SD_DQRCRC14),
	R("dqrcrc15",	BCM2835_SD_DQRCRC15),
	R("dqlcrc0",	BCM2835_SD_DQLCRC0),
	R("dqlcrc1",	BCM2835_SD_DQLCRC1),
	R("dqlcrc2",	BCM2835_SD_DQLCRC2),
	R("dqlcrc3",	BCM2835_SD_DQLCRC3),
	R("dqlcrc4",	BCM2835_SD_DQLCRC4),
	R("dqlcrc5",	BCM2835_SD_DQLCRC5),
	R("dqlcrc6",	BCM2835_SD_DQLCRC6),
	R("dqlcrc7",	BCM2835_SD_DQLCRC7),
	R("dqlcrc8",	BCM2835_SD_DQLCRC8),
	R("dqlcrc9",	BCM2835_SD_DQLCRC9),
	R("dqlcrc10",	BCM2835_SD_DQLCRC10),
	R("dqlcrc11",	BCM2835_SD_DQLCRC11),
	R("dqlcrc12",	BCM2835_SD_DQLCRC12),
	R("dqlcrc13",	BCM2835_SD_DQLCRC13),
	R("dqlcrc14",	BCM2835_SD_DQLCRC14),
	R("dqlcrc15",	BCM2835_SD_DQLCRC15)
};

static void bcm2835_sdram_debugfs(struct platform_device *pdev)
{
	struct bcm2835_sdram_data *data = platform_get_drvdata(pdev);
	struct debugfs_regset32 *regset;

	data->debugfsdir = debugfs_create_dir("bcm2835_sdram", NULL);
	if (!data->debugfsdir)
		return;

	regset = devm_kzalloc(&pdev->dev, sizeof(*regset),
			      GFP_KERNEL);
	if (!regset)
		return;

	regset->regs = bcm2835_sdram_regs;
	regset->nregs = ARRAY_SIZE(bcm2835_sdram_regs);
	regset->base = data->regs;

	debugfs_create_regset32("regset", S_IRUGO,
				data->debugfsdir, regset);
}

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

	/* setup debugfs */
	bcm2835_sdram_debugfs(pdev);

	return 0;
}

static int bcm2835_sdram_remove(struct platform_device *pdev)
{
	struct bcm2835_sdram_data *data = platform_get_drvdata(pdev);

	debugfs_remove_recursive(data->debugfsdir);

	return 0;
}

static const struct of_device_id bcm2835_sdram_of_match_table[] = {
	{ .compatible = "brcm,bcm2835-sdram", },
	{},
};
MODULE_DEVICE_TABLE(of, bcm2835_sdram_of_match_table);

static struct platform_driver bcm2835_sdram_driver = {
	.probe = bcm2835_sdram_probe,
	.remove = bcm2835_sdram_remove,
	.driver = {
		.name = "bcm2835_sdram",
		.of_match_table = bcm2835_sdram_of_match_table,
	},
};
module_platform_driver(bcm2835_sdram_driver);

MODULE_AUTHOR("Martin Sperl");
MODULE_DESCRIPTION("sdram driver for bcm2835 chip");
MODULE_LICENSE("GPL");
