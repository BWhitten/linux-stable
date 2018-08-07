// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Semtech SX125x LoRa tranciver backed on an SX1301 concentrator
 *
 * Copyright (c) 2018   Ben Whitten
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#include "sx125x.h"
#include "sx1301.h"

static struct regmap_config sx125x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.cache_type = REGCACHE_NONE,

	.read_flag_mask = 0,
	.write_flag_mask = BIT(7),

	.max_register = SX125X_MAX_REGISTER,
};

static int sx125x_con_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regmap *regmap;
	const struct regmap_bus *regmap_bus;
	int ret;

	dev_info(dev, "Entered probe for con\n");

	regmap_bus = sx1301_concentrator_regmap_bus();
	regmap = devm_regmap_init(dev, regmap_bus, dev,
			&sx125x_regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "Regmap allocation failed: %d\n", ret);
		return ret;
	}

	dev_info(dev, "Got the regmap, about to core probe");

	return sx125x_core_probe(dev, regmap);
}

static int sx125x_con_remove(struct platform_device *pdev)
{
	return sx125x_core_remove(&pdev->dev);
}

#ifdef CONFIG_OF
static const struct of_device_id sx125x_con_of_match[] = {
	{ .compatible = "semtech,sx1255" },
	{ .compatible = "semtech,sx1257" },
	{},
};
MODULE_DEVICE_TABLE(of, sx125x_con_of_match);
#endif

static struct platform_driver sx125x_con_driver = {
	.probe  = sx125x_con_probe,
	.remove = sx125x_con_remove,
	.driver = {
		.name = "sx125x_con",
		.of_match_table = of_match_ptr(sx125x_con_of_match),
	},
};
module_platform_driver(sx125x_con_driver);

MODULE_AUTHOR("Ben Whitten <ben.whitten@gmail.com>");
MODULE_DESCRIPTION("Semtech SX125x LoRa Radio Driver");
MODULE_LICENSE("GPL");

