// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Semtech SX1255/SX1257 LoRa tranciver
 *
 * Copyright (c) 2018   Ben Whitten
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>

#include "sx125x.h"

static struct regmap_config sx125x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.cache_type = REGCACHE_NONE,

	.read_flag_mask = 0,
	.write_flag_mask = BIT(7),

	.max_register = SX125X_MAX_REGISTER,
};

static int sx125x_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;
	int ret;

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi->max_speed_hz = 10000000;
	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "SPI setup failed.\n");
		return ret;
	}

	regmap = devm_regmap_init_spi(spi, &sx125x_regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(&spi->dev, "Regmap allocation failed: %d\n", ret);
		return ret;
	}

	return sx125x_core_probe(&spi->dev, regmap);
}

static int sx125x_spi_remove(struct spi_device *spi)
{
	return sx125x_core_remove(&spi->dev);
}

#ifdef CONFIG_OF
static const struct of_device_id sx125x_spi_of_match[] = {
	{ .compatible = "semtech,sx1255" },
	{ .compatible = "semtech,sx1257" },
	{},
};
MODULE_DEVICE_TABLE(of, sx125x_spi_of_match);
#endif

static struct spi_driver sx125x_spi_driver = {
	.probe  = sx125x_spi_probe,
	.remove = sx125x_spi_remove,
	.driver = {
		.name = "sx125x_spi",
		.of_match_table = of_match_ptr(sx125x_spi_of_match),
	},
};
module_spi_driver(sx125x_spi_driver);

MODULE_AUTHOR("Ben Whitten <ben.whitten@gmail.com>");
MODULE_DESCRIPTION("Semtech SX125x LoRa Radio Driver");
MODULE_LICENSE("GPL");

