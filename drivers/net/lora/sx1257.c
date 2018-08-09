// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Semtech SX1255/SX1257 LoRa transceiver
 *
 * Copyright (c) 2018 Andreas Färber
 * Copyright (c) 2018 Ben Whitten
 *
 * Based on SX1301 HAL code:
 * Copyright (c) 2013 Semtech-Cycleo
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#include "sx125x.h"

#define REG_CLK_SELECT_TX_DAC_CLK_SELECT_CLK_IN	BIT(0)
#define REG_CLK_SELECT_CLK_OUT			BIT(1)

static struct regmap_config sx125x_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.cache_type = REGCACHE_NONE,

	.read_flag_mask = 0,
	.write_flag_mask = BIT(7),

	.max_register = SX125X_MAX_REGISTER,
};

struct sx125x_priv {
	struct regmap		*regmap;
};

static int sx1257_probe(struct spi_device *spi)
{
	struct sx125x_priv *priv;
	unsigned int val;
	int ret;

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi->max_speed_hz = 10000000;
	ret = spi_setup(spi);
	if (ret) {
		dev_err(&spi->dev, "SPI setup failed.\n");
		return ret;
	}

	priv = devm_kzalloc(&spi->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(&spi->dev, priv);

	priv->regmap = devm_regmap_init_spi(spi, &sx125x_spi_regmap_config);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err(&spi->dev, "Regmap allocation failed: %d\n", ret);
		return ret;
	}

	if (false) {
		ret = regmap_read(priv->regmap, SX1255_VERSION, &val);
		if (ret) {
			dev_err(&spi->dev, "version read failed\n");
			return ret;
		}
		dev_info(&spi->dev, "SX125x version: %02x\n", val);
	}

	val = REG_CLK_SELECT_TX_DAC_CLK_SELECT_CLK_IN;
	if (strcmp(spi->controller->dev.of_node->name, "radio-b") == 0) { /* HACK */
		val |= REG_CLK_SELECT_CLK_OUT;
		dev_info(&spi->dev, "enabling clock output\n");
	}

	ret = regmap_write(priv->regmap, SX125X_CLK_SELECT, val);
	if (ret) {
		dev_err(&spi->dev, "clk write failed\n");
		return ret;
	}

	dev_dbg(&spi->dev, "clk written\n");

	if (true) {
		ret = regmap_write(priv->regmap, SX1257_XOSC, 13 + 2 * 16);
		if (ret) {
			dev_err(&spi->dev, "xosc write failed\n");
			return ret;
		}
	}

	dev_info(&spi->dev, "SX1257 module probed\n");

	return 0;
}

static int sx1257_remove(struct spi_device *spi)
{
	dev_info(&spi->dev, "SX1257 module removed\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sx1257_dt_ids[] = {
	{ .compatible = "semtech,sx1255" },
	{ .compatible = "semtech,sx1257" },
	{}
};
MODULE_DEVICE_TABLE(of, sx1257_dt_ids);
#endif

static struct spi_driver sx1257_spi_driver = {
	.driver = {
		.name = "sx1257",
		.of_match_table = of_match_ptr(sx1257_dt_ids),
	},
	.probe = sx1257_probe,
	.remove = sx1257_remove,
};

module_spi_driver(sx1257_spi_driver);

MODULE_DESCRIPTION("SX1257 SPI driver");
MODULE_AUTHOR("Andreas Färber <afaerber@suse.de>");
MODULE_AUTHOR("Ben Whitten <ben.whitten@gmail.com>");
MODULE_LICENSE("GPL");
