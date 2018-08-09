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

#ifdef CONFIG_LORA_SX125X_SPI
#include <linux/spi/spi.h>
#endif

#include "sx125x.h"

#define REG_CLK_SELECT_TX_DAC_CLK_SELECT_CLK_IN	BIT(0)
#define REG_CLK_SELECT_CLK_OUT			BIT(1)

struct sx125x_priv {
	struct regmap		*regmap;
};

static struct regmap_config __maybe_unused sx125x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.cache_type = REGCACHE_NONE,

	.read_flag_mask = 0,
	.write_flag_mask = BIT(7),

	.max_register = SX125X_MAX_REGISTER,
};

static int __maybe_unused sx125x_regmap_probe(struct device *dev, struct regmap *regmap, unsigned int radio)
{
	struct sx125x_priv *priv;
	unsigned int val;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);
	priv->regmap = regmap;

	if (false) {
		ret = regmap_read(priv->regmap, SX1255_VERSION, &val);
		if (ret) {
			dev_err(dev, "version read failed\n");
			return ret;
		}
		dev_info(dev, "SX125x version: %02x\n", val);
	}

	val = REG_CLK_SELECT_TX_DAC_CLK_SELECT_CLK_IN;
	if (radio == 1) { /* HACK */
		val |= REG_CLK_SELECT_CLK_OUT;
		dev_info(dev, "enabling clock output\n");
	}

	ret = regmap_write(priv->regmap, SX125X_CLK_SELECT, val);
	if (ret) {
		dev_err(dev, "clk write failed\n");
		return ret;
	}

	dev_dbg(dev, "clk written\n");

	if (true) {
		ret = regmap_write(priv->regmap, SX1257_XOSC, 13 + 2 * 16);
		if (ret) {
			dev_err(dev, "xosc write failed\n");
			return ret;
		}
	}

	dev_info(dev, "SX125x module probed\n");

	return 0;
}

static int __maybe_unused sx125x_regmap_remove(struct device *dev)
{
	dev_info(dev, "SX125x module removed\n");

	return 0;
}

#ifdef CONFIG_LORA_SX125X_SPI
static int sx125x_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;
	unsigned int radio;
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

	radio = (strcmp(spi->dev.parent->of_node->name, "radio-b") == 0) ? 1 : 0;
	return sx125x_regmap_probe(&spi->dev, regmap, radio);
}

static int sx125x_spi_remove(struct spi_device *spi)
{
	return sx125x_regmap_remove(&spi->dev);
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
#endif

static int __init sx125x_init(void)
{
	int ret = 0;

#ifdef CONFIG_LORA_SX125X_SPI
	ret = spi_register_driver(&sx125x_spi_driver);
	if (ret < 0) {
		pr_err("failed to init sx125x spi (%d)\n", ret);
		return ret;
	}
#endif

	return ret;
}
module_init(sx125x_init);

static void __exit sx125x_exit(void)
{
#ifdef CONFIG_LORA_SX125X_SPI
	spi_unregister_driver(&sx125x_spi_driver);
#endif
}
module_exit(sx125x_exit);

MODULE_DESCRIPTION("Semtech SX125x LoRa Radio Driver");
MODULE_AUTHOR("Andreas Färber <afaerber@suse.de>");
MODULE_AUTHOR("Ben Whitten <ben.whitten@gmail.com>");
MODULE_LICENSE("GPL");
