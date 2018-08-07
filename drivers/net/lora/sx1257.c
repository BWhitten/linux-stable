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

#define REG_CLK_SELECT		0x10

#define REG_CLK_SELECT_TX_DAC_CLK_SELECT_CLK_IN	BIT(0)
#define REG_CLK_SELECT_CLK_OUT			BIT(1)

enum sx125x_fields {
	F_PA_DRIVER_EN,
	F_TX_EN,
	F_RX_EN,
	F_STDBY_EN,
	F_TX_DAC_GAIN,
	F_TX_MIXER_GAIN,
	F_TX_PLL_BW,
	F_TX_ANA_BW,
	F_TX_DAC_BW,
	F_RX_LNA_GAIN,
	F_RX_BB_GAIN,
	F_RX_LNA_Z_IN,
	F_RX_PLL_BW,
	F_RX_ADC_BW,
	F_RX_ADC_TRIM,
	F_RX_BB_BW,
	F_RX_ADC_TEMP,
	F_DIO0_MAP,
	F_DIO1_MAP,
	F_DIO2_MAP,
	F_DIO3_MAP,
	F_DIG_LOOPBACK,
	F_RF_LOOPBACK,
	F_CLK_OUT,
	F_TX_DAC_CLK_SEL,
	F_LOW_BAT_EN,
	F_PLL_LOCK_RX,
	F_PLL_LOCK_TX,
	F_LOW_BAT_THRES,
	F_SX1257_XOSC_GM_STARTUP,
	F_SX1257_XOSC_DISABLE_REG,
	F_SX1257_XOSC_DISABLE_AMP,
	F_SX1257_XOSC_DISABLE_CORE,
};

static const struct reg_field sx125x_reg_fields[] = {
	/* MODE */
	[F_PA_DRIVER_EN]   = REG_FIELD(SX125X_MODE, 3, 3),
	[F_TX_EN]          = REG_FIELD(SX125X_MODE, 2, 2),
	[F_RX_EN]          = REG_FIELD(SX125X_MODE, 1, 1),
	[F_STDBY_EN]       = REG_FIELD(SX125X_MODE, 0, 0),
	/* TX_GAIN */
	[F_TX_DAC_GAIN]    = REG_FIELD(SX125X_TX_GAIN, 4, 6),
	[F_TX_MIXER_GAIN]  = REG_FIELD(SX125X_TX_GAIN, 0, 3),
	/* TX_BW */
	[F_TX_PLL_BW]      = REG_FIELD(SX125X_TX_BW, 5, 6),
	[F_TX_ANA_BW]      = REG_FIELD(SX125X_TX_BW, 0, 4),
	/* TX_DAC_BW */
	[F_TX_DAC_BW]      = REG_FIELD(SX125X_TX_DAC_BW, 0, 2),
	/* RX_ANA_GAIN */
	[F_RX_LNA_GAIN]    = REG_FIELD(SX125X_RX_ANA_GAIN, 5, 7),
	[F_RX_BB_GAIN]     = REG_FIELD(SX125X_RX_ANA_GAIN, 1, 4),
	[F_RX_LNA_Z_IN]    = REG_FIELD(SX125X_RX_ANA_GAIN, 0, 0),
	/* RX_BW */
	[F_RX_ADC_BW]      = REG_FIELD(SX125X_RX_BW, 5, 7),
	[F_RX_ADC_TRIM]    = REG_FIELD(SX125X_RX_BW, 2, 4),
	[F_RX_BB_BW]       = REG_FIELD(SX125X_RX_BW, 0, 1),
	/* RX_PLL_BW */
	[F_RX_PLL_BW]      = REG_FIELD(SX125X_RX_PLL_BW, 1, 2),
	[F_RX_ADC_TEMP]    = REG_FIELD(SX125X_RX_PLL_BW, 0, 0),
	/* DIO_MAPPING */
	[F_DIO0_MAP]       = REG_FIELD(SX125X_DIO_MAPPING, 6, 7),
	[F_DIO1_MAP]       = REG_FIELD(SX125X_DIO_MAPPING, 4, 5),
	[F_DIO2_MAP]       = REG_FIELD(SX125X_DIO_MAPPING, 2, 3),
	[F_DIO3_MAP]       = REG_FIELD(SX125X_DIO_MAPPING, 0, 1),
	/* CLK_SELECT */
	[F_DIG_LOOPBACK]   = REG_FIELD(SX125X_CLK_SELECT, 3, 3),
	[F_RF_LOOPBACK]    = REG_FIELD(SX125X_CLK_SELECT, 2, 2),
	[F_CLK_OUT]        = REG_FIELD(SX125X_CLK_SELECT, 1, 1),
	[F_TX_DAC_CLK_SEL] = REG_FIELD(SX125X_CLK_SELECT, 0, 0),
	/* MODE_STATUS */
	[F_LOW_BAT_EN]     = REG_FIELD(SX125X_MODE_STATUS, 2, 2),
	[F_PLL_LOCK_RX]    = REG_FIELD(SX125X_MODE_STATUS, 1, 1),
	[F_PLL_LOCK_TX]    = REG_FIELD(SX125X_MODE_STATUS, 0, 0),
	/* LOW_BAT_THRES */
	[F_LOW_BAT_THRES]  = REG_FIELD(SX125X_LOW_BAT_THRES, 0, 2),
	/* XOSC */ /* TODO maybe make this dynamic */
	[F_SX1257_XOSC_GM_STARTUP]  = REG_FIELD(SX1257_XOSC, 0, 3),
	[F_SX1257_XOSC_DISABLE_REG]  = REG_FIELD(SX1257_XOSC, 4, 4),
	[F_SX1257_XOSC_DISABLE_CORE]  = REG_FIELD(SX1257_XOSC, 5, 5),
	[F_SX1257_XOSC_DISABLE_AMP]  = REG_FIELD(SX1257_XOSC, 6, 6),
};

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
	struct regmap_field	*regmap_fields[ARRAY_SIZE(sx125x_reg_fields)];
};

static int sx1257_write(struct spi_device *spi, u8 reg, u8 val)
{
	u8 buf[2];

	buf[0] = reg | BIT(7);
	buf[1] = val;
	return spi_write(spi, buf, 2);
}

static int sx1257_read(struct spi_device *spi, u8 reg, u8 *val)
{
	u8 addr = reg & 0x7f;
	return spi_write_then_read(spi, &addr, 1, val, 1);
}

static int sx1257_probe(struct spi_device *spi)
{
	struct sx125x_priv *priv;
	u8 val;
	int ret;
	int i;
	unsigned int version;

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

	for (i = 0; i < ARRAY_SIZE(sx125x_reg_fields); i++) {
                const struct reg_field *reg_fields = sx125x_reg_fields;

                priv->regmap_fields[i] = devm_regmap_field_alloc(&spi->dev,
                                priv->regmap,
                                reg_fields[i]);
                if (IS_ERR(priv->regmap_fields[i])) {
                        ret = PTR_ERR(priv->regmap_fields[i]);
                        dev_err(&spi->dev, "Cannot allocate regmap field: %d\n", ret);
                        return ret;
                }
        }

	ret = regmap_read(priv->regmap, SX125X_VERSION, &version);
	if (ret)
		return ret;
        dev_info(&spi->dev, "SX125x version: %d\n", version);

	val = REG_CLK_SELECT_TX_DAC_CLK_SELECT_CLK_IN;
	if (strcmp(spi->controller->dev.of_node->name, "radio-b") == 0) { /* HACK */
		val |= REG_CLK_SELECT_CLK_OUT;
		dev_info(&spi->dev, "enabling clock output\n");
	}

	ret = sx1257_write(spi, REG_CLK_SELECT, val);
	if (ret) {
		dev_err(&spi->dev, "clk write failed\n");
		return ret;
	}

	dev_dbg(&spi->dev, "clk written\n");

	if (true) {
		ret = sx1257_write(spi, 0x26, 13 + 2 * 16);
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
