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

#include "sx125x.h"

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

struct sx125x_priv {
	struct device		*dev;
	struct regmap		*regmap;
	struct regmap_field	*regmap_fields[ARRAY_SIZE(sx125x_reg_fields)];
};

static int sx125x_field_read(struct sx125x_priv *priv,
		enum sx125x_fields field_id)
{
	int ret;
	int val;

	ret = regmap_field_read(priv->regmap_fields[field_id], &val);
	if (ret)
		return ret;

	return val;
}

static int sx125x_field_write(struct sx125x_priv *priv,
		enum sx125x_fields field_id, u8 val)
{
	return regmap_field_write(priv->regmap_fields[field_id], val);
}

int sx125x_core_probe(struct device *dev, struct regmap *regmap)
{
	struct sx125x_priv *priv;
	int ret;
	int i;
	unsigned int version;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);
	priv->regmap = regmap;
	priv->dev = dev;

	for (i = 0; i < ARRAY_SIZE(sx125x_reg_fields); i++) {
		const struct reg_field *reg_fields = sx125x_reg_fields;

		priv->regmap_fields[i] = devm_regmap_field_alloc(dev,
				priv->regmap,
				reg_fields[i]);
		if (IS_ERR(priv->regmap_fields[i])) {
			ret = PTR_ERR(priv->regmap_fields[i]);
			dev_err(dev, "Cannot allocate regmap field: %d\n", ret);
			return ret;
		}
	}

	ret = regmap_read(priv->regmap, SX125X_VERSION, &version);
	if (ret)
		return ret;
	dev_info(dev, "SX125x version: %d\n", version);


	/* TODO HACK to get clocks on */
	ret = sx125x_field_write(priv, F_TX_DAC_CLK_SEL, 1);
	if (ret)
		return ret;
	/* TODO HACK If we are a clock provider in DT, just turn it on */
	if (of_find_property(dev->of_node, "#clock-cells", NULL)) {
		ret = sx125x_field_write(priv, F_CLK_OUT, 1);
		dev_info(dev, "enabling clock output\n");
		if (ret)
			return ret;
	}
	dev_dbg(dev, "CLKs enabled\n");

	/* TODO HACK checkout my radio type = sx1257 */
	/* Disable core XOSC */
	ret = sx125x_field_write(priv, F_SX1257_XOSC_DISABLE_CORE, 1);
	if (ret)
		return ret;
	ret = sx125x_field_write(priv, F_SX1257_XOSC_GM_STARTUP, 13);
	if (ret)
		return ret;

	dev_info(dev, "SX125x module probed\n");

	return 0;
}
EXPORT_SYMBOL_GPL(sx125x_core_probe);

int sx125x_core_remove(struct device *dev)
{
	dev_info(dev, "SX125x module removed\n");

	return 0;
}
EXPORT_SYMBOL_GPL(sx125x_core_remove);

MODULE_DESCRIPTION("SX125x core driver");
MODULE_AUTHOR("Andreas Färber <afaerber@suse.de>");
MODULE_AUTHOR("Ben Whitten <ben.whitten@gmail.com>");
MODULE_LICENSE("GPL");
