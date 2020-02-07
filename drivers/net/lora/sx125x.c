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

#ifdef CONFIG_LORA_SX125X_CON
#include <linux/lora/sx130x.h>
#endif

#include "sx125x.h"

enum sx125x_fields {
	F_PA_DRIVER_EN,
	F_TX_EN,
	F_RX_EN,
	F_STDBY_EN,
	F_CLK_OUT,
	F_TX_DAC_CLK_SEL,
	F_SX1257_XOSC_GM_STARTUP,
	F_SX1257_XOSC_DISABLE_CORE,
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
	F_LOW_BAT_EN,
	F_PLL_LOCK_RX,
	F_PLL_LOCK_TX,
};

static const struct reg_field sx125x_regmap_fields[] = {
	/* MODE */
	[F_PA_DRIVER_EN]   = REG_FIELD(SX125X_MODE, 3, 3),
	[F_TX_EN]          = REG_FIELD(SX125X_MODE, 2, 2),
	[F_RX_EN]          = REG_FIELD(SX125X_MODE, 1, 1),
	[F_STDBY_EN]       = REG_FIELD(SX125X_MODE, 0, 0),
	/* CLK_SELECT */
	[F_CLK_OUT]        = REG_FIELD(SX125X_CLK_SELECT, 1, 1),
	[F_TX_DAC_CLK_SEL] = REG_FIELD(SX125X_CLK_SELECT, 0, 0),
	/* XOSC */ /* TODO maybe make this dynamic */
	[F_SX1257_XOSC_GM_STARTUP]  = REG_FIELD(SX1257_XOSC, 0, 3),
	[F_SX1257_XOSC_DISABLE_CORE]  = REG_FIELD(SX1257_XOSC, 5, 5),
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
	/* MODE_STATUS */
	[F_LOW_BAT_EN]     = REG_FIELD(SX125X_MODE_STATUS, 2, 2),
	[F_PLL_LOCK_RX]    = REG_FIELD(SX125X_MODE_STATUS, 1, 1),
	[F_PLL_LOCK_TX]    = REG_FIELD(SX125X_MODE_STATUS, 0, 0),
};

struct sx125x_priv {
	struct regmap		*regmap;
	struct regmap_field     *regmap_fields[ARRAY_SIZE(sx125x_regmap_fields)];
};

static struct regmap_config __maybe_unused sx125x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.cache_type = REGCACHE_NONE,

	.read_flag_mask = 0,
	.write_flag_mask = BIT(7),

	.max_register = SX125X_MAX_REGISTER,
};

static int sx125x_field_write(struct sx125x_priv *priv,
		enum sx125x_fields field_id, u8 val)
{
	return regmap_field_write(priv->regmap_fields[field_id], val);
}

static inline int sx125x_field_force_write(struct sx125x_priv *priv,
                enum sx125x_fields field_id, u8 val)
{
        return regmap_field_force_write(priv->regmap_fields[field_id], val);
}

static int __maybe_unused sx125x_regmap_probe(struct device *dev, struct regmap *regmap)
{
	struct sx125x_priv *priv;
	unsigned int val;
	const char *rname;
	int ret;
	int i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);
	priv->regmap = regmap;
	for (i = 0; i < ARRAY_SIZE(sx125x_regmap_fields); i++) {
		const struct reg_field *reg_fields = sx125x_regmap_fields;

		priv->regmap_fields[i] = devm_regmap_field_alloc(dev,
				priv->regmap,
				reg_fields[i]);
		if (IS_ERR(priv->regmap_fields[i])) {
			ret = PTR_ERR(priv->regmap_fields[i]);
			dev_err(dev, "Cannot allocate regmap field: %d\n", ret);
			return ret;
		}
	}

	if (true) {
		ret = regmap_read(priv->regmap, SX1255_VERSION, &val);
		if (ret) {
			dev_err(dev, "version read failed (%d)\n", ret);
			return ret;
		}
		if (val != 0x21) {
			dev_err(dev, "unexpected version: %u\n", val);
			return -EINVAL;
		}
		dev_info(dev, "SX125x version: %02x\n", val);
	}

	rname = dev_name(dev);
	if (rname[strlen(rname) - 1] == 'b') {
		ret = sx125x_field_write(priv, F_CLK_OUT, 1);
		if (ret) {
			dev_err(dev, "enabling clock output failed\n");
			return ret;
		}

		dev_info(dev, "enabling clock output\n");
	}

	/* TODO Only needs setting on radio on the TX path */
	ret = sx125x_field_write(priv, F_TX_DAC_CLK_SEL, 1);
	if (ret) {
		dev_err(dev, "clock select failed\n");
		return ret;
	}

	dev_dbg(dev, "clk written\n");

	if (true) {
		ret = sx125x_field_write(priv, F_SX1257_XOSC_DISABLE_CORE, 1);
		if (ret) {
			dev_err(dev, "xosc disable failed\n");
			return ret;
		}

		ret = sx125x_field_write(priv, F_SX1257_XOSC_GM_STARTUP, 13);
		if (ret) {
			dev_err(dev, "xosc startup adjust failed\n");
			return ret;
		}
	}

	if (true) {
		/* Tx gain and trim */
		ret = sx125x_field_write(priv, F_TX_MIXER_GAIN, 14);
		if (ret) {
			dev_err(dev, "setting TX mixer gain failed\n");
			return ret;
		}

		ret = sx125x_field_write(priv, F_TX_DAC_GAIN, 2);
		if (ret) {
			dev_err(dev, "setting TX dac gain failed\n");
			return ret;
		}

		ret = sx125x_field_write(priv, F_TX_ANA_BW, 0);
		if (ret) {
			dev_err(dev, "setting TX ana bw failed\n");
			return ret;
		}

		ret = sx125x_field_write(priv, F_TX_PLL_BW, 1);
		if (ret) {
			dev_err(dev, "setting TX pll bw failed\n");
			return ret;
		}

		ret = sx125x_field_write(priv, F_TX_DAC_BW, 5);
		if (ret) {
			dev_err(dev, "setting TX ana bw failed\n");
			return ret;
		}

		/* Rx gain and trim */
		ret = sx125x_field_write(priv, F_RX_LNA_Z_IN, 1);
		if (ret) {
			dev_err(dev, "setting RC lna z in failed\n");
			return ret;
		}

		ret = sx125x_field_write(priv, F_RX_BB_GAIN, 12);
		if (ret) {
			dev_err(dev, "setting RX bb gain failed\n");
			return ret;
		}

		ret = sx125x_field_write(priv, F_RX_LNA_GAIN, 1);
		if (ret) {
			dev_err(dev, "setting RX lna z in failed\n");
			return ret;
		}

		ret = sx125x_field_write(priv, F_RX_BB_BW, 12);
		if (ret) {
			dev_err(dev, "setting RX bb bw failed\n");
			return ret;
		}

		ret = sx125x_field_write(priv, F_RX_ADC_TRIM, 6);
		if (ret) {
			dev_err(dev, "setting RX adc trim failed\n");
			return ret;
		}

		ret = sx125x_field_write(priv, F_RX_ADC_BW, 7);
		if (ret) {
			dev_err(dev, "setting RX adc bw failed\n");
			return ret;
		}

		ret = sx125x_field_write(priv, F_RX_ADC_TEMP, 0);
		if (ret) {
			dev_err(dev, "setting RX adc temp failed\n");
			return ret;
		}

		ret = sx125x_field_write(priv, F_RX_PLL_BW, 0);
		if (ret) {
			dev_err(dev, "setting RX pll bw failed\n");
			return ret;
		}
	}

	if (true) {
		u32 part_int, part_frac;

		part_int = 868500000 / (SX125X_32MHz_FRAC << 8);
		part_frac = ((868500000 %
			(SX125X_32MHz_FRAC << 8)) << 8) / SX125X_32MHz_FRAC;

		ret = regmap_write(priv->regmap, SX125X_FRF_RX_MSB,
				   0xFF & part_int);
		if (ret) {
			dev_err(dev, "RX MSB write failed\n");
			return ret;
		}

		ret = regmap_write(priv->regmap, SX125X_FRF_RX_MID,
				   0xFF & (part_frac >> 8));
		if (ret) {
			dev_err(dev, "RX MID write failed\n");
			return ret;
		}

		ret = regmap_write(priv->regmap, SX125X_FRF_RX_LSB,
				   0xFF & part_frac);
		if (ret) {
			dev_err(dev, "RX LSB write failed\n");
			return ret;
		}
	}

	/* TODO start pll*/
	ret = sx125x_field_write(priv, F_STDBY_EN, 1);
	if (ret) {
		dev_err(dev, "setting standby enable failed\n");
		return ret;
	}

	i = 5;
	do {
		if (i == 0) {
			dev_err(dev, "RX PLL lock failed\n");
			return -ETIMEDOUT;
		}

		ret = sx125x_field_force_write(priv, F_RX_EN, 0);
		if (ret) {
			dev_err(dev, "setting RX enable failed\n");
			return ret;
		}

		ret = sx125x_field_force_write(priv, F_RX_EN, 1);
		if (ret) {
			dev_err(dev, "setting RX enable failed\n");
			return ret;
		}

		i--;
		usleep_range(1000, 1200);
		ret = regmap_field_read(priv->regmap_fields[F_PLL_LOCK_RX],
					&val);
		if (ret) {
			dev_err(dev, "RX pll lock read failed (%d)\n", ret);
			return ret;
		}
	} while (!val);
/*
	if (regmap_field_read_poll_timeout(priv->regmap_fields[F_PLL_LOCK_RX],
					   val, val, 1000, 5000)) {
		dev_err(dev, "RX PLL lock failed\n");
		return -ETIMEDOUT;
	}
*/
	dev_info(dev, "SX125x module probed\n");

	return 0;
}

static int __maybe_unused sx125x_regmap_remove(struct device *dev)
{
	dev_info(dev, "SX125x module removed\n");

	return 0;
}

#ifdef CONFIG_LORA_SX125X_CON
static int sx125x_con_probe(struct sx130x_radio_device *rdev)
{
	struct device *dev = &rdev->dev;
	int ret;

	rdev->regmap = devm_regmap_init(dev, rdev->regmap_bus, rdev,
			&sx125x_regmap_config);
	if (IS_ERR(rdev->regmap)) {
		ret = PTR_ERR(rdev->regmap);
		dev_err(dev, "Regmap allocation failed: %d\n", ret);
		return ret;
	}

	return sx125x_regmap_probe(dev, rdev->regmap);
}

static int sx125x_con_remove(struct sx130x_radio_device *rdev)
{
	return sx125x_regmap_remove(&rdev->dev);
}

#ifdef CONFIG_OF
static const struct of_device_id sx125x_con_of_match[] = {
	{ .compatible = "semtech,sx1255" },
	{ .compatible = "semtech,sx1257" },
	{},
};
MODULE_DEVICE_TABLE(of, sx125x_con_of_match);
#endif

static struct sx130x_radio_driver sx125x_con_driver = {
	.probe  = sx125x_con_probe,
	.remove = sx125x_con_remove,
	.driver = {
		.name = "sx125x_con",
		.of_match_table = of_match_ptr(sx125x_con_of_match),
	},
};
#endif

#ifdef CONFIG_LORA_SX125X_SPI
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

	return sx125x_regmap_probe(&spi->dev, regmap);
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
#ifdef CONFIG_LORA_SX125X_CON
	ret = sx130x_register_radio_driver(&sx125x_con_driver);
	if (ret < 0) {
		pr_err("failed to init sx125x con (%d)\n", ret);
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
#ifdef CONFIG_LORA_SX125X_CON
	sx130x_unregister_radio_driver(&sx125x_con_driver);
#endif
}
module_exit(sx125x_exit);

MODULE_DESCRIPTION("Semtech SX125x LoRa Radio Driver");
MODULE_AUTHOR("Andreas Färber <afaerber@suse.de>");
MODULE_AUTHOR("Ben Whitten <ben.whitten@gmail.com>");
MODULE_LICENSE("GPL");
