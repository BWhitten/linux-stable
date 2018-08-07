// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Semtech SX1301 LoRa concentrator
 *
 * Copyright (c) 2018 Andreas Färber
 * Copyright (c) 2018 Ben Whitten
 *
 * Based on SX1301 HAL code:
 * Copyright (c) 2013 Semtech-Cycleo
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/lora.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/lora/dev.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>

#include "sx1301.h"

enum sx1301_fields {
	F_SOFT_RESET,
	F_START_BIST0,
	F_START_BIST1,
	F_BIST0_FINISHED,
	F_BIST1_FINISHED,
	F_GLOBAL_EN,
	F_CLK32M_EN,
	F_RADIO_A_EN,
	F_RADIO_B_EN,
	F_RADIO_RST,

	F_RX_INVERT_IQ,
	F_MODEM_INVERT_IQ,
	F_MBWSSF_MODEM_INVERT_IQ,
	F_RX_EDGE_SELECT,
	F_MISC_RADIO_EN,
	F_FSK_MODEM_INVERT_IQ,

	F_RSSI_BB_FILTER_ALPHA,
	F_RSSI_DEC_FILTER_ALPHA,
	F_RSSI_CHANN_FILTER_ALPHA,

	F_DEC_GAIN_OFFSET,
	F_CHAN_GAIN_OFFSET,

	F_LLR_SCALE,
	F_SNR_AVG_CST,

	F_CORR_NUM_SAME_PEAK,
	F_CORR_MAC_GAIN,

	F_FSK_CH_BW_EXPO,
	F_FSK_RSSI_LENGTH,
	F_FSK_RX_INVERT,
	F_FSK_PKT_MODE,

	F_FSK_PSIZE,
	F_FSK_CRC_EN,
	F_FSK_DCFREE_ENC,
	F_FSK_CRC_IBM,

	F_FSK_ERROR_OSR_TOL,
	F_FSK_RADIO_SELECT,

	F_TX_MODE,
	F_TX_ZERO_PAD,
	F_TX_EDGE_SELECT,
	F_TX_EDGE_SELECT_TOP,

	F_TX_GAIN,
	F_TX_CHIRP_LOW_PASS,
	F_TX_FCC_WIDEBAND,
	F_TX_SWAP_IQ,

	F_FSK_TX_GAUSSIAN_EN,
	F_FSK_TX_GAUSSIAN_SELECT_BT,
	F_FSK_TX_PATTERN_EN,
	F_FSK_TX_PREAMBLE_SEQ,
	F_FSK_TX_PSIZE,

	F_FORCE_HOST_RADIO_CTRL,
	F_FORCE_HOST_FE_CTRL,
	F_FORCE_DEC_FILTER_GAIN,

	F_MCU_RST_0,
	F_MCU_RST_1,
	F_MCU_SELECT_MUX_0,
	F_MCU_SELECT_MUX_1,
	F_MCU_CORRUPTION_DETECTED_0,
	F_MCU_CORRUPTION_DETECTED_1,
	F_MCU_SELECT_EDGE_0,
	F_MCU_SELECT_EDGE_1,

	F_EMERGENCY_FORCE_HOST_CTRL,
};

static const struct reg_field sx1301_reg_fields[] = {
	/* PAGE */
	[F_SOFT_RESET]		= REG_FIELD(SX1301_PAGE, 7, 7),
	/* BIST */
	[F_START_BIST0]		= REG_FIELD(SX1301_BIST, 0, 0),
	[F_START_BIST1]		= REG_FIELD(SX1301_BIST, 1, 1),
	/* BIST_S */
	[F_BIST0_FINISHED]	= REG_FIELD(SX1301_BIST_S, 0, 0),
	[F_BIST1_FINISHED]	= REG_FIELD(SX1301_BIST_S, 1, 1),
	/* GEN */
	[F_GLOBAL_EN]		= REG_FIELD(SX1301_GEN,  3, 3),
	/* CKEN */
	[F_CLK32M_EN]		= REG_FIELD(SX1301_CKEN, 0, 0),
	/* RADIO_CFG */
	[F_RADIO_A_EN]		= REG_FIELD(SX1301_RADIO_CFG, 0, 0),
	[F_RADIO_B_EN]		= REG_FIELD(SX1301_RADIO_CFG, 1, 1),
	[F_RADIO_RST]		= REG_FIELD(SX1301_RADIO_CFG, 2, 2),

	/* IQCFG */
	[F_RX_INVERT_IQ]	= REG_FIELD(SX1301_IQCFG, 0, 0),
	[F_MODEM_INVERT_IQ]	= REG_FIELD(SX1301_IQCFG, 1, 1),
	[F_MBWSSF_MODEM_INVERT_IQ]	= REG_FIELD(SX1301_IQCFG, 2, 2),
	[F_RX_EDGE_SELECT]	= REG_FIELD(SX1301_IQCFG, 3, 3),
	[F_MISC_RADIO_EN]	= REG_FIELD(SX1301_IQCFG, 4, 4),
	[F_FSK_MODEM_INVERT_IQ]	= REG_FIELD(SX1301_IQCFG, 5, 5),

	/* RSSI_X_FILTER_ALPHA */
	[F_RSSI_BB_FILTER_ALPHA] =
		REG_FIELD(SX1301_RSSI_BB_FILTER_ALPHA, 0, 4),
	[F_RSSI_DEC_FILTER_ALPHA] =
		REG_FIELD(SX1301_RSSI_DEC_FILTER_ALPHA, 0, 4),
	[F_RSSI_CHANN_FILTER_ALPHA] =
		REG_FIELD(SX1301_RSSI_CHANN_FILTER_ALPHA, 0, 4),

	/* GAIN_OFFSET */
	[F_DEC_GAIN_OFFSET]	= REG_FIELD(SX1301_GAIN_OFFSET, 0, 3),
	[F_CHAN_GAIN_OFFSET]	= REG_FIELD(SX1301_GAIN_OFFSET, 4, 7),

	/* MISC_CFG1 */
	[F_LLR_SCALE]		= REG_FIELD(SX1301_MISC_CFG1, 0, 3),
	[F_SNR_AVG_CST]		= REG_FIELD(SX1301_MISC_CFG1, 4, 5),

	/* CORR_CFG */
	[F_CORR_NUM_SAME_PEAK]	= REG_FIELD(SX1301_CORR_CFG, 0, 3),
	[F_CORR_MAC_GAIN]	= REG_FIELD(SX1301_CORR_CFG, 4, 6),

	/* FSK_CFG1 */
	[F_FSK_CH_BW_EXPO]	= REG_FIELD(SX1301_FSK_CFG1, 0, 2),
	[F_FSK_RSSI_LENGTH]	= REG_FIELD(SX1301_FSK_CFG1, 3, 5),
	[F_FSK_RX_INVERT]	= REG_FIELD(SX1301_FSK_CFG1, 6, 6),
	[F_FSK_PKT_MODE]	= REG_FIELD(SX1301_FSK_CFG1, 7, 7),

	/* FSK_CFG2 */
	[F_FSK_PSIZE]		= REG_FIELD(SX1301_FSK_CFG2, 0, 2),
	[F_FSK_CRC_EN]		= REG_FIELD(SX1301_FSK_CFG2, 3, 3),
	[F_FSK_DCFREE_ENC]	= REG_FIELD(SX1301_FSK_CFG2, 4, 5),
	[F_FSK_CRC_IBM]		= REG_FIELD(SX1301_FSK_CFG2, 6, 6),

	/* FSK_ERROR_OSR_TOL */
	[F_FSK_ERROR_OSR_TOL]	= REG_FIELD(SX1301_FSK_ERROR_OSR_TOL, 0, 4),
	[F_FSK_RADIO_SELECT]	= REG_FIELD(SX1301_FSK_ERROR_OSR_TOL, 7, 7),

	/* TX_CFG1 */
	[F_TX_MODE]		= REG_FIELD(SX1301_TX_CFG1, 0, 0),
	[F_TX_ZERO_PAD]		= REG_FIELD(SX1301_TX_CFG1, 1, 4),
	[F_TX_EDGE_SELECT]	= REG_FIELD(SX1301_TX_CFG1, 5, 5),
	[F_TX_EDGE_SELECT_TOP]	= REG_FIELD(SX1301_TX_CFG1, 6, 6),

	/* TX_CFG2 */
	[F_TX_GAIN]		= REG_FIELD(SX1301_TX_CFG2, 0, 1),
	[F_TX_CHIRP_LOW_PASS]	= REG_FIELD(SX1301_TX_CFG2, 2, 4),
	[F_TX_FCC_WIDEBAND]	= REG_FIELD(SX1301_TX_CFG2, 5, 6),
	[F_TX_SWAP_IQ]		= REG_FIELD(SX1301_TX_CFG2, 7, 7),

	/* FSK_TX */
	[F_FSK_TX_GAUSSIAN_EN]	= REG_FIELD(SX1301_FSK_TX, 0, 0),
	[F_FSK_TX_GAUSSIAN_SELECT_BT]	= REG_FIELD(SX1301_FSK_TX, 1, 2),
	[F_FSK_TX_PATTERN_EN]	= REG_FIELD(SX1301_FSK_TX, 3, 3),
	[F_FSK_TX_PREAMBLE_SEQ]	= REG_FIELD(SX1301_FSK_TX, 4, 4),
	[F_FSK_TX_PSIZE]	= REG_FIELD(SX1301_FSK_TX, 5, 7),

	/* FORCE_CTRL */
	[F_FORCE_HOST_RADIO_CTRL] = REG_FIELD(SX1301_FORCE_CTRL, 1, 1),
	[F_FORCE_HOST_FE_CTRL]    = REG_FIELD(SX1301_FORCE_CTRL, 2, 2),
	[F_FORCE_DEC_FILTER_GAIN] = REG_FIELD(SX1301_FORCE_CTRL, 3, 3),

	/* MCU_CTRL */
	[F_MCU_RST_0]		= REG_FIELD(SX1301_MCU_CTRL, 0, 0),
	[F_MCU_RST_1]		= REG_FIELD(SX1301_MCU_CTRL, 1, 1),
	[F_MCU_SELECT_MUX_0]	= REG_FIELD(SX1301_MCU_CTRL, 2, 2),
	[F_MCU_SELECT_MUX_1]	= REG_FIELD(SX1301_MCU_CTRL, 3, 3),
	[F_MCU_CORRUPTION_DETECTED_0] = REG_FIELD(SX1301_MCU_CTRL, 4, 4),
	[F_MCU_CORRUPTION_DETECTED_1] = REG_FIELD(SX1301_MCU_CTRL, 5, 5),
	[F_MCU_SELECT_EDGE_0]	= REG_FIELD(SX1301_MCU_CTRL, 6, 6),
	[F_MCU_SELECT_EDGE_1]	= REG_FIELD(SX1301_MCU_CTRL, 7, 7),

	/* EMERGENCY_FORCE_HOST_CTRL */
	[F_EMERGENCY_FORCE_HOST_CTRL] =
		REG_FIELD(SX1301_EMERGENCY_FORCE_HOST_CTRL, 0, 0),
};

static const struct regmap_range_cfg sx1301_ranges[] = {
	{
		.name = "Pages",

		.range_min = SX1301_VIRT_BASE,
		.range_max = SX1301_MAX_REGISTER,

		.selector_reg = SX1301_PAGE,
		.selector_mask = 0x3,

		.window_start = 0,
		.window_len = SX1301_PAGE_LEN,
	},
};

static struct regmap_config sx1301_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.cache_type = REGCACHE_NONE,

	.read_flag_mask = 0,
	.write_flag_mask = BIT(7),

	.ranges = sx1301_ranges,
	.num_ranges = ARRAY_SIZE(sx1301_ranges),
	.max_register = SX1301_MAX_REGISTER,
};

struct spi_sx1301 {
	struct spi_device *parent;
	u8 page;
	u8 regs;
};

struct sx1301_priv {
	struct device *dev;
	struct spi_device *spi;
	struct lora_priv lora;
	struct gpio_desc *rst_gpio;
	struct regmap		*regmap;
	struct regmap_field	*regmap_fields[ARRAY_SIZE(sx1301_reg_fields)];
};

static int sx1301_field_read(struct sx1301_priv *priv,
		enum sx1301_fields field_id)
{
	int ret;
	int val;

	ret = regmap_field_read(priv->regmap_fields[field_id], &val);
	if (ret)
		return ret;

	return val;
}

static int sx1301_field_write(struct sx1301_priv *priv,
		enum sx1301_fields field_id, u8 val)
{
	return regmap_field_write(priv->regmap_fields[field_id], val);
}

static int sx1301_read_burst(struct sx1301_priv *priv, u8 reg, u8 *val, size_t len)
{
	u8 addr = reg & 0x7f;
	return spi_write_then_read(priv->spi, &addr, 1, val, len);
}

static int sx1301_write_burst(struct sx1301_priv *priv, u8 reg, const u8 *val, size_t len)
{
	u8 addr = reg | BIT(7);
	struct spi_transfer xfr[2] = {
		{ .tx_buf = &addr, .len = 1 },
		{ .tx_buf = val, .len = len },
	};

	return spi_sync_transfer(priv->spi, xfr, 2);
}

static int sx1301_agc_ram_read(struct sx1301_priv *priv, u8 addr, u8 *val)
{
	int ret;
	unsigned int read;

	ret = regmap_write(priv->regmap, SX1301_DBG_AGC_MCU_RAM_ADDR, addr);
	if (ret) {
		dev_err(priv->dev, "AGC RAM addr write failed\n");
		return ret;
	}

	ret = regmap_read(priv->regmap, SX1301_DBG_AGC_MCU_RAM_DATA, &read);
	if (ret) {
		dev_err(priv->dev, "AGC RAM data read failed\n");
		return ret;
	}
	*val = read;

	return 0;
}

static int sx1301_arb_ram_read(struct sx1301_priv *priv, u8 addr, u8 *val)
{
	int ret;
	unsigned int read;

	ret = regmap_write(priv->regmap, SX1301_DBG_ARB_MCU_RAM_ADDR, addr);
	if (ret) {
		dev_err(priv->dev, "ARB RAM addr write failed\n");
		return ret;
	}

	ret = regmap_read(priv->regmap, SX1301_DBG_ARB_MCU_RAM_DATA, &read);
	if (ret) {
		dev_err(priv->dev, "ARB RAM data read failed\n");
		return ret;
	}
	*val = read;

	return 0;
}

static int sx1301_load_firmware(struct sx1301_priv *priv, int mcu, const struct firmware *fw)
{
	u8 *buf;
	int ret;
	unsigned int read;
	enum sx1301_fields rst, select_mux;

	if (fw->size > SX1301_MCU_FW_BYTE)
		return -EINVAL;

	switch (mcu) {
	case 0:
		rst = F_MCU_RST_0;
		select_mux = F_MCU_SELECT_MUX_0;
		break;
	case 1:
		rst = F_MCU_RST_1;
		select_mux = F_MCU_SELECT_MUX_1;
		break;
	default:
		return -EINVAL;
	}

	ret = sx1301_field_write(priv, rst, 1);
	if (ret)
		return ret;
	ret = sx1301_field_write(priv, select_mux, 0);
	if (ret)
		return ret;

	/* Load firmware into the data register */
	ret = regmap_write(priv->regmap, SX1301_MPA, 0);
	if (ret)
		return ret;

	ret = sx1301_write_burst(priv, SX1301_MPA, fw->data, fw->size);
	if (ret) {
		dev_err(priv->dev, "MCU prom data write failed\n");
		return ret;
	}

	ret = regmap_read(priv->regmap, SX1301_MPD, &read);
	if (ret) {
		dev_err(priv->dev, "MCU prom data dummy read failed\n");
		return ret;
	}

	buf = devm_kzalloc(priv->dev, fw->size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = sx1301_read_burst(priv, SX1301_MPD, buf, fw->size);
	if (ret) {
		dev_err(priv->dev, "MCU prom data read failed\n");
		return ret;
	}

	if (memcmp(fw->data, buf, fw->size)) {
		dev_err(priv->dev, "MCU prom data read does not match data written\n");
		return -ENXIO;
	}

	devm_kfree(priv->dev, buf);

	ret = sx1301_field_write(priv, select_mux, 1);
	if (ret)
		return ret;

	return 0;
}

static int sx1301_agc_calibrate(struct sx1301_priv *priv)
{
	const struct firmware *fw;
	u8 val;
	int ret;
	unsigned int cal;

	ret = request_firmware(&fw, "sx1301_agc_calibration.bin", priv->dev);
	if (ret) {
		dev_err(priv->dev, "AGC CAL firmware file load failed\n");
		return ret;
	}

	ret = sx1301_load_firmware(priv, 1, fw);
	release_firmware(fw);
	if (ret) {
		dev_err(priv->dev, "AGC CAL firmware load failed\n");
		return ret;
	}

	ret = sx1301_field_write(priv, F_FORCE_HOST_RADIO_CTRL, 0);
	if (ret)
		return ret;

	val = BIT(4); /* with DAC gain=3 */
	if (false)
		val |= BIT(5); /* SX1255 */

	ret = regmap_write(priv->regmap, SX1301_CHRS, val);
	if (ret)
		return ret;

	ret = sx1301_field_write(priv, F_MCU_RST_1, 0);
	if (ret)
		return ret;

	ret = sx1301_agc_ram_read(priv, 0x20, &val);
	if (ret) {
		dev_err(priv->dev, "AGC RAM data read failed\n");
		return ret;
	}

	dev_info(priv->dev, "AGC calibration firmware version %u\n", (unsigned)val);

	if (val != SX1301_MCU_AGC_CAL_FW_VERSION) {
		dev_err(priv->dev, "unexpected firmware version, expecting %u\n",
				SX1301_MCU_AGC_CAL_FW_VERSION);
		return -ENXIO;
	}

	ret = sx1301_field_write(priv, F_EMERGENCY_FORCE_HOST_CTRL, 0);
	if (ret)
		return ret;

	dev_err(priv->dev, "starting calibration...\n");
	msleep(2300);

	ret = sx1301_field_write(priv, F_EMERGENCY_FORCE_HOST_CTRL, 1);
	if (ret)
		return ret;

	ret = regmap_read(priv->regmap, SX1301_AGCSTS, &cal);
	if (ret)
		return ret;

	dev_info(priv->dev, "AGC status: %02x\n", cal);
	if ((cal & (BIT(7) | BIT(0))) != (BIT(7) | BIT(0))) {
		dev_err(priv->dev, "AGC calibration failed\n");
		return -ENXIO;
	}

	return 0;
}

static int sx1301_load_all_firmware(struct sx1301_priv *priv)
{
	const struct firmware *fw;
	u8 val;
	int ret;

	ret = request_firmware(&fw, "sx1301_arb.bin", priv->dev);
	if (ret) {
		dev_err(priv->dev, "ARB firmware file load failed\n");
		return ret;
	}

	ret = sx1301_load_firmware(priv, 0, fw);
	release_firmware(fw);
	if (ret)
		return ret;

	ret = request_firmware(&fw, "sx1301_agc.bin", priv->dev);
	if (ret) {
		dev_err(priv->dev, "AGC firmware file load failed\n");
		return ret;
	}

	ret = sx1301_load_firmware(priv, 1, fw);
	release_firmware(fw);
	if (ret)
		return ret;

	ret = sx1301_field_write(priv, F_FORCE_HOST_RADIO_CTRL, 0);
	if (ret)
		return ret;
	ret = sx1301_field_write(priv, F_FORCE_HOST_FE_CTRL, 0);
	if (ret)
		return ret;
	ret = sx1301_field_write(priv, F_FORCE_DEC_FILTER_GAIN, 0);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, SX1301_CHRS, 0);
	if (ret)
		return ret;

	/* Release the CPUs */
	ret = sx1301_field_write(priv, F_MCU_RST_0, 0);
	if (ret)
		return ret;
	ret = sx1301_field_write(priv, F_MCU_RST_1, 0);
	if (ret)
		return ret;

	ret = sx1301_agc_ram_read(priv, 0x20, &val);
	if (ret) {
		dev_err(priv->dev, "AGC RAM data read failed\n");
		return ret;
	}

	dev_info(priv->dev, "AGC firmware version %u\n", (unsigned)val);

	if (val != SX1301_MCU_AGC_FW_VERSION) {
		dev_err(priv->dev, "unexpected firmware version, expecting %u\n",
				SX1301_MCU_AGC_FW_VERSION);
		return -ENXIO;
	}

	ret = sx1301_arb_ram_read(priv, 0x20, &val);
	if (ret) {
		dev_err(priv->dev, "ARB RAM data read failed\n");
		return ret;
	}

	dev_info(priv->dev, "ARB firmware version %u\n", (unsigned)val);

	if (val != SX1301_MCU_ARB_FW_VERSION) {
		dev_err(priv->dev, "unexpected firmware version, expecting %u\n",
				SX1301_MCU_ARB_FW_VERSION);
		return -ENXIO;
	}

	return 0;
}

static int sx1301_regmap_bus_write(void *context, unsigned int reg,
		unsigned int val)
{
	struct device *dev = context;
	struct sx1301_priv *priv = dev_get_drvdata(dev->parent);
	unsigned int addr, data, cs, rb;
	u32 radio;
	int ret;

	/* Device address */
	ret = of_property_read_u32(dev->of_node, "reg", &radio);
	if (ret)
		return ret;

	if (radio == 0) {
		addr = SX1301_RADIO_A_SPI_ADDR;
		data = SX1301_RADIO_A_SPI_DATA;
		cs = SX1301_RADIO_A_SPI_CS;
		rb = SX1301_RADIO_A_SPI_DATA_RB;
	} else {
		addr = SX1301_RADIO_B_SPI_ADDR;
		data = SX1301_RADIO_B_SPI_DATA;
		cs = SX1301_RADIO_B_SPI_CS;
		rb = SX1301_RADIO_B_SPI_DATA_RB;
	}

	ret = regmap_write(priv->regmap, cs, 0);
	if (ret)
		return ret;
	ret = regmap_write(priv->regmap, addr, reg);
	if (ret)
		return ret;
	ret = regmap_write(priv->regmap, data, val);
	if (ret)
		return ret;
	ret = regmap_write(priv->regmap, cs, 1);
	if (ret)
		return ret;
	ret = regmap_write(priv->regmap, cs, 0);
	if (ret)
		return ret;

	return 0;
}

static int sx1301_regmap_bus_read(void *context, unsigned int reg,
		unsigned int *val)
{
	struct device *dev = context;
	struct sx1301_priv *priv = dev_get_drvdata(dev->parent);
	unsigned int addr, data, cs, rb;
	u32 radio;
	int ret;

	/* Device address */
	ret = of_property_read_u32(dev->of_node, "reg", &radio);
	if (ret)
		return ret;

	if (radio == 0) {
		addr = SX1301_RADIO_A_SPI_ADDR;
		data = SX1301_RADIO_A_SPI_DATA;
		cs = SX1301_RADIO_A_SPI_CS;
		rb = SX1301_RADIO_A_SPI_DATA_RB;
	} else {
		addr = SX1301_RADIO_B_SPI_ADDR;
		data = SX1301_RADIO_B_SPI_DATA;
		cs = SX1301_RADIO_B_SPI_CS;
		rb = SX1301_RADIO_B_SPI_DATA_RB;
	}

	ret = regmap_write(priv->regmap, cs, 0);
	if (ret)
		return ret;
	/* address to tx */
	ret = regmap_write(priv->regmap, addr, reg);
	if (ret)
		return ret;
	ret = regmap_write(priv->regmap, data, 0);
	if (ret)
		return ret;
	ret = regmap_write(priv->regmap, cs, 1);
	if (ret)
		return ret;
	ret = regmap_write(priv->regmap, cs, 0);
	if (ret)
		return ret;

	/* reading */
	return regmap_read(priv->regmap, rb, val);
}

static const struct regmap_bus sx1301_regmap_bus = {
	.reg_write = sx1301_regmap_bus_write,
	.reg_read = sx1301_regmap_bus_read,
};

const struct regmap_bus *sx1301_concentrator_regmap_bus(void)
{
	return &sx1301_regmap_bus;
}
EXPORT_SYMBOL_GPL(sx1301_concentrator_regmap_bus);

static int sx1301_probe(struct spi_device *spi)
{
	struct net_device *netdev;
	struct sx1301_priv *priv;
	struct gpio_desc *rst;
	int ret;
	int i;
	unsigned int ver;

	rst = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(rst))
		return PTR_ERR(rst);

	gpiod_set_value_cansleep(rst, 1);
	msleep(100);
	gpiod_set_value_cansleep(rst, 0);
	msleep(100);

	spi->bits_per_word = 8;
	spi_setup(spi);

	netdev = alloc_loradev(sizeof(*priv));
	if (!netdev)
		return -ENOMEM;

	ret = devm_lora_register_netdev(&spi->dev, netdev);
	if (ret) {
		free_loradev(netdev);
		return ret;
	}

	priv = netdev_priv(netdev);
	priv->rst_gpio = rst;

	spi_set_drvdata(spi, priv);
	SET_NETDEV_DEV(netdev, &spi->dev);
	priv->dev = &spi->dev;
	priv->spi = spi;

	priv->regmap = devm_regmap_init_spi(spi, &sx1301_regmap_config);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err(&spi->dev, "Regmap allocation failed: %d\n", ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(sx1301_reg_fields); i++) {
		const struct reg_field *reg_fields = sx1301_reg_fields;

		priv->regmap_fields[i] = devm_regmap_field_alloc(&spi->dev,
				priv->regmap,
				reg_fields[i]);
		if (IS_ERR(priv->regmap_fields[i])) {
			ret = PTR_ERR(priv->regmap_fields[i]);
			dev_err(&spi->dev, "Cannot allocate regmap field: %d\n", ret);
			return ret;
		}
	}

	ret = regmap_read(priv->regmap, SX1301_VER, &ver);
	if (ret) {
		dev_err(&spi->dev, "version read failed\n");
		return ret;
	}

	if (ver != SX1301_CHIP_VERSION) {
		dev_err(&spi->dev, "unexpected version: %u\n", ver);
		return -ENXIO;
	}

	ret = regmap_write(priv->regmap, SX1301_PAGE, 0);
	if (ret) {
		dev_err(&spi->dev, "page/reset write failed\n");
		return ret;
	}

	ret = sx1301_field_write(priv, F_SOFT_RESET, 1);
	if (ret) {
		dev_err(&spi->dev, "soft reset failed\n");
		return ret;
	}

	/* gate clocks */
	ret = sx1301_field_write(priv, F_GLOBAL_EN, 0);
	if (ret)
		return ret;
	ret = sx1301_field_write(priv, F_CLK32M_EN, 0);
	if (ret)
		return ret;

	/* switch on and reset the radios (also starts the 32 MHz XTAL) */
	ret = sx1301_field_write(priv, F_RADIO_A_EN, 1);
	if (ret)
		return ret;
	ret = sx1301_field_write(priv, F_RADIO_B_EN, 1);
	if (ret)
		return ret;
	mdelay(500);
	ret = sx1301_field_write(priv, F_RADIO_RST, 1);
	if (ret)
		return ret;
	mdelay(5);
	ret = sx1301_field_write(priv, F_RADIO_RST, 0);
	if (ret)
		return ret;

	/* probe platform */
	ret = devm_of_platform_populate(priv->dev);
	if (ret)
		return ret;

	/* GPIO */
	ret = regmap_write(priv->regmap, SX1301_GPMODE, 0x1F);
	if (ret)
		return ret;
	ret = regmap_write(priv->regmap, SX1301_GPSO, 0x2);
	if (ret)
		return ret;

	/* TODO LBT */

	/* start clocks */
	ret = sx1301_field_write(priv, F_GLOBAL_EN, 1);
	if (ret)
		return ret;
	ret = sx1301_field_write(priv, F_CLK32M_EN, 1);
	if (ret)
		return ret;

	/* calibration */

	ret = sx1301_agc_calibrate(priv);
	if (ret)
		return ret;

	/* TODO */

	ret = sx1301_load_all_firmware(priv);
	if (ret)
		return ret;

	dev_info(&spi->dev, "SX1301 module probed\n");

	return 0;
}

static int sx1301_remove(struct spi_device *spi)
{
	struct net_device *netdev = spi_get_drvdata(spi);

	//unregister_loradev(netdev);
	free_loradev(netdev);

	dev_info(&spi->dev, "SX1301 module removed\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sx1301_dt_ids[] = {
	{ .compatible = "semtech,sx1301" },
	{}
};
MODULE_DEVICE_TABLE(of, sx1301_dt_ids);
#endif

static struct spi_driver sx1301_spi_driver = {
	.driver = {
		.name = "sx1301",
		.of_match_table = of_match_ptr(sx1301_dt_ids),
	},
	.probe = sx1301_probe,
	.remove = sx1301_remove,
};

module_spi_driver(sx1301_spi_driver);

MODULE_DESCRIPTION("SX1301 SPI driver");
MODULE_AUTHOR("Andreas Färber <afaerber@suse.de>");
MODULE_AUTHOR("Ben Whitten <ben.whitten@gmail.com>");
MODULE_LICENSE("GPL");
