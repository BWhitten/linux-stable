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
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/lora/dev.h>
#include <linux/spi/spi.h>

#include "sx130x.h"

struct sx130x_fields_sequence {
	enum sx130x_fields field;
	u8 val;
};

static const struct sx130x_fields_sequence sx130x_regmap_fields_patch[] = {
	/* I/Q path setup */
	{F_RSSI_BB_FILTER_ALPHA,	6},
	{F_RSSI_DEC_FILTER_ALPHA,	7},
	{F_RSSI_CHANN_FILTER_ALPHA,	7},
	{F_RSSI_BB_DEFAULT_VALUE,	23},
	{F_RSSI_DEC_DEFAULT_VALUE,	66},
	{F_RSSI_CHANN_DEFAULT_VALUE,	85},
	{F_DEC_GAIN_OFFSET,		7},
	{F_CHAN_GAIN_OFFSET,		6},

	/* LoRa 'multi' demodulator setup */
	{F_SNR_AVG_CST,			3},
	{F_FRAME_SYNCH_PEAK1_POS,	3}, // Public LoRa network
	{F_FRAME_SYNCH_PEAK2_POS,	4}, // Public LoRa network

	/* LoRa standalone 'MBWSSF' demodulator setup */
	{F_MBWSSF_FRAME_SYNCH_PEAK1_POS, 3}, // Public LoRa network
	{F_MBWSSF_FRAME_SYNCH_PEAK2_POS, 4}, // Public LoRa network

	/* Improvement of ref clock freq error tolerance */
	{F_ADJUST_MODEM_START_OFFSET_RDX4L, 1},
	{F_ADJUST_MODEM_START_OFFSET_SF12_RDX4L, (4094 & 0xFF)},
	{F_ADJUST_MODEM_START_OFFSET_SF12_RDX4H, (4094 >> 8)},
	{F_CORR_MAC_GAIN,		7},

	/* FSK datapath setup */
	{F_FSK_RX_INVERT,		1},
	{F_FSK_MODEM_INVERT_IQ,		1},

	/* FSK demodulator setup */
	{F_FSK_RSSI_LENGTH,		4},
	{F_FSK_PKT_MODE,		1},
	{F_FSK_CRC_EN,			1},
	{F_FSK_DCFREE_ENC,		2},
	{F_FSK_ERROR_OSR_TOL,		10},
	{F_FSK_PKT_LENGTH,		255},
	{F_FSK_PATTERN_TIMEOUT_CFGL,	128},

	/* TX general parameters */
	{F_TX_START_DELAYL, (TX_START_DELAY_DEFAULT & 0xFF)},
	{F_TX_START_DELAYH, (TX_START_DELAY_DEFAULT >> 8)},

	/* TX LoRa */
	{F_TX_SWAP_IQ,			1},
	{F_TX_FRAME_SYNCH_PEAK1_POS,	3}, // Public LoRa network
	{F_TX_FRAME_SYNCH_PEAK2_POS,	4}, // Public LoRa network

	/* TX FSK */
	{F_FSK_TX_GAUSSIAN_SELECT_BT,	2},
};

static const struct reg_field sx130x_regmap_fields[] = {
	/* PAGE */
	[F_SOFT_RESET]          = REG_FIELD(SX1301_PAGE, 7, 7),
	/* GEN */
	[F_MBWSSF_MODEM_EN]     = REG_FIELD(SX1301_GEN,  0, 0),
	[F_FSK_MODEM_EN]        = REG_FIELD(SX1301_GEN,  2, 2),
	[F_GLOBAL_EN]           = REG_FIELD(SX1301_GEN,  3, 3),
	/* CKEN */
	[F_CLK32M_EN]           = REG_FIELD(SX1301_CKEN, 0, 0),
	/* RADIO_CFG */
	[F_RADIO_A_EN]          = REG_FIELD(SX1301_RADIO_CFG, 0, 0),
	[F_RADIO_B_EN]          = REG_FIELD(SX1301_RADIO_CFG, 1, 1),
	[F_RADIO_RST]           = REG_FIELD(SX1301_RADIO_CFG, 2, 2),
	/* MCU_CTRL */
	[F_MCU_RST_0]           = REG_FIELD(SX1301_MCU_CTRL, 0, 0),
	[F_MCU_RST_1]           = REG_FIELD(SX1301_MCU_CTRL, 1, 1),
	[F_MCU_SELECT_MUX_0]    = REG_FIELD(SX1301_MCU_CTRL, 2, 2),
	[F_MCU_SELECT_MUX_1]    = REG_FIELD(SX1301_MCU_CTRL, 3, 3),
	/* FORCE_CTRL */
	[F_FORCE_HOST_RADIO_CTRL] = REG_FIELD(SX1301_FORCE_CTRL, 1, 1),
	[F_FORCE_HOST_FE_CTRL]    = REG_FIELD(SX1301_FORCE_CTRL, 2, 2),
	[F_FORCE_DEC_FILTER_GAIN] = REG_FIELD(SX1301_FORCE_CTRL, 3, 3),
	/* EMERGENCY_FORCE_HOST_CTRL */
	[F_EMERGENCY_FORCE_HOST_CTRL] =
		REG_FIELD(SX1301_EMERGENCY_FORCE_HOST_CTRL, 0, 0),

	/* RSSI_X_FILTER_ALPHA */
	[F_RSSI_BB_FILTER_ALPHA] = REG_FIELD(SX1301_RSSI_BB_FILTER_ALPHA, 0, 4),
	[F_RSSI_DEC_FILTER_ALPHA] = REG_FIELD(SX1301_RSSI_DEC_FILTER_ALPHA, 0, 4),
	[F_RSSI_CHANN_FILTER_ALPHA]	= REG_FIELD(SX1301_RSSI_CHANN_FILTER_ALPHA, 0, 4),
	/* RSSI_X_DEFAULT_VALUE */
	[F_RSSI_BB_DEFAULT_VALUE] = REG_FIELD(SX1301_RSSI_BB_DEFAULT_VALUE, 0, 7),
	[F_RSSI_DEC_DEFAULT_VALUE] = REG_FIELD(SX1301_RSSI_DEC_DEFAULT_VALUE, 0, 7),
	[F_RSSI_CHANN_DEFAULT_VALUE] = REG_FIELD(SX1301_RSSI_CHANN_DEFAULT_VALUE, 0, 7),
	/* GAIN_OFFSET */
	[F_DEC_GAIN_OFFSET]	= REG_FIELD(SX1301_GAIN_OFFSET, 0, 3),
	[F_CHAN_GAIN_OFFSET]	= REG_FIELD(SX1301_GAIN_OFFSET, 4, 7),

	[F_SNR_AVG_CST] = REG_FIELD(SX1301_MISC_CFG1, 4, 5),
	[F_FRAME_SYNCH_PEAK1_POS] = REG_FIELD(SX1301_FRAME_SYNCH, 0, 3),
	[F_FRAME_SYNCH_PEAK2_POS] = REG_FIELD(SX1301_FRAME_SYNCH, 4, 7),

	[F_MBWSSF_FRAME_SYNCH_PEAK1_POS] = REG_FIELD(SX1301_BHSYNCPOS, 0, 3),
	[F_MBWSSF_FRAME_SYNCH_PEAK2_POS] = REG_FIELD(SX1301_BHSYNCPOS, 4, 7),

	[F_ADJUST_MODEM_START_OFFSET_RDX4L] =
		REG_FIELD(SX1301_MODEM_START_RDX4L, 0, 7),  // 12 bits
	[F_ADJUST_MODEM_START_OFFSET_RDX4H] =
		REG_FIELD(SX1301_MODEM_START_RDX4H, 0, 3),
	[F_ADJUST_MODEM_START_OFFSET_SF12_RDX4L] =
		REG_FIELD(SX1301_MODEM_START_SF12_RDX4L, 0, 7), // 12 bits
	[F_ADJUST_MODEM_START_OFFSET_SF12_RDX4H] =
		REG_FIELD(SX1301_MODEM_START_SF12_RDX4H, 0, 3),
	[F_CORR_MAC_GAIN] = REG_FIELD(SX1301_CORR_CFG, 4, 6),

	[F_FSK_RSSI_LENGTH] = REG_FIELD(SX1301_FSK_CFG1, 3, 5),
	[F_FSK_RX_INVERT] = REG_FIELD(SX1301_FSK_CFG1, 6, 6),
	[F_FSK_PKT_MODE] = REG_FIELD(SX1301_FSK_CFG1, 7, 7),

	[F_FSK_MODEM_INVERT_IQ] = REG_FIELD(SX1301_IQCFG, 5, 5),

	[F_FSK_PSIZE] = REG_FIELD(SX1301_FSK_CFG2, 0, 2),
	[F_FSK_CRC_EN] = REG_FIELD(SX1301_FSK_CFG2, 3, 3),
	[F_FSK_DCFREE_ENC] = REG_FIELD(SX1301_FSK_CFG2, 4, 5),
	[F_FSK_ERROR_OSR_TOL] = REG_FIELD(SX1301_FSK_ERROR_OSR_TOL, 0, 4),
	[F_FSK_PKT_LENGTH] = REG_FIELD(SX1301_FSK_PKT_LENGTH, 0, 7),
	[F_FSK_PATTERN_TIMEOUT_CFGL] =
		REG_FIELD(SX1301_FSK_PATTERN_TIMEOUT_CFGL, 0, 7),  // 10 bits
	[F_FSK_PATTERN_TIMEOUT_CFGH] =
		REG_FIELD(SX1301_FSK_PATTERN_TIMEOUT_CFGH, 0, 1),

	[F_TX_START_DELAYL] = REG_FIELD(SX1301_TX_START_DELAYL, 0, 7), // 16 bit
	[F_TX_START_DELAYH] = REG_FIELD(SX1301_TX_START_DELAYH, 0, 7),

	[F_TX_SWAP_IQ] = REG_FIELD(SX1301_TX_CFG2, 7, 7),
	[F_TX_FRAME_SYNCH_PEAK1_POS] = REG_FIELD(SX1301_TX_FRAME_SYNCH, 0, 3),
	[F_TX_FRAME_SYNCH_PEAK2_POS] = REG_FIELD(SX1301_TX_FRAME_SYNCH, 4, 7),

	[F_FSK_TX_GAUSSIAN_SELECT_BT] = REG_FIELD(SX1301_FSK_TX, 1, 2),
	[F_FSK_TX_PSIZE] = REG_FIELD(SX1301_FSK_TX, 5, 7),
};

struct sx130x_tx_gain_lut {
	s8 power;	/* dBm measured at board connector */
	u8 dig_gain;
	u8 pa_gain;
	u8 dac_gain;
	u8 mix_gain;
};

struct sx130x_priv {
	struct lora_dev_priv	lora;
	struct device		*dev;
	struct gpio_desc	*rst_gpio;
	struct gpio_desc	*gpio[SX1301_NUM_GPIOS];
	struct regmap		*regmap;
	struct regmap_field	*regmap_fields[ARRAY_SIZE(sx130x_regmap_fields)];
	struct mutex		io_lock;
	void			*drvdata;
	struct sx130x_tx_gain_lut tx_gain_lut[SX1301_TX_GAIN_LUT_MAX];
	u8 tx_gain_lut_size;
};

struct regmap *sx130x_get_regmap(struct device *dev)
{
	struct net_device *netdev = dev_get_drvdata(dev);
	struct sx130x_priv *priv = netdev_priv(netdev);

	return priv->regmap;
}

void sx130x_set_drvdata(struct device *dev, void *drvdata)
{
	struct net_device *netdev = dev_get_drvdata(dev);
	struct sx130x_priv *priv = netdev_priv(netdev);

	priv->drvdata = drvdata;
}
EXPORT_SYMBOL_GPL(sx130x_set_drvdata);

void *sx130x_get_drvdata(struct device *dev)
{
	struct net_device *netdev = dev_get_drvdata(dev);
	struct sx130x_priv *priv = netdev_priv(netdev);

	return priv->drvdata;
}
EXPORT_SYMBOL_GPL(sx130x_get_drvdata);

void sx130x_io_lock(struct device *dev)
{
	struct net_device *netdev = dev_get_drvdata(dev);
	struct sx130x_priv *priv = netdev_priv(netdev);

	mutex_lock(&priv->io_lock);
}

void sx130x_io_unlock(struct device *dev)
{
	struct net_device *netdev = dev_get_drvdata(dev);
	struct sx130x_priv *priv = netdev_priv(netdev);

	mutex_unlock(&priv->io_lock);
}

static const struct regmap_range_cfg sx130x_regmap_ranges[] = {
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

static bool sx130x_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SX1301_MPD:
	case SX1301_AGCSTS:

	case SX1301_CHRS:
	case SX1301_MCU_CTRL:

	case SX1301_RADIO_A_SPI_DATA_RB:
	case SX1301_RADIO_B_SPI_DATA_RB:
	case SX1301_DBG_ARB_MCU_RAM_DATA:
	case SX1301_DBG_AGC_MCU_RAM_DATA:
		return true;
	default:
		return false;
	}
}

static bool sx130x_writeable_noinc_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SX1301_MPD:
		return true;
	default:
		return false;
	}
}

static bool sx130x_readable_noinc_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SX1301_MPD:
		return true;
	default:
		return false;
	}
}

const struct regmap_config sx130x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.cache_type = REGCACHE_RBTREE,
	.disable_locking = true,

	.read_flag_mask = 0,
	.write_flag_mask = BIT(7),

	.volatile_reg = sx130x_volatile_reg,
	.writeable_noinc_reg = sx130x_writeable_noinc_reg,
	.readable_noinc_reg = sx130x_readable_noinc_reg,

	.ranges = sx130x_regmap_ranges,
	.num_ranges = ARRAY_SIZE(sx130x_regmap_ranges),
	.max_register = SX1301_MAX_REGISTER,
};
EXPORT_SYMBOL_GPL(sx130x_regmap_config);

static inline int sx130x_field_write(struct sx130x_priv *priv,
		enum sx130x_fields field_id, u8 val)
{
	return regmap_field_write(priv->regmap_fields[field_id], val);
}

static inline int sx130x_field_force_write(struct sx130x_priv *priv,
		enum sx130x_fields field_id, u8 val)
{
	return regmap_field_force_write(priv->regmap_fields[field_id], val);
}

static int sx130x_soft_reset(struct sx130x_priv *priv)
{
	int ret;

	regcache_cache_bypass(priv->regmap, true);
	ret = sx130x_field_write(priv, F_SOFT_RESET, 1);
	regcache_cache_bypass(priv->regmap, false);
	if (ret)
		return ret;

	regcache_mark_dirty(priv->regmap);
	if (sx130x_regmap_config.cache_type != REGCACHE_NONE)
		return regcache_drop_region(priv->regmap,
			0, sx130x_regmap_config.max_register);
	return 0;
}

static int sx130x_fields_patch(struct sx130x_priv *priv)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(sx130x_regmap_fields_patch); i++) {
		ret = sx130x_field_force_write(priv, sx130x_regmap_fields_patch[i].field,
					       sx130x_regmap_fields_patch[i].val);
		if (ret) {
			dev_err(priv->dev,
				"Failed to patch regmap field: %d\n", i);
			break;
		}
	}

	return ret;
}

static int sx130x_agc_ram_read(struct sx130x_priv *priv, u8 addr, unsigned int *val)
{
	int ret;

	ret = regmap_write(priv->regmap, SX1301_DBG_AGC_MCU_RAM_ADDR, addr);
	if (ret) {
		dev_err(priv->dev, "AGC RAM addr write failed\n");
		return ret;
	}

	ret = regmap_read(priv->regmap, SX1301_DBG_AGC_MCU_RAM_DATA, val);
	if (ret) {
		dev_err(priv->dev, "AGC RAM data read failed\n");
		return ret;
	}

	return 0;
}

static int sx130x_arb_ram_read(struct sx130x_priv *priv, u8 addr, unsigned int *val)
{
	int ret;

	ret = regmap_write(priv->regmap, SX1301_DBG_ARB_MCU_RAM_ADDR, addr);
	if (ret) {
		dev_err(priv->dev, "ARB RAM addr write failed\n");
		return ret;
	}

	ret = regmap_read(priv->regmap, SX1301_DBG_ARB_MCU_RAM_DATA, val);
	if (ret) {
		dev_err(priv->dev, "ARB RAM data read failed\n");
		return ret;
	}

	return 0;
}

static int sx130x_load_firmware(struct sx130x_priv *priv, int mcu, const struct firmware *fw)
{
	u8 *buf;
	enum sx130x_fields rst, select_mux;
	unsigned int val;
	int ret;

	if (fw->size != SX1301_MCU_FW_BYTE) {
		dev_err(priv->dev, "Unexpected firmware size\n");
		return -EINVAL;
	}

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

	ret = sx130x_field_write(priv, rst, 1);
	if (ret) {
		dev_err(priv->dev, "MCU reset failed\n");
		return ret;
	}

	ret = sx130x_field_write(priv, select_mux, 0);
	if (ret) {
		dev_err(priv->dev, "MCU RAM select mux failed\n");
		return ret;
	}

	ret = regmap_write(priv->regmap, SX1301_MPA, 0);
	if (ret) {
		dev_err(priv->dev, "MCU prom addr write failed\n");
		return ret;
	}

	regcache_cache_bypass(priv->regmap, true);
	ret = regmap_noinc_write(priv->regmap, SX1301_MPD, fw->data, fw->size);
	if (ret) {
		dev_err(priv->dev, "MCU prom data write failed\n");
		return ret;
	}
	regcache_cache_bypass(priv->regmap, false);

	ret = regmap_read(priv->regmap, SX1301_MPD, &val);
	if (ret) {
		dev_err(priv->dev, "MCU prom data dummy read failed\n");
		return ret;
	}

	buf = kzalloc(fw->size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	regcache_cache_bypass(priv->regmap, true);
	ret = regmap_noinc_read(priv->regmap, SX1301_MPD, buf, fw->size);
	if (ret) {
		dev_err(priv->dev, "MCU prom data read failed\n");
		kfree(buf);
		return ret;
	}
	regcache_cache_bypass(priv->regmap, false);

	if (memcmp(fw->data, buf, fw->size)) {
		dev_err(priv->dev, "MCU prom data read does not match data written\n");
		kfree(buf);
		return -ENXIO;
	}

	kfree(buf);

	ret = sx130x_field_force_write(priv, select_mux, 1);
	if (ret) {
		dev_err(priv->dev, "MCU RAM release mux failed\n");
		return ret;
	}

	return 0;
}

static int sx130x_agc_transaction(struct sx130x_priv *priv, unsigned int val,
				  unsigned int *status)
{
	int ret;

	ret = regmap_write(priv->regmap, SX1301_CHRS, SX1301_AGC_CMD_WAIT);
	if (ret) {
		dev_err(priv->dev, "AGC transaction start failed\n");
		return ret;
	}
	usleep_range(1000, 2000);

	ret = regmap_write(priv->regmap, SX1301_CHRS, val);
	if (ret) {
		dev_err(priv->dev, "AGC transaction value failed\n");
		return ret;
	}
	usleep_range(1000, 2000);

	ret = regmap_read(priv->regmap, SX1301_AGCSTS, status);
	if (ret) {
		dev_err(priv->dev, "AGC status read failed\n");
		return ret;
	}

	return 0;
}

static int sx130x_agc_calibrate(struct sx130x_priv *priv)
{
	const struct firmware *fw;
	unsigned int val;
	int ret;

	ret = request_firmware(&fw, "sx1301_agc_calibration.bin", priv->dev);
	if (ret) {
		dev_err(priv->dev, "agc cal firmware file load failed\n");
		return ret;
	}

	ret = sx130x_load_firmware(priv, 1, fw);
	release_firmware(fw);
	if (ret) {
		dev_err(priv->dev, "agc cal firmware load failed\n");
		return ret;
	}

	ret = sx130x_field_force_write(priv, F_FORCE_HOST_RADIO_CTRL, 0);
	if (ret) {
		dev_err(priv->dev, "force host control failed\n");
		return ret;
	}

	val = BIT(4); /* with DAC gain=3 */
	if (false)
		val |= BIT(5); /* SX1255 */

	ret = regmap_write(priv->regmap, SX1301_CHRS, val);
	if (ret) {
		dev_err(priv->dev, "radio select write failed\n");
		return ret;
	}

	ret = sx130x_field_force_write(priv, F_MCU_RST_1, 0);
	if (ret) {
		dev_err(priv->dev, "MCU 1 reset failed\n");
		return ret;
	}

	ret = sx130x_agc_ram_read(priv, 0x20, &val);
	if (ret) {
		dev_err(priv->dev, "AGC RAM data read failed\n");
		return ret;
	}

	dev_info(priv->dev, "AGC calibration firmware version %u\n", (unsigned)val);

	if (val != SX1301_MCU_AGC_CAL_FW_VERSION) {
		dev_err(priv->dev, "unexpected firmware version, expecting %u\n",
				SX1301_MCU_AGC_CAL_FW_VERSION);
		return -EIO;
	}

	ret = sx130x_field_force_write(priv, F_EMERGENCY_FORCE_HOST_CTRL, 0);
	if (ret) {
		dev_err(priv->dev, "emergency force failed\n");
		return ret;
	}

	dev_err(priv->dev, "starting calibration...\n");
	msleep(2300);

	ret = sx130x_field_write(priv, F_EMERGENCY_FORCE_HOST_CTRL, 1);
	if (ret) {
		dev_err(priv->dev, "emergency force release failed\n");
		return ret;
	}

	ret = regmap_read(priv->regmap, SX1301_AGCSTS, &val);
	if (ret) {
		dev_err(priv->dev, "AGC status read failed\n");
		return ret;
	}

	dev_info(priv->dev, "AGC status: %02x\n", (unsigned)val);
	if ((val & (BIT(7) | BIT(0))) != (BIT(7) | BIT(0))) {
		dev_err(priv->dev, "AGC calibration failed\n");
		return -EIO;
	}

	return 0;
}

static int sx130x_enable_correlators(struct sx130x_priv *priv)
{
	int ret;
	int i;

	for (i = 0; i < 8; i++) {
		/* Multi */
		ret = regmap_write(priv->regmap, SX1301_COR0DETEN + i, 0x7E);
		if (ret) {
			dev_err(priv->dev, "correlator %d setup failed\n", i);
			return ret;
		}
	}

	return 0;
}

static int sx130x_load_all_firmware(struct sx130x_priv *priv)
{
	const struct firmware *fw;
	unsigned int val;
	int ret;

	ret = request_firmware(&fw, "sx1301_arb.bin", priv->dev);
	if (ret) {
		dev_err(priv->dev, "arb firmware file load failed\n");
		return ret;
	}

	ret = sx130x_load_firmware(priv, 0, fw);
	release_firmware(fw);
	if (ret)
		return ret;

	ret = request_firmware(&fw, "sx1301_agc.bin", priv->dev);
	if (ret) {
		dev_err(priv->dev, "agc firmware file load failed\n");
		return ret;
	}

	ret = sx130x_load_firmware(priv, 1, fw);
	release_firmware(fw);
	if (ret)
		return ret;

	ret = sx130x_field_write(priv, F_FORCE_HOST_RADIO_CTRL, 0);
	if (ret)
		return ret;
	ret = sx130x_field_write(priv, F_FORCE_HOST_FE_CTRL, 0);
	if (ret)
		return ret;
	ret = sx130x_field_write(priv, F_FORCE_DEC_FILTER_GAIN, 0);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, SX1301_CHRS, 0);
	if (ret) {
		dev_err(priv->dev, "radio select write failed\n");
		return ret;
	}

	ret = sx130x_field_write(priv, F_MCU_RST_0, 0);
	if (ret) {
		dev_err(priv->dev, "MCU 0 release failed\n");
		return ret;
	}

	ret = sx130x_field_force_write(priv, F_MCU_RST_1, 0);
	if (ret) {
		dev_err(priv->dev, "MCU 1 release failed\n");
		return ret;
	}

	ret = sx130x_agc_ram_read(priv, 0x20, &val);
	if (ret) {
		dev_err(priv->dev, "AGC RAM data read failed\n");
		return ret;
	}

	dev_info(priv->dev, "AGC firmware version %u\n", (unsigned)val);

	if (val != SX1301_MCU_AGC_FW_VERSION) {
		dev_err(priv->dev, "unexpected firmware version, expecting %u\n",
				SX1301_MCU_AGC_FW_VERSION);
		return -EIO;
	}

	ret = sx130x_arb_ram_read(priv, 0x20, &val);
	if (ret) {
		dev_err(priv->dev, "ARB RAM data read failed\n");
		return ret;
	}

	dev_info(priv->dev, "ARB firmware version %u\n", (unsigned)val);

	if (val != SX1301_MCU_ARB_FW_VERSION) {
		dev_err(priv->dev, "unexpected firmware version, expecting %u\n",
				SX1301_MCU_ARB_FW_VERSION);
		return -EIO;
	}

	return 0;
}

static int sx130x_load_tx_gain_lut(struct sx130x_priv *priv)
{
	struct net_device *netdev = dev_get_drvdata(priv->dev);
	struct sx130x_tx_gain_lut *lut = priv->tx_gain_lut;
	unsigned int status, val;
	int ret, i;

	for (i = 0; i < priv->tx_gain_lut_size; i++) {
		val = lut->pa_gain << SX1301_PA_GAIN_OFFSET;
		val |= lut->dac_gain << SX1301_DAC_GAIN_OFFSET;
		val |= lut->mix_gain;

		netdev_info(netdev, "AGC LUT entry %d dBm: 0x%02x\n", lut->power, val);
		ret = sx130x_agc_transaction(priv, val, &status);
		if (ret) {
			netdev_err(netdev, "AGC LUT load failed\n");
			return ret;
		}
		if (status != (SX1301_AGC_STATUS_SUCCESS | i)) {
			netdev_err(netdev, "AGC firmware LUT init error: 0x%02x", status);
			return -ENXIO;
		}
		lut++;
	}

	/* Abort the transaction if there are less then 16 entries */
	if (priv->tx_gain_lut_size < SX1301_TX_GAIN_LUT_MAX) {
		ret = sx130x_agc_transaction(priv, SX1301_AGC_CMD_ABORT, &status);
		if (ret) {
			netdev_err(netdev, "AGC LUT abort failed\n");
			return ret;
		}
		if (status != SX1301_AGC_STATUS_SUCCESS) {
			netdev_err(netdev, "AGC firmware LUT abort error: 0x%02x", status);
			return -ENXIO;
		}
	}

	return ret;
};

static int sx130x_agc_init(struct sx130x_priv *priv)
{
	struct net_device *netdev = dev_get_drvdata(priv->dev);
	unsigned int tx_msb;
	unsigned int val;
	int ret;

	ret = regmap_read(priv->regmap, SX1301_AGCSTS, &val);
	if (ret) {
		netdev_err(netdev, "AGC status read failed\n");
		return ret;
	}
	if (val != SX1301_AGC_STATUS_READY) {
		netdev_err(netdev, "AGC firmware init failure: 0x%02x\n", val);
		return -ENXIO;
	}

	ret = sx130x_load_tx_gain_lut(priv);
	if (ret)
		return ret;

	/*
	 * Load Tx freq MSBs
	 * Always 3 if f > 768 for SX1257 and SX1258 or f > 384 for SX1255
	 */
	tx_msb = 3; /* TODO detect radio type */

	ret = sx130x_agc_transaction(priv, tx_msb, &val);
	if (ret) {
		netdev_err(netdev, "AGC Tx MSBs load failed\n");
		return ret;
	}
	if (val != (SX1301_AGC_STATUS_SUCCESS | tx_msb)) {
		netdev_err(netdev, "AGC firmware Tx MSBs error: 0x%02x", val);
		return -ENXIO;
	}

	/* Load chan_select firmware option */
	ret = sx130x_agc_transaction(priv, 0, &val);
	if (ret) {
		netdev_err(netdev, "AGC chan select failed\n");
		return ret;
	}
	if (val != (SX1301_AGC_STATUS_SUCCESS | 0)) {
		netdev_err(netdev, "AGC firmware chan select error: 0x%02x", val);
		return -ENXIO;
	}

	/* End AGC firmware init and check status */
	/* TODO load the intended value of radio_select here
	 * LORA IF mapping to radio A/B (per bit, 0=A, 1=B) */
	ret = sx130x_agc_transaction(priv, 0x07, &val);
	if (ret) {
		netdev_err(netdev, "AGC radio select failed\n");
		return ret;
	}
	if (val != SX1301_AGC_STATUS_INITIALISED) {
		netdev_err(netdev, "AGC firmware init error: 0x%02x", val);
		return -ENXIO;
	}

	return ret;
}

static netdev_tx_t sx130x_loradev_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	if (skb->protocol != htons(ETH_P_LORA)) {
		kfree_skb(skb);
		netdev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	netif_stop_queue(netdev);

	/* TODO */
	return NETDEV_TX_OK;
}

static int sx130x_set_channel(struct sx130x_priv *priv, int chan, int offset)
{
	int ret;

	ret = regmap_write(priv->regmap, SX1301_IF0L + (chan * 2),
				IF_HZ_TO_REG(offset) & 0xFF);
	if (ret) {
		dev_err(priv->dev, "IF%dL failed\n", chan);
		return ret;
	}
	ret = regmap_write(priv->regmap, SX1301_IF0H + (chan * 2),
				(IF_HZ_TO_REG(offset) >> 8) & 0xFF);
	if (ret) {
		dev_err(priv->dev, "IF%dH failed\n", chan);
		return ret;
	}

	return ret;
}

static int sx130x_set_STD_channel(struct sx130x_priv *priv)
{
	int ret;

	ret = sx130x_set_channel(priv, 8, -200000);
	if (ret) {
		return ret;
	}

	ret = sx130x_field_force_write(priv, F_MBWSSF_MODEM_EN, 0);
	if (ret) {
		dev_err(priv->dev, "diable MBWSSF modem failed\n");
		return ret;
	}

	return ret;
}

static int sx130x_set_FSK_channel(struct sx130x_priv *priv)
{
	int ret;
	uint8_t sync_word_size = 3;
	uint64_t sync_word = 0xC194C1;
	uint64_t sync_word_reg;

	ret = sx130x_set_channel(priv, 9, 300000);
	if (ret) {
		return ret;
	}

	ret = sx130x_field_write(priv, F_FSK_PSIZE, sync_word_size-1);
	if (ret) {
		dev_err(priv->dev, "sync word size failed\n");
		return ret;
	}

	ret = sx130x_field_write(priv, F_FSK_TX_PSIZE, sync_word_size-1);
	if (ret) {
		dev_err(priv->dev, "sync word size failed\n");
		return ret;
	}

	sync_word_reg = sync_word << (8 * (8 - sync_word_size));
	ret = regmap_bulk_write(priv->regmap, SX1301_FSK_REF_PATTERN_LSB,
			        &sync_word_reg, 8);
	if (ret) {
		dev_err(priv->dev, "sync word size failed\n");
		return ret;
	}

	ret = sx130x_field_force_write(priv, F_FSK_MODEM_EN, 0);
	if (ret) {
		dev_err(priv->dev, "diable FSK modem failed\n");
		return ret;
	}

	return ret;
}

static int sx130x_loradev_open(struct net_device *netdev)
{
	struct sx130x_priv *priv = netdev_priv(netdev);
	unsigned int x;
	int ret;

	netdev_dbg(netdev, "%s", __func__);

	if (!sx130x_radio_devices_okay(priv->dev)) {
		netdev_err(netdev, "radio devices not yet bound to a driver\n");
		return -ENXIO;
	}

	mutex_lock(&priv->io_lock);

	ret = sx130x_field_write(priv, F_GLOBAL_EN, 1);
	if (ret) {
		dev_err(priv->dev, "enable global clocks failed\n");
		goto err_reg;
	}

	ret = sx130x_field_force_write(priv, F_CLK32M_EN, 1);
	if (ret) {
		dev_err(priv->dev, "enable 32M clock failed\n");
		goto err_reg;
	}

	/* calibration */

	ret = sx130x_agc_calibrate(priv);
	if (ret)
		goto err_calibrate;

	ret = sx130x_fields_patch(priv);
	if (ret)
		goto err_patch;

	/* TODO Frequency time drift */
	x = 4096000000 / (868500000 >> 1);
	/* dividend: (4*2048*1000000) >> 1, rescaled to avoid 32b overflow */
    	x = ( x > 63 ) ? 63 : x; /* saturation */
	ret = regmap_write(priv->regmap, SX1301_FREQ_TO_TIME_DRIFT, x);
	if (ret) {
		dev_err(priv->dev, "Freq to time drift failed\n");
		goto err_freq;
	}

	x = 4096000000 / (868500000 >> 3); /* dividend: (16*2048*1000000) >> 3, rescaled to avoid 32b overflow */
	x = ( x > 63 ) ? 63 : x; /* saturation */
	ret = regmap_write(priv->regmap, SX1301_MBWSSF_FREQ_TO_TIME_DRIFT, x);
	if (ret) {
		dev_err(priv->dev, "MBWSSF Freq to time drift failed\n");
		goto err_freq;
	}


	/* TODO Configure lora multi demods, bitfield of active */

	/* TODO Load concentrator multi channel frequencies */
	ret = sx130x_set_channel(priv, 0, -400000);
	if (ret) {
		goto err_freq;
	}

	ret = sx130x_set_channel(priv, 1, -200000);
	if (ret) {
		goto err_freq;
	}

	ret = sx130x_set_channel(priv, 2, 0);
	if (ret) {
		goto err_freq;
	}

	ret = sx130x_set_channel(priv, 3, -400000);
	if (ret) {
		goto err_freq;
	}

	ret = sx130x_set_channel(priv, 4, -200000);
	if (ret) {
		goto err_freq;
	}

	ret = sx130x_set_channel(priv, 5, 0);
	if (ret) {
		goto err_freq;
	}

	ret = sx130x_set_channel(priv, 6, 200000);
	if (ret) {
		goto err_freq;
	}

	ret = sx130x_set_channel(priv, 7, 400000);
	if (ret) {
		goto err_freq;
	}

	/* TODO enable the correlator on enabled frequencies */
	ret = sx130x_enable_correlators(priv);
	if (ret)
		goto err_freq;

	/* TODO PPM, and modem enable */

	/* TODO turn on IF8 */
	ret = sx130x_set_STD_channel(priv);
	if (ret) {
		goto err_freq;
	}

	/* TODO turn on FSK IF9 */
	ret = sx130x_set_FSK_channel(priv);
	if (ret) {
		goto err_freq;
	}

	ret = sx130x_load_all_firmware(priv);
	if (ret)
		goto err_firmware;

	usleep_range(1000, 2000);
	ret = sx130x_agc_init(priv);
	if (ret)
		goto err_firmware;

	ret = open_loradev(netdev);
	if (ret)
		goto err_open;

	mutex_unlock(&priv->io_lock);

	netif_start_queue(netdev);

	return 0;

err_open:
err_firmware:
err_freq:
err_patch:
err_calibrate:
err_reg:
	mutex_unlock(&priv->io_lock);
	return ret;
}

static int sx130x_loradev_stop(struct net_device *netdev)
{
	netdev_dbg(netdev, "%s", __func__);

	netif_stop_queue(netdev);
	close_loradev(netdev);

	return 0;
}

static const struct net_device_ops sx130x_net_device_ops = {
	.ndo_open = sx130x_loradev_open,
	.ndo_stop = sx130x_loradev_stop,
	.ndo_start_xmit = sx130x_loradev_start_xmit,
};

int sx130x_early_probe(struct regmap *regmap, struct gpio_desc *rst)
{
	struct device *dev = regmap_get_device(regmap);
	struct net_device *netdev;
	struct sx130x_priv *priv;
	u8 tmp[5 * SX1301_TX_GAIN_LUT_MAX];
	u8 *power_lut;
	int ret;
	int i;

	netdev = devm_alloc_loradev(dev, sizeof(*priv));
	if (!netdev)
		return -ENOMEM;

	netdev->netdev_ops = &sx130x_net_device_ops;
	SET_NETDEV_DEV(netdev, dev);

	priv = netdev_priv(netdev);
	priv->regmap = regmap;
	priv->rst_gpio = rst;

	mutex_init(&priv->io_lock);

	dev_set_drvdata(dev, netdev);
	priv->dev = dev;

	for (i = 0; i < ARRAY_SIZE(sx130x_regmap_fields); i++) {
		const struct reg_field *reg_fields = sx130x_regmap_fields;

		priv->regmap_fields[i] = devm_regmap_field_alloc(dev,
				priv->regmap,
				reg_fields[i]);
		if (IS_ERR(priv->regmap_fields[i])) {
			ret = PTR_ERR(priv->regmap_fields[i]);
			dev_err(dev, "Cannot allocate regmap field (%d)\n", ret);
			return ret;
		}
	}

	if (IS_ENABLED(CONFIG_OF)) {
		ret = of_property_read_variable_u8_array(dev->of_node, "semtech,power-lut",
							 tmp, 5, 5 * SX1301_TX_GAIN_LUT_MAX);
		if (ret < 0) {
			dev_err(dev, "No power table found (%d)\n", ret);
			return -EINVAL;
		}

		if (ret % 5) {
			dev_err(dev, "Invalid power table\n");
			return -EINVAL;
		} else {
			priv->tx_gain_lut_size = ret / 5;
			power_lut = tmp;
			for (i = 0; i < priv->tx_gain_lut_size; i++) {
				priv->tx_gain_lut[i].power = *(power_lut++);
				priv->tx_gain_lut[i].dig_gain = *(power_lut++);
				priv->tx_gain_lut[i].pa_gain = *(power_lut++) & 0x03;
				priv->tx_gain_lut[i].dac_gain = *(power_lut++) & 0x03;
				priv->tx_gain_lut[i].mix_gain = *(power_lut++) & 0x0F;
			}
		}

		for (i = 0; i < SX1301_NUM_GPIOS; i++) {
			priv->gpio[i] = devm_gpiod_get_index_optional(dev, "gpio", i, GPIOD_IN);
			if (priv->gpio[i] == NULL)
				dev_dbg(dev, "GPIO%d not available, ignoring", i);
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(sx130x_early_probe);

int sx130x_probe(struct device *dev)
{
	struct net_device *netdev = dev_get_drvdata(dev);
	struct sx130x_priv *priv = netdev_priv(netdev);
	unsigned int ver;
	unsigned int val;
	int ret, i;

	ret = regmap_read(priv->regmap, SX1301_VER, &ver);
	if (ret) {
		dev_err(dev, "version read failed (%d)\n", ret);
		return ret;
	}

	if (ver != SX1301_CHIP_VERSION) {
		dev_err(dev, "unexpected version: %u\n", ver);
		return -EIO;
	}

	ret = regmap_write(priv->regmap, SX1301_PAGE, 0);
	if (ret) {
		dev_err(dev, "page/reset write failed (%d)\n", ret);
		return ret;
	}

	ret = sx130x_soft_reset(priv);
	if (ret) {
		dev_err(dev, "soft reset failed (%d)\n", ret);
		return ret;
	}

	ret = sx130x_field_write(priv, F_GLOBAL_EN, 0);
	if (ret) {
		dev_err(dev, "gate global clocks failed (%d)\n", ret);
		return ret;
	}

	ret = sx130x_field_write(priv, F_CLK32M_EN, 0);
	if (ret) {
		dev_err(dev, "gate 32M clock failed (%d)\n", ret);
		return ret;
	}

	ret = sx130x_field_write(priv, F_RADIO_A_EN, 1);
	if (ret) {
		dev_err(dev, "radio A enable failed (%d)\n", ret);
		return ret;
	}

	ret = sx130x_field_force_write(priv, F_RADIO_B_EN, 1);
	if (ret) {
		dev_err(dev, "radio B enable failed (%d)\n", ret);
		return ret;
	}

	msleep(500);

	ret = sx130x_field_force_write(priv, F_RADIO_RST, 1);
	if (ret) {
		dev_err(dev, "radio assert reset failed (%d)\n", ret);
		return ret;
	}

	msleep(5);

	ret = sx130x_field_force_write(priv, F_RADIO_RST, 0);
	if (ret) {
		dev_err(dev, "radio deassert reset failed (%d)\n", ret);
		return ret;
	}

	/* radio */

	ret = devm_sx130x_register_radio_devices(dev);
	if (ret)
		return ret;

	mutex_lock(&priv->io_lock);

	/* GPIO */

	ret = regmap_read(priv->regmap, SX1301_GPMODE, &val);
	if (ret) {
		dev_err(dev, "GPIO mode read failed (%d)\n", ret);
		goto out;
	}

	val |= GENMASK(4, 0);

	ret = regmap_write(priv->regmap, SX1301_GPMODE, val);
	if (ret) {
		dev_err(dev, "GPIO mode write failed (%d)\n", ret);
		goto out;
	}

	ret = regmap_read(priv->regmap, SX1301_GPSO, &val);
	if (ret) {
		dev_err(dev, "GPIO select output read failed (%d)\n", ret);
		goto out;
	}

	/* If we have any GPIO connected leave in configuration 0 */
	val &= ~GENMASK(3, 0);
	for (i = 0; i < SX1301_NUM_GPIOS; i++) {
		if (priv->gpio[i])
			break;
	}
	if (i == SX1301_NUM_GPIOS)
		val |= 2;

	ret = regmap_write(priv->regmap, SX1301_GPSO, val);
	if (ret) {
		dev_err(dev, "GPIO select output write failed (%d)\n", ret);
		goto out;
	}

	/* TODO LBT */

	ret = register_loradev(netdev);
	if (ret)
		goto out;

	dev_info(dev, "SX1301 module probed\n");

out:
	mutex_unlock(&priv->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(sx130x_probe);

int sx130x_remove(struct device *dev)
{
	struct net_device *netdev = dev_get_drvdata(dev);

	unregister_loradev(netdev);

	dev_info(dev, "SX1301 module removed\n");

	return 0;
}
EXPORT_SYMBOL_GPL(sx130x_remove);

static int sx130x_spi_probe(struct spi_device *spi)
{
	struct gpio_desc *rst;
	struct regmap *regmap;
	int ret;

	rst = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(rst)) {
		if (PTR_ERR(rst) != -EPROBE_DEFER)
			dev_err(&spi->dev, "Failed to obtain reset GPIO\n");
		return PTR_ERR(rst);
	}

	gpiod_set_value_cansleep(rst, 1);
	msleep(100);
	gpiod_set_value_cansleep(rst, 0);
	msleep(100);

	spi->bits_per_word = 8;
	spi_setup(spi);

	regmap = devm_regmap_init_spi(spi, &sx130x_regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(&spi->dev, "Regmap allocation failed: %d\n", ret);
		return ret;
	}

	ret = sx130x_early_probe(regmap, rst);
	if (ret)
		return ret;

	return sx130x_probe(&spi->dev);
}

static int sx130x_spi_remove(struct spi_device *spi)
{
	return sx130x_remove(&spi->dev);;
}

#ifdef CONFIG_OF
static const struct of_device_id sx130x_dt_ids[] = {
	{ .compatible = "semtech,sx1301" },
	{}
};
MODULE_DEVICE_TABLE(of, sx130x_dt_ids);
#endif

static struct spi_driver sx130x_spi_driver = {
	.driver = {
		.name = "sx130x",
		.of_match_table = of_match_ptr(sx130x_dt_ids),
	},
	.probe = sx130x_spi_probe,
	.remove = sx130x_spi_remove,
};

static int __init sx130x_init(void)
{
	int ret;

	ret = sx130x_radio_init();
	if (ret)
		return ret;

	return spi_register_driver(&sx130x_spi_driver);
}
module_init(sx130x_init);

static void __exit sx130x_exit(void)
{
	spi_unregister_driver(&sx130x_spi_driver);
	sx130x_radio_exit();
}
module_exit(sx130x_exit);

MODULE_DESCRIPTION("SX130x SPI driver");
MODULE_AUTHOR("Andreas Färber <afaerber@suse.de>");
MODULE_AUTHOR("Ben Whitten <ben.whitten@gmail.com>");
MODULE_LICENSE("GPL");
