// SPDX-License-Identifier: GPL-2.0-or-later
/* Semtech SX1301 LoRa concentrator
 *
 * Copyright (c) 2018 Andreas Färber
 * Copyright (c) 2018 Ben Whitten
 *
 * Based on SX1301 HAL code:
 * Copyright (c) 2013 Semtech-Cycleo
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/lora.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/lora/dev.h>
#include <linux/spi/spi.h>

#include "sx1301.h"

static struct sx1301_tx_gain_lut tx_gain_lut[] = {
	{
		.dig_gain = 0,
		.pa_gain = 0,
		.dac_gain = 3,
		.mix_gain = 8,
		.rf_power = -3,
	},
	{
		.dig_gain = 0,
		.pa_gain = 0,
		.dac_gain = 3,
		.mix_gain = 9,
		.rf_power = 0,
	},
	{
		.dig_gain = 0,
		.pa_gain = 0,
		.dac_gain = 3,
		.mix_gain = 12,
		.rf_power = 3,
	},
	{
		.dig_gain = 0,
		.pa_gain = 0,
		.dac_gain = 3,
		.mix_gain = 13,
		.rf_power = 4,
	},
	{
		.dig_gain = 0,
		.pa_gain = 1,
		.dac_gain = 3,
		.mix_gain = 8,
		.rf_power = 6,
	},
	{
		.dig_gain = 0,
		.pa_gain = 1,
		.dac_gain = 3,
		.mix_gain = 9,
		.rf_power = 9,
	},
	{
		.dig_gain = 0,
		.pa_gain = 1,
		.dac_gain = 3,
		.mix_gain = 10,
		.rf_power = 10,
	},
	{
		.dig_gain = 0,
		.pa_gain = 1,
		.dac_gain = 3,
		.mix_gain = 11,
		.rf_power = 12,
	},
	{
		.dig_gain = 0,
		.pa_gain = 1,
		.dac_gain = 3,
		.mix_gain = 12,
		.rf_power = 13,
	},
	{
		.dig_gain = 0,
		.pa_gain = 1,
		.dac_gain = 3,
		.mix_gain = 13,
		.rf_power = 14,
	},
	{
		.dig_gain = 0,
		.pa_gain = 1,
		.dac_gain = 3,
		.mix_gain = 15,
		.rf_power = 16,
	},
	{
		.dig_gain = 0,
		.pa_gain = 2,
		.dac_gain = 3,
		.mix_gain = 10,
		.rf_power = 19,
	},
	{
		.dig_gain = 0,
		.pa_gain = 2,
		.dac_gain = 3,
		.mix_gain = 11,
		.rf_power = 21,
	},
	{
		.dig_gain = 0,
		.pa_gain = 2,
		.dac_gain = 3,
		.mix_gain = 12,
		.rf_power = 22,
	},
	{
		.dig_gain = 0,
		.pa_gain = 2,
		.dac_gain = 3,
		.mix_gain = 13,
		.rf_power = 24,
	},
	{
		.dig_gain = 0,
		.pa_gain = 2,
		.dac_gain = 3,
		.mix_gain = 14,
		.rf_power = 25,
	},
};

static const struct regmap_range_cfg sx1301_regmap_ranges[] = {
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

	.ranges = sx1301_regmap_ranges,
	.num_ranges = ARRAY_SIZE(sx1301_regmap_ranges),
	.max_register = SX1301_MAX_REGISTER,
};

static int sx1301_field_write(struct sx1301_priv *priv,
			      enum sx1301_fields field_id, u8 val)
{
	return regmap_field_write(priv->regmap_fields[field_id], val);
}

static int sx1301_agc_ram_read(struct sx1301_priv *priv, u8 addr,
			       unsigned int *val)
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

static int sx1301_arb_ram_read(struct sx1301_priv *priv, u8 addr,
			       unsigned int *val)
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

static int sx1301_load_firmware(struct sx1301_priv *priv, int mcu,
				const struct firmware *fw)
{
	u8 *buf;
	enum sx1301_fields rst, select_mux;
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

	ret = sx1301_field_write(priv, rst, 1);
	if (ret) {
		dev_err(priv->dev, "MCU reset failed\n");
		return ret;
	}

	ret = sx1301_field_write(priv, select_mux, 0);
	if (ret) {
		dev_err(priv->dev, "MCU RAM select mux failed\n");
		return ret;
	}

	ret = regmap_write(priv->regmap, SX1301_MPA, 0);
	if (ret) {
		dev_err(priv->dev, "MCU prom addr write failed\n");
		return ret;
	}

	ret = regmap_noinc_write(priv->regmap, SX1301_MPD, fw->data, fw->size);
	if (ret) {
		dev_err(priv->dev, "MCU prom data write failed\n");
		return ret;
	}

	ret = regmap_read(priv->regmap, SX1301_MPD, &val);
	if (ret) {
		dev_err(priv->dev, "MCU prom data dummy read failed\n");
		return ret;
	}

	buf = kzalloc(fw->size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = regmap_noinc_read(priv->regmap, SX1301_MPD, buf, fw->size);
	if (ret) {
		dev_err(priv->dev, "MCU prom data read failed\n");
		kfree(buf);
		return ret;
	}

	if (memcmp(fw->data, buf, fw->size)) {
		dev_err(priv->dev,
			"MCU prom data read does not match data written\n");
		kfree(buf);
		return -ENXIO;
	}

	kfree(buf);

	ret = sx1301_field_write(priv, select_mux, 1);
	if (ret) {
		dev_err(priv->dev, "MCU RAM release mux failed\n");
		return ret;
	}

	return 0;
}

static int sx1301_agc_transaction(struct sx1301_priv *priv, unsigned int val,
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

static int sx1301_agc_calibrate(struct sx1301_priv *priv)
{
	const struct firmware *fw;
	unsigned int val;
	int ret;

	ret = request_firmware(&fw, "sx1301_agc_calibration.bin", priv->dev);
	if (ret) {
		dev_err(priv->dev, "agc cal firmware file load failed\n");
		return ret;
	}

	ret = sx1301_load_firmware(priv, 1, fw);
	release_firmware(fw);
	if (ret) {
		dev_err(priv->dev, "agc cal firmware load failed\n");
		return ret;
	}

	ret = sx1301_field_write(priv, F_FORCE_HOST_RADIO_CTRL, 0);
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

	ret = sx1301_field_write(priv, F_MCU_RST_1, 0);
	if (ret) {
		dev_err(priv->dev, "MCU 1 reset failed\n");
		return ret;
	}

	ret = sx1301_agc_ram_read(priv, 0x20, &val);
	if (ret) {
		dev_err(priv->dev, "AGC RAM data read failed\n");
		return ret;
	}

	dev_info(priv->dev, "AGC calibration firmware version %u\n", val);

	if (val != SX1301_MCU_AGC_CAL_FW_VERSION) {
		dev_err(priv->dev,
			"unexpected firmware version, expecting %u\n",
			SX1301_MCU_AGC_CAL_FW_VERSION);
		return -ENXIO;
	}

	ret = sx1301_field_write(priv, F_EMERGENCY_FORCE_HOST_CTRL, 0);
	if (ret) {
		dev_err(priv->dev, "emergency force failed\n");
		return ret;
	}

	dev_err(priv->dev, "starting calibration...\n");
	msleep(2300);

	ret = sx1301_field_write(priv, F_EMERGENCY_FORCE_HOST_CTRL, 1);
	if (ret) {
		dev_err(priv->dev, "emergency force release failed\n");
		return ret;
	}

	ret = regmap_read(priv->regmap, SX1301_AGCSTS, &val);
	if (ret) {
		dev_err(priv->dev, "AGC status read failed\n");
		return ret;
	}

	dev_info(priv->dev, "AGC status: %02x\n", val);
	if ((val & (BIT(7) | BIT(0))) != (BIT(7) | BIT(0))) {
		dev_err(priv->dev, "AGC calibration failed\n");
		return -ENXIO;
	}

	return 0;
}

static int sx1301_load_all_firmware(struct sx1301_priv *priv)
{
	const struct firmware *fw;
	unsigned int val;
	int ret;

	ret = request_firmware(&fw, "sx1301_arb.bin", priv->dev);
	if (ret) {
		dev_err(priv->dev, "arb firmware file load failed\n");
		return ret;
	}

	ret = sx1301_load_firmware(priv, 0, fw);
	release_firmware(fw);
	if (ret)
		return ret;

	ret = request_firmware(&fw, "sx1301_agc.bin", priv->dev);
	if (ret) {
		dev_err(priv->dev, "agc firmware file load failed\n");
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
	if (ret) {
		dev_err(priv->dev, "radio select write failed\n");
		return ret;
	}

	ret = sx1301_field_write(priv, F_MCU_RST_0, 0);
	if (ret) {
		dev_err(priv->dev, "MCU 0 release failed\n");
		return ret;
	}

	ret = sx1301_field_write(priv, F_MCU_RST_1, 0);
	if (ret) {
		dev_err(priv->dev, "MCU 1 release failed\n");
		return ret;
	}

	ret = sx1301_agc_ram_read(priv, 0x20, &val);
	if (ret) {
		dev_err(priv->dev, "AGC RAM data read failed\n");
		return ret;
	}

	dev_info(priv->dev, "AGC firmware version %u\n", val);

	if (val != SX1301_MCU_AGC_FW_VERSION) {
		dev_err(priv->dev,
			"unexpected firmware version, expecting %u\n",
			SX1301_MCU_AGC_FW_VERSION);
		return -ENXIO;
	}

	ret = sx1301_arb_ram_read(priv, 0x20, &val);
	if (ret) {
		dev_err(priv->dev, "ARB RAM data read failed\n");
		return ret;
	}

	dev_info(priv->dev, "ARB firmware version %u\n", val);

	if (val != SX1301_MCU_ARB_FW_VERSION) {
		dev_err(priv->dev,
			"unexpected firmware version, expecting %u\n",
			SX1301_MCU_ARB_FW_VERSION);
		return -ENXIO;
	}

	return ret;
}

static int sx1301_load_tx_gain_lut(struct sx1301_priv *priv)
{
	struct sx1301_tx_gain_lut *lut = priv->tx_gain_lut;
	unsigned int val, status;
	int ret, i;

	/* HACK use internal gain table in the short term */
	lut = tx_gain_lut;
	priv->tx_gain_lut_size = ARRAY_SIZE(tx_gain_lut);

	for (i = 0; i < priv->tx_gain_lut_size; i++) {
		val = lut->mix_gain + (lut->dac_gain << 4) +
			(lut->pa_gain << 6);
		ret = sx1301_agc_transaction(priv, val, &status);
		if (ret) {
			dev_err(priv->dev, "AGC LUT load failed\n");
			return ret;
		}
		if (val != (0x30 + i)) {
			dev_err(priv->dev,
				"AGC firmware LUT init error: 0x%02X", val);
			return -ENXIO;
		}
		lut++;
	}

	/* Abort the transaction if there are less then 16 entries */
	if (priv->tx_gain_lut_size < SX1301_TX_GAIN_LUT_MAX) {
		ret = sx1301_agc_transaction(priv, SX1301_AGC_CMD_ABORT, &val);
		if (ret) {
			dev_err(priv->dev, "AGC LUT abort failed\n");
			return ret;
		}
		if (val != 0x30) {
			dev_err(priv->dev,
				"AGC firmware LUT abort error: 0x%02X", val);
			return -ENXIO;
		}
	}

	return ret;
};

static netdev_tx_t sx130x_loradev_start_xmit(struct sk_buff *skb,
					     struct net_device *netdev)
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

static int sx130x_loradev_open(struct net_device *netdev)
{
	struct sx1301_priv *priv = netdev_priv(netdev);
	int ret;
	unsigned int val;

	netdev_dbg(netdev, "%s", __func__);

	if (!sx130x_radio_devices_okay(priv->dev)) {
		netdev_err(netdev, "radio devices not yet bound to a driver\n");
		return -ENXIO;
	}

	priv->clk32m = devm_clk_get(priv->dev, "clk32m");
	if (IS_ERR(priv->clk32m)) {
		dev_err(priv->dev, "failed to get clk32m\n");
		return PTR_ERR(priv->clk32m);
	}

	ret = clk_prepare_enable(priv->clk32m);
	if (ret) {
		dev_err(priv->dev, "failed to enable clk32m: %d\n", ret);
		return ret;
	}

	ret = sx1301_field_write(priv, F_GLOBAL_EN, 1);
	if (ret) {
		dev_err(priv->dev, "enable global clocks failed\n");
		return ret;
	}

	ret = sx1301_field_write(priv, F_CLK32M_EN, 1);
	if (ret) {
		dev_err(priv->dev, "enable 32M clock failed\n");
		return ret;
	}

	/* calibration */

	ret = sx1301_agc_calibrate(priv);
	if (ret)
		return ret;

	/* TODO Load constant adjustments, patches */

	/* TODO Frequency time drift */

	/* TODO Configure lora multi demods, bitfield of active */

	/* TODO Load concenrator multi channel frequencies */

	/* TODO enale to correlator on enabled frequenies */

	/* TODO PPMi, and modem enable */

	ret = sx1301_load_all_firmware(priv);
	if (ret)
		return ret;

	ret = sx1301_load_tx_gain_lut(priv);
	if (ret)
		return ret;

	/* Load Tx freq MSBs
	 * Always 3 if f > 768 for SX1257 or f > 384 for SX1255
	 */
	ret = sx1301_agc_transaction(priv, 3, &val);
	if (ret) {
		dev_err(priv->dev, "AGC Tx MSBs load failed\n");
		return ret;
	}
	if (val != 0x33) {
		dev_err(priv->dev, "AGC firmware Tx MSBs error: 0x%02X", val);
		return -ENXIO;
	}

	/* Load chan_select firmware option */
	ret = sx1301_agc_transaction(priv, 0, &val);
	if (ret) {
		dev_err(priv->dev, "AGC chan select failed\n");
		return ret;
	}
	if (val != 0x30) {
		dev_err(priv->dev,
			"AGC firmware chan select error: 0x%02X", val);
		return -ENXIO;
	}

	/* End AGC firmware init and check status */
	ret = sx1301_agc_transaction(priv, 0, &val);
	if (ret) {
		dev_err(priv->dev, "AGC radio select failed\n");
		return ret;
	}
	if (val != 0x40) {
		dev_err(priv->dev, "AGC firmware init error: 0x%02X", val);
		return -ENXIO;
	}

	ret = open_loradev(netdev);
	if (ret)
		return ret;

	netif_start_queue(netdev);

	return 0;
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

static int sx1301_probe(struct spi_device *spi)
{
	struct net_device *netdev;
	struct sx1301_priv *priv;
	struct gpio_desc *rst;
	int ret;
	int i;
	unsigned int ver;
	unsigned int val;

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

	netdev = devm_alloc_loradev(&spi->dev, sizeof(*priv));
	if (!netdev)
		return -ENOMEM;

	netdev->netdev_ops = &sx130x_net_device_ops;
	SET_NETDEV_DEV(netdev, &spi->dev);

	priv = netdev_priv(netdev);
	priv->rst_gpio = rst;

	spi_set_drvdata(spi, netdev);
	priv->dev = &spi->dev;

	priv->regmap = devm_regmap_init_spi(spi, &sx1301_regmap_config);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err(&spi->dev, "Regmap allocation failed: %d\n", ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(sx1301_regmap_fields); i++) {
		const struct reg_field *reg_fields = sx1301_regmap_fields;

		priv->regmap_fields[i] = devm_regmap_field_alloc(&spi->dev,
								 priv->regmap,
								 reg_fields[i]);
		if (IS_ERR(priv->regmap_fields[i])) {
			ret = PTR_ERR(priv->regmap_fields[i]);
			dev_err(&spi->dev,
				"Cannot allocate regmap field: %d\n", ret);
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

	ret = sx1301_field_write(priv, F_GLOBAL_EN, 0);
	if (ret) {
		dev_err(&spi->dev, "gate global clocks failed\n");
		return ret;
	}

	ret = sx1301_field_write(priv, F_CLK32M_EN, 0);
	if (ret) {
		dev_err(&spi->dev, "gate 32M clock failed\n");
		return ret;
	}

	ret = sx1301_field_write(priv, F_RADIO_A_EN, 1);
	if (ret) {
		dev_err(&spi->dev, "radio a enable failed\n");
		return ret;
	}

	ret = sx1301_field_write(priv, F_RADIO_B_EN, 1);
	if (ret) {
		dev_err(&spi->dev, "radio b enable failed\n");
		return ret;
	}

	msleep(500);

	ret = sx1301_field_write(priv, F_RADIO_RST, 1);
	if (ret) {
		dev_err(&spi->dev, "radio asert reset failed\n");
		return ret;
	}

	usleep_range(5000, 6000);

	ret = sx1301_field_write(priv, F_RADIO_RST, 0);
	if (ret) {
		dev_err(&spi->dev, "radio deasert reset failed\n");
		return ret;
	}

	/* radio */

	ret = devm_sx130x_register_radio_devices(&spi->dev);
	if (ret)
		return ret;

	/* GPIO */

	ret = regmap_read(priv->regmap, SX1301_GPMODE, &val);
	if (ret) {
		dev_err(&spi->dev, "GPIO mode read failed\n");
		return ret;
	}

	val |= GENMASK(4, 0);

	ret = regmap_write(priv->regmap, SX1301_GPMODE, val);
	if (ret) {
		dev_err(&spi->dev, "GPIO mode write failed\n");
		return ret;
	}

	ret = regmap_read(priv->regmap, SX1301_GPSO, &val);
	if (ret) {
		dev_err(&spi->dev, "GPIO select output read failed\n");
		return ret;
	}

	val &= ~GENMASK(3, 0);
	val |= 2;

	ret = regmap_write(priv->regmap, SX1301_GPSO, val);
	if (ret) {
		dev_err(&spi->dev, "GPIO select output write failed\n");
		return ret;
	}

	/* TODO LBT */

	ret = register_loradev(netdev);
	if (ret)
		return ret;

	dev_info(&spi->dev, "SX1301 module probed\n");

	return 0;
}

static int sx1301_remove(struct spi_device *spi)
{
	struct net_device *netdev = spi_get_drvdata(spi);

	unregister_loradev(netdev);

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

static struct spi_driver sx130x_spi_driver = {
	.driver = {
		.name = "sx1301",
		.of_match_table = of_match_ptr(sx1301_dt_ids),
	},
	.probe = sx1301_probe,
	.remove = sx1301_remove,
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

MODULE_DESCRIPTION("SX1301 SPI driver");
MODULE_AUTHOR("Andreas Färber <afaerber@suse.de>");
MODULE_AUTHOR("Ben Whitten <ben.whitten@gmail.com>");
MODULE_LICENSE("GPL");
