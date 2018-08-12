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
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/lora/dev.h>
#include <linux/spi/spi.h>

#include "sx1301.h"

#define REG_PAGE_RESET_SOFT_RESET	BIT(7)

#define REG_16_GLOBAL_EN		BIT(3)

#define REG_17_CLK32M_EN		BIT(0)

#define REG_0_105_FORCE_HOST_RADIO_CTRL		BIT(1)
#define REG_0_105_FORCE_HOST_FE_CTRL		BIT(2)
#define REG_0_105_FORCE_DEC_FILTER_GAIN		BIT(3)

#define REG_0_MCU_RST_0			BIT(0)
#define REG_0_MCU_RST_1			BIT(1)
#define REG_0_MCU_SELECT_MUX_0		BIT(2)
#define REG_0_MCU_SELECT_MUX_1		BIT(3)

#define REG_2_43_RADIO_A_EN		BIT(0)
#define REG_2_43_RADIO_B_EN		BIT(1)
#define REG_2_43_RADIO_RST		BIT(2)

#define REG_EMERGENCY_FORCE_HOST_CTRL	BIT(0)

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

struct spi_sx1301 {
	struct spi_device *parent;
	unsigned int regs;
};

struct sx1301_priv {
	struct lora_dev_priv lora;
	struct device		*dev;
	struct spi_device	*spi;
	struct gpio_desc *rst_gpio;
	struct spi_controller *radio_a_ctrl, *radio_b_ctrl;
	struct regmap		*regmap;
};

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

static int sx1301_soft_reset(struct sx1301_priv *priv)
{
	return regmap_write(priv->regmap, SX1301_PAGE, REG_PAGE_RESET_SOFT_RESET);
}

#define REG_RADIO_X_DATA		0
#define REG_RADIO_X_DATA_READBACK	1
#define REG_RADIO_X_ADDR		2
#define REG_RADIO_X_CS			4

static int sx1301_radio_set_cs(struct spi_controller *ctrl, bool enable)
{
	struct spi_sx1301 *ssx = spi_controller_get_devdata(ctrl);
	struct net_device *netdev = spi_get_drvdata(ssx->parent);
	struct sx1301_priv *priv = netdev_priv(netdev);
	unsigned int cs;
	int ret;

	dev_dbg(&ctrl->dev, "setting CS to %s\n", enable ? "1" : "0");

	ret = regmap_read(priv->regmap, ssx->regs + REG_RADIO_X_CS, &cs);
	if (ret) {
		dev_warn(&ctrl->dev, "failed to read CS (%d)\n", ret);
		cs = 0;
	}

	if (enable)
		cs |= BIT(0);
	else
		cs &= ~BIT(0);

	ret = regmap_write(priv->regmap, ssx->regs + REG_RADIO_X_CS, cs);
	if (ret) {
		dev_err(&ctrl->dev, "failed to write CS (%d)\n", ret);
		return ret;
	}

	return 0;
}

static void sx1301_radio_spi_set_cs(struct spi_device *spi, bool enable)
{
	dev_dbg(&spi->dev, "setting SPI CS to %s\n", enable ? "1" : "0");

	if (enable)
		return;

	sx1301_radio_set_cs(spi->controller, enable);
}

static int sx1301_radio_spi_transfer_one(struct spi_controller *ctrl,
	struct spi_device *spi, struct spi_transfer *xfr)
{
	struct spi_sx1301 *ssx = spi_controller_get_devdata(ctrl);
	struct net_device *netdev = spi_get_drvdata(ssx->parent);
	struct sx1301_priv *priv = netdev_priv(netdev);
	const u8 *tx_buf = xfr->tx_buf;
	u8 *rx_buf = xfr->rx_buf;
	unsigned int val;
	int ret;

	if (xfr->len == 0 || xfr->len > 3)
		return -EINVAL;

	dev_dbg(&spi->dev, "transferring one (%u)\n", xfr->len);

	if (tx_buf) {
		ret = regmap_write(priv->regmap, ssx->regs + REG_RADIO_X_ADDR, tx_buf ? tx_buf[0] : 0);
		if (ret) {
			dev_err(&spi->dev, "SPI radio address write failed\n");
			return ret;
		}

		ret = regmap_write(priv->regmap, ssx->regs + REG_RADIO_X_DATA, (tx_buf && xfr->len >= 2) ? tx_buf[1] : 0);
		if (ret) {
			dev_err(&spi->dev, "SPI radio data write failed\n");
			return ret;
		}

		ret = sx1301_radio_set_cs(ctrl, true);
		if (ret) {
			dev_err(&spi->dev, "SPI radio CS set failed\n");
			return ret;
		}

		ret = sx1301_radio_set_cs(ctrl, false);
		if (ret) {
			dev_err(&spi->dev, "SPI radio CS unset failed\n");
			return ret;
		}
	}

	if (rx_buf) {
		ret = regmap_read(priv->regmap, ssx->regs + REG_RADIO_X_DATA_READBACK, &val);
		if (ret) {
			dev_err(&spi->dev, "SPI radio data read failed\n");
			return ret;
		}
		rx_buf[xfr->len - 1] = val & 0xff;
	}

	return 0;
}

static int sx1301_agc_ram_read(struct sx1301_priv *priv, u8 addr, unsigned int *val)
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

static int sx1301_arb_ram_read(struct sx1301_priv *priv, u8 addr, unsigned int *val)
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

static int sx1301_load_firmware(struct sx1301_priv *priv, int mcu, const struct firmware *fw)
{
	u8 *buf;
	u8 rst, select_mux;
	unsigned int val;
	int ret;

	if (fw->size != SX1301_MCU_FW_BYTE) {
		dev_err(priv->dev, "Unexpected firmware size\n");
		return -EINVAL;
	}

	switch (mcu) {
	case 0:
		rst = REG_0_MCU_RST_0;
		select_mux = REG_0_MCU_SELECT_MUX_0;
		break;
	case 1:
		rst = REG_0_MCU_RST_1;
		select_mux = REG_0_MCU_SELECT_MUX_1;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_read(priv->regmap, SX1301_MCU_CTRL, &val);
	if (ret) {
		dev_err(priv->dev, "MCU read failed\n");
		return ret;
	}

	val |= rst;
	val &= ~select_mux;

	ret = regmap_write(priv->regmap, SX1301_MCU_CTRL, val);
	if (ret) {
		dev_err(priv->dev, "MCU reset / select mux write failed\n");
		return ret;
	}

	ret = regmap_write(priv->regmap, SX1301_MPA, 0);
	if (ret) {
		dev_err(priv->dev, "MCU prom addr write failed\n");
		return ret;
	}

	ret = sx1301_write_burst(priv, SX1301_MPD, fw->data, fw->size);
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

	ret = sx1301_read_burst(priv, SX1301_MPD, buf, fw->size);
	if (ret) {
		dev_err(priv->dev, "MCU prom data read failed\n");
		kfree(buf);
		return ret;
	}

	if (memcmp(fw->data, buf, fw->size)) {
		dev_err(priv->dev, "MCU prom data read does not match data written\n");
		kfree(buf);
		return -ENXIO;
	}

	kfree(buf);

	ret = regmap_read(priv->regmap, SX1301_MCU_CTRL, &val);
	if (ret) {
		dev_err(priv->dev, "MCU read (1) failed\n");
		return ret;
	}

	val |= select_mux;

	ret = regmap_write(priv->regmap, SX1301_MCU_CTRL, val);
	if (ret) {
		dev_err(priv->dev, "MCU reset / select mux write (1) failed\n");
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

	ret = regmap_read(priv->regmap, SX1301_FORCE_CTRL, &val);
	if (ret) {
		dev_err(priv->dev, "0|105 read failed\n");
		return ret;
	}

	val &= ~REG_0_105_FORCE_HOST_RADIO_CTRL;

	ret = regmap_write(priv->regmap, SX1301_FORCE_CTRL, val);
	if (ret) {
		dev_err(priv->dev, "0|105 write failed\n");
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

	ret = regmap_read(priv->regmap, SX1301_MCU_CTRL, &val);
	if (ret) {
		dev_err(priv->dev, "MCU read (0) failed\n");
		return ret;
	}

	val &= ~REG_0_MCU_RST_1;

	ret = regmap_write(priv->regmap, SX1301_MCU_CTRL, val);
	if (ret) {
		dev_err(priv->dev, "MCU write (0) failed\n");
		return ret;
	}

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

	ret = regmap_read(priv->regmap, SX1301_EMERGENCY_FORCE_HOST_CTRL, &val);
	if (ret) {
		dev_err(priv->dev, "emergency force read failed\n");
		return ret;
	}

	val &= ~REG_EMERGENCY_FORCE_HOST_CTRL;

	ret = regmap_write(priv->regmap, SX1301_EMERGENCY_FORCE_HOST_CTRL, val);
	if (ret) {
		dev_err(priv->dev, "emergency force write failed\n");
		return ret;
	}

	dev_err(priv->dev, "starting calibration...\n");
	msleep(2300);

	ret = regmap_read(priv->regmap, SX1301_EMERGENCY_FORCE_HOST_CTRL, &val);
	if (ret) {
		dev_err(priv->dev, "emergency force read (1) failed\n");
		return ret;
	}

	val |= REG_EMERGENCY_FORCE_HOST_CTRL;

	ret = regmap_write(priv->regmap, SX1301_EMERGENCY_FORCE_HOST_CTRL, val);
	if (ret) {
		dev_err(priv->dev, "emergency force write (1) failed\n");
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

	ret = regmap_read(priv->regmap, SX1301_FORCE_CTRL, &val);
	if (ret) {
		dev_err(priv->dev, "0|105 read failed\n");
		return ret;
	}

	val &= ~(REG_0_105_FORCE_HOST_RADIO_CTRL | REG_0_105_FORCE_HOST_FE_CTRL | REG_0_105_FORCE_DEC_FILTER_GAIN);

	ret = regmap_write(priv->regmap, SX1301_FORCE_CTRL, val);
	if (ret) {
		dev_err(priv->dev, "0|105 write failed\n");
		return ret;
	}

	ret = regmap_write(priv->regmap, SX1301_CHRS, 0);
	if (ret) {
		dev_err(priv->dev, "radio select write failed\n");
		return ret;
	}

	ret = regmap_read(priv->regmap, SX1301_MCU_CTRL, &val);
	if (ret) {
		dev_err(priv->dev, "MCU read (0) failed\n");
		return ret;
	}

	val &= ~(REG_0_MCU_RST_1 | REG_0_MCU_RST_0);

	ret = regmap_write(priv->regmap, SX1301_MCU_CTRL, val);
	if (ret) {
		dev_err(priv->dev, "MCU write (0) failed\n");
		return ret;
	}

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

static int sx130x_loradev_open(struct net_device *netdev)
{
	struct sx1301_priv *priv = netdev_priv(netdev);
	unsigned int val;
	int ret;

	netdev_dbg(netdev, "%s", __func__);

	ret = regmap_read(priv->regmap, SX1301_GEN, &val);
	if (ret) {
		netdev_err(netdev, "16 read (1) failed\n");
		return ret;
	}

	val |= REG_16_GLOBAL_EN;

	ret = regmap_write(priv->regmap, SX1301_GEN, val);
	if (ret) {
		netdev_err(netdev, "16 write (1) failed\n");
		return ret;
	}

	ret = regmap_read(priv->regmap, SX1301_CKEN, &val);
	if (ret) {
		netdev_err(netdev, "17 read (1) failed\n");
		return ret;
	}

	val |= REG_17_CLK32M_EN;

	ret = regmap_write(priv->regmap, SX1301_CKEN, val);
	if (ret) {
		netdev_err(netdev, "17 write (1) failed\n");
		return ret;
	}

	/* calibration */

	ret = sx1301_agc_calibrate(priv);
	if (ret)
		return ret;

	/* TODO */

	ret = sx1301_load_all_firmware(priv);
	if (ret)
		return ret;

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

static void sx1301_radio_setup(struct spi_controller *ctrl)
{
	ctrl->mode_bits = SPI_CS_HIGH | SPI_NO_CS;
	ctrl->bits_per_word_mask = SPI_BPW_MASK(8);
	ctrl->num_chipselect = 1;
	ctrl->set_cs = sx1301_radio_spi_set_cs;
	ctrl->transfer_one = sx1301_radio_spi_transfer_one;
}

static int sx1301_probe(struct spi_device *spi)
{
	struct net_device *netdev;
	struct sx1301_priv *priv;
	struct spi_sx1301 *radio;
	struct gpio_desc *rst;
	int ret;
	unsigned int ver;
	unsigned int val;

	rst = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(rst))
		return PTR_ERR(rst);

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

	priv = netdev_priv(netdev);
	priv->rst_gpio = rst;

	spi_set_drvdata(spi, netdev);
	priv->dev = &spi->dev;
	priv->spi = spi;
	SET_NETDEV_DEV(netdev, &spi->dev);

	priv->regmap = devm_regmap_init_spi(spi, &sx1301_regmap_config);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err(&spi->dev, "Regmap allocation failed: %d\n", ret);
		return ret;
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

	ret = sx1301_soft_reset(priv);
	if (ret) {
		dev_err(&spi->dev, "soft reset failed\n");
		return ret;
	}

	ret = regmap_read(priv->regmap, SX1301_GEN, &val);
	if (ret) {
		dev_err(&spi->dev, "16 read failed\n");
		return ret;
	}

	val &= ~REG_16_GLOBAL_EN;

	ret = regmap_write(priv->regmap, SX1301_GEN, val);
	if (ret) {
		dev_err(&spi->dev, "16 write failed\n");
		return ret;
	}

	ret = regmap_read(priv->regmap, SX1301_CKEN, &val);
	if (ret) {
		dev_err(&spi->dev, "17 read failed\n");
		return ret;
	}

	val &= ~REG_17_CLK32M_EN;

	ret = regmap_write(priv->regmap, SX1301_CKEN, val);
	if (ret) {
		dev_err(&spi->dev, "17 write failed\n");
		return ret;
	}

	ret = regmap_read(priv->regmap, SX1301_RADIO_CFG, &val);
	if (ret) {
		dev_err(&spi->dev, "2|43 read failed\n");
		return ret;
	}

	val |= REG_2_43_RADIO_B_EN | REG_2_43_RADIO_A_EN;

	ret = regmap_write(priv->regmap, SX1301_RADIO_CFG, val);
	if (ret) {
		dev_err(&spi->dev, "2|43 write failed\n");
		return ret;
	}

	msleep(500);

	ret = regmap_read(priv->regmap, SX1301_RADIO_CFG, &val);
	if (ret) {
		dev_err(&spi->dev, "2|43 read failed\n");
		return ret;
	}

	val |= REG_2_43_RADIO_RST;

	ret = regmap_write(priv->regmap, SX1301_RADIO_CFG, val);
	if (ret) {
		dev_err(&spi->dev, "2|43 write failed\n");
		return ret;
	}

	msleep(5);

	ret = regmap_read(priv->regmap, SX1301_RADIO_CFG, &val);
	if (ret) {
		dev_err(&spi->dev, "2|43 read failed\n");
		return ret;
	}

	val &= ~REG_2_43_RADIO_RST;

	ret = regmap_write(priv->regmap, SX1301_RADIO_CFG, val);
	if (ret) {
		dev_err(&spi->dev, "2|43 write failed\n");
		return ret;
	}

	/* radio A */

	priv->radio_a_ctrl = spi_alloc_master(&spi->dev, sizeof(*radio));
	if (!priv->radio_a_ctrl) {
		return -ENOMEM;
	}

	sx1301_radio_setup(priv->radio_a_ctrl);
	priv->radio_a_ctrl->dev.of_node = of_get_child_by_name(spi->dev.of_node, "radio-a");

	radio = spi_controller_get_devdata(priv->radio_a_ctrl);
	radio->regs = SX1301_RADIO_A_SPI_DATA;
	radio->parent = spi;

	ret = devm_spi_register_controller(&spi->dev, priv->radio_a_ctrl);
	if (ret) {
		dev_err(&spi->dev, "radio A SPI register failed\n");
		spi_controller_put(priv->radio_a_ctrl);
		return ret;
	}

	/* radio B */

	priv->radio_b_ctrl = spi_alloc_master(&spi->dev, sizeof(*radio));
	if (!priv->radio_b_ctrl) {
		return -ENOMEM;
	}

	sx1301_radio_setup(priv->radio_b_ctrl);
	priv->radio_b_ctrl->dev.of_node = of_get_child_by_name(spi->dev.of_node, "radio-b");

	radio = spi_controller_get_devdata(priv->radio_b_ctrl);
	radio->regs = SX1301_RADIO_B_SPI_DATA;
	radio->parent = spi;

	ret = devm_spi_register_controller(&spi->dev, priv->radio_b_ctrl);
	if (ret) {
		dev_err(&spi->dev, "radio B SPI register failed\n");
		spi_controller_put(priv->radio_b_ctrl);
		return ret;
	}

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
