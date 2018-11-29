// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Semtech SX1301 LoRa concentrator
 *
 * Copyright (c) 2018 Andreas Färber
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
#include <linux/lora/dev.h>
#include <linux/spi/spi.h>

#define REG_PAGE_RESET			0
#define REG_VERSION			1
#define REG_MCU_PROM_ADDR		9
#define REG_MCU_PROM_DATA		10
#define REG_GPIO_SELECT_INPUT		27
#define REG_GPIO_SELECT_OUTPUT		28
#define REG_GPIO_MODE			29
#define REG_MCU_AGC_STATUS		32
#define REG_0_RADIO_SELECT		35
#define REG_0_MCU			106
#define REG_2_SPI_RADIO_A_DATA		33
#define REG_2_SPI_RADIO_A_DATA_READBACK	34
#define REG_2_SPI_RADIO_A_ADDR		35
#define REG_2_SPI_RADIO_A_CS		37
#define REG_2_SPI_RADIO_B_DATA		38
#define REG_2_SPI_RADIO_B_DATA_READBACK	39
#define REG_2_SPI_RADIO_B_ADDR		40
#define REG_2_SPI_RADIO_B_CS		42
#define REG_2_DBG_ARB_MCU_RAM_DATA	64
#define REG_2_DBG_AGC_MCU_RAM_DATA	65
#define REG_2_DBG_ARB_MCU_RAM_ADDR	80
#define REG_2_DBG_AGC_MCU_RAM_ADDR	81
#define REG_EMERGENCY_FORCE		127

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

struct spi_sx1301 {
	struct spi_device *parent;
	u8 page;
	u8 regs;
};

struct sx1301_priv {
	struct lora_dev_priv lora;
	struct gpio_desc *rst_gpio;
	u8 cur_page;
	struct spi_controller *radio_a_ctrl, *radio_b_ctrl;
};

static int sx1301_read_burst(struct spi_device *spi, u8 reg, u8 *val, size_t len)
{
	u8 addr = reg & 0x7f;
	return spi_write_then_read(spi, &addr, 1, val, len);
}

static int sx1301_read(struct spi_device *spi, u8 reg, u8 *val)
{
	return sx1301_read_burst(spi, reg, val, 1);
}

static int sx1301_write_burst(struct spi_device *spi, u8 reg, const u8 *val, size_t len)
{
	u8 addr = reg | BIT(7);
	struct spi_transfer xfr[2] = {
		{ .tx_buf = &addr, .len = 1 },
		{ .tx_buf = val, .len = len },
	};

	return spi_sync_transfer(spi, xfr, 2);
}

static int sx1301_write(struct spi_device *spi, u8 reg, u8 val)
{
	return sx1301_write_burst(spi, reg, &val, 1);
}

static int sx1301_page_switch(struct spi_device *spi, u8 page)
{
	struct sx1301_priv *priv = spi_get_drvdata(spi);
	int ret;

	if (priv->cur_page == page)
		return 0;

	dev_dbg(&spi->dev, "switching to page %u\n", (unsigned)page);
	ret = sx1301_write(spi, REG_PAGE_RESET, page & 0x3);
	if (ret) {
		dev_err(&spi->dev, "switching to page %u failed\n", (unsigned)page);
		return ret;
	}

	priv->cur_page = page;

	return 0;
}

static int sx1301_page_read(struct spi_device *spi, u8 page, u8 reg, u8 *val)
{
	int ret;

	ret = sx1301_page_switch(spi, page);
	if (ret)
		return ret;

	return sx1301_read(spi, reg, val);
}

static int sx1301_page_write(struct spi_device *spi, u8 page, u8 reg, u8 val)
{
	int ret;

	ret = sx1301_page_switch(spi, page);
	if (ret)
		return ret;

	return sx1301_write(spi, reg, val);
}

static int sx1301_soft_reset(struct spi_device *spi)
{
	return sx1301_write(spi, REG_PAGE_RESET, REG_PAGE_RESET_SOFT_RESET);
}

#define REG_RADIO_X_DATA		0
#define REG_RADIO_X_DATA_READBACK	1
#define REG_RADIO_X_ADDR		2
#define REG_RADIO_X_CS			4

static int sx1301_radio_set_cs(struct spi_controller *ctrl, bool enable)
{
	struct spi_sx1301 *ssx = spi_controller_get_devdata(ctrl);
	u8 cs;
	int ret;

	dev_dbg(&ctrl->dev, "setting CS to %s\n", enable ? "1" : "0");

	ret = sx1301_page_read(ssx->parent, ssx->page, ssx->regs + REG_RADIO_X_CS, &cs);
	if (ret) {
		dev_warn(&ctrl->dev, "failed to read CS (%d)\n", ret);
		cs = 0;
	}

	if (enable)
		cs |= BIT(0);
	else
		cs &= ~BIT(0);

	ret = sx1301_page_write(ssx->parent, ssx->page, ssx->regs + REG_RADIO_X_CS, cs);
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
	const u8 *tx_buf = xfr->tx_buf;
	u8 *rx_buf = xfr->rx_buf;
	int ret;

	if (xfr->len == 0 || xfr->len > 3)
		return -EINVAL;

	dev_dbg(&spi->dev, "transferring one (%u)\n", xfr->len);

	if (tx_buf) {
		ret = sx1301_page_write(ssx->parent, ssx->page, ssx->regs + REG_RADIO_X_ADDR, tx_buf ? tx_buf[0] : 0);
		if (ret) {
			dev_err(&spi->dev, "SPI radio address write failed\n");
			return ret;
		}

		ret = sx1301_page_write(ssx->parent, ssx->page, ssx->regs + REG_RADIO_X_DATA, (tx_buf && xfr->len >= 2) ? tx_buf[1] : 0);
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
		ret = sx1301_page_read(ssx->parent, ssx->page, ssx->regs + REG_RADIO_X_DATA_READBACK, &rx_buf[xfr->len - 1]);
		if (ret) {
			dev_err(&spi->dev, "SPI radio data read failed\n");
			return ret;
		}
	}

	return 0;
}

static int sx1301_agc_ram_read(struct spi_device *spi, u8 addr, u8 *val)
{
	int ret;

	ret = sx1301_page_write(spi, 2, REG_2_DBG_AGC_MCU_RAM_ADDR, addr);
	if (ret) {
		dev_err(&spi->dev, "AGC RAM addr write failed\n");
		return ret;
	}

	ret = sx1301_page_read(spi, 2, REG_2_DBG_AGC_MCU_RAM_DATA, val);
	if (ret) {
		dev_err(&spi->dev, "AGC RAM data read failed\n");
		return ret;
	}

	return 0;
}

static int sx1301_arb_ram_read(struct spi_device *spi, u8 addr, u8 *val)
{
	int ret;

	ret = sx1301_page_write(spi, 2, REG_2_DBG_ARB_MCU_RAM_ADDR, addr);
	if (ret) {
		dev_err(&spi->dev, "ARB RAM addr write failed\n");
		return ret;
	}

	ret = sx1301_page_read(spi, 2, REG_2_DBG_ARB_MCU_RAM_DATA, val);
	if (ret) {
		dev_err(&spi->dev, "ARB RAM data read failed\n");
		return ret;
	}

	return 0;
}

static int sx1301_load_firmware(struct spi_device *spi, int mcu, const u8 *data, size_t len)
{
	u8 *buf;
	u8 val, rst, select_mux;
	int ret;

	if (len > 8192)
		return -EINVAL;

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

	ret = sx1301_page_read(spi, 0, REG_0_MCU, &val);
	if (ret) {
		dev_err(&spi->dev, "MCU read failed\n");
		return ret;
	}

	val |= rst;
	val &= ~select_mux;

	ret = sx1301_page_write(spi, 0, REG_0_MCU, val);
	if (ret) {
		dev_err(&spi->dev, "MCU reset / select mux write failed\n");
		return ret;
	}

	ret = sx1301_write(spi, REG_MCU_PROM_ADDR, 0);
	if (ret) {
		dev_err(&spi->dev, "MCU prom addr write failed\n");
		return ret;
	}

	ret = sx1301_write_burst(spi, REG_MCU_PROM_DATA, data, len);
	if (ret) {
		dev_err(&spi->dev, "MCU prom data write failed\n");
		return ret;
	}

	ret = sx1301_read(spi, REG_MCU_PROM_DATA, &val);
	if (ret) {
		dev_err(&spi->dev, "MCU prom data dummy read failed\n");
		return ret;
	}

	buf = kzalloc(len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = sx1301_read_burst(spi, REG_MCU_PROM_DATA, buf, len);
	if (ret) {
		dev_err(&spi->dev, "MCU prom data read failed\n");
		kfree(buf);
		return ret;
	}

	if (memcmp(data, buf, len)) {
		dev_err(&spi->dev, "MCU prom data read does not match data written\n");
		kfree(buf);
		return -ENXIO;
	}

	kfree(buf);

	ret = sx1301_page_read(spi, 0, REG_0_MCU, &val);
	if (ret) {
		dev_err(&spi->dev, "MCU read (1) failed\n");
		return ret;
	}

	val |= select_mux;

	ret = sx1301_page_write(spi, 0, REG_0_MCU, val);
	if (ret) {
		dev_err(&spi->dev, "MCU reset / select mux write (1) failed\n");
		return ret;
	}

	return 0;
}

static int sx1301_agc_calibrate(struct spi_device *spi)
{
	const struct firmware *fw;
	u8 val;
	int ret;

	ret = request_firmware(&fw, "sx1301_agc_calibration.bin", &spi->dev);
	if (ret) {
		dev_err(&spi->dev, "agc cal firmware file load failed\n");
		return ret;
	}

	if (fw->size != 8192) {
		dev_err(&spi->dev, "unexpected agc cal firmware size\n");
		return -EINVAL;
	}

	ret = sx1301_load_firmware(spi, 1, fw->data, fw->size);
	release_firmware(fw);
	if (ret) {
		dev_err(&spi->dev, "agc cal firmware load failed\n");
		return ret;
	}

	ret = sx1301_page_read(spi, 0, 105, &val);
	if (ret) {
		dev_err(&spi->dev, "0|105 read failed\n");
		return ret;
	}

	val &= ~REG_0_105_FORCE_HOST_RADIO_CTRL;

	ret = sx1301_page_write(spi, 0, 105, val);
	if (ret) {
		dev_err(&spi->dev, "0|105 write failed\n");
		return ret;
	}

	val = BIT(4); /* with DAC gain=3 */
	if (false)
		val |= BIT(5); /* SX1255 */

	ret = sx1301_page_write(spi, 0, REG_0_RADIO_SELECT, val);
	if (ret) {
		dev_err(&spi->dev, "radio select write failed\n");
		return ret;
	}

	ret = sx1301_page_read(spi, 0, REG_0_MCU, &val);
	if (ret) {
		dev_err(&spi->dev, "MCU read (0) failed\n");
		return ret;
	}

	val &= ~REG_0_MCU_RST_1;

	ret = sx1301_page_write(spi, 0, REG_0_MCU, val);
	if (ret) {
		dev_err(&spi->dev, "MCU write (0) failed\n");
		return ret;
	}

	ret = sx1301_agc_ram_read(spi, 0x20, &val);
	if (ret) {
		dev_err(&spi->dev, "AGC RAM data read failed\n");
		return ret;
	}

	dev_info(&spi->dev, "AGC calibration firmware version %u\n", (unsigned)val);

	if (val != 2) {
		dev_err(&spi->dev, "unexpected firmware version, expecting %u\n", 2);
		return -ENXIO;
	}

	ret = sx1301_page_switch(spi, 3);
	if (ret) {
		dev_err(&spi->dev, "page switch 3 failed\n");
		return ret;
	}

	ret = sx1301_read(spi, REG_EMERGENCY_FORCE, &val);
	if (ret) {
		dev_err(&spi->dev, "emergency force read failed\n");
		return ret;
	}

	val &= ~REG_EMERGENCY_FORCE_HOST_CTRL;

	ret = sx1301_write(spi, REG_EMERGENCY_FORCE, val);
	if (ret) {
		dev_err(&spi->dev, "emergency force write failed\n");
		return ret;
	}

	dev_err(&spi->dev, "starting calibration...\n");
	msleep(2300);

	ret = sx1301_read(spi, REG_EMERGENCY_FORCE, &val);
	if (ret) {
		dev_err(&spi->dev, "emergency force read (1) failed\n");
		return ret;
	}

	val |= REG_EMERGENCY_FORCE_HOST_CTRL;

	ret = sx1301_write(spi, REG_EMERGENCY_FORCE, val);
	if (ret) {
		dev_err(&spi->dev, "emergency force write (1) failed\n");
		return ret;
	}

	ret = sx1301_read(spi, REG_MCU_AGC_STATUS, &val);
	if (ret) {
		dev_err(&spi->dev, "AGC status read failed\n");
		return ret;
	}

	dev_info(&spi->dev, "AGC status: %02x\n", (unsigned)val);
	if ((val & (BIT(7) | BIT(0))) != (BIT(7) | BIT(0))) {
		dev_err(&spi->dev, "AGC calibration failed\n");
		return -ENXIO;
	}

	return 0;
}

static int sx1301_load_all_firmware(struct spi_device *spi)
{
	const struct firmware *fw;
	u8 val;
	int ret;

	ret = request_firmware(&fw, "sx1301_arb.bin", &spi->dev);
	if (ret) {
		dev_err(&spi->dev, "arb firmware file load failed\n");
		return ret;
	}

	if (fw->size != 8192) {
		dev_err(&spi->dev, "unexpected arb firmware size\n");
		release_firmware(fw);
		return -EINVAL;
	}

	ret = sx1301_load_firmware(spi, 0, fw->data, fw->size);
	release_firmware(fw);
	if (ret)
		return ret;

	ret = request_firmware(&fw, "sx1301_agc.bin", &spi->dev);
	if (ret) {
		dev_err(&spi->dev, "agc firmware file load failed\n");
		return ret;
	}

	if (fw->size != 8192) {
		dev_err(&spi->dev, "unexpected agc firmware size\n");
		release_firmware(fw);
		return -EINVAL;
	}

	ret = sx1301_load_firmware(spi, 1, fw->data, fw->size);
	release_firmware(fw);
	if (ret)
		return ret;

	ret = sx1301_page_read(spi, 0, 105, &val);
	if (ret) {
		dev_err(&spi->dev, "0|105 read failed\n");
		return ret;
	}

	val &= ~(REG_0_105_FORCE_HOST_RADIO_CTRL | REG_0_105_FORCE_HOST_FE_CTRL | REG_0_105_FORCE_DEC_FILTER_GAIN);

	ret = sx1301_page_write(spi, 0, 105, val);
	if (ret) {
		dev_err(&spi->dev, "0|105 write failed\n");
		return ret;
	}

	ret = sx1301_page_write(spi, 0, REG_0_RADIO_SELECT, 0);
	if (ret) {
		dev_err(&spi->dev, "radio select write failed\n");
		return ret;
	}

	ret = sx1301_page_read(spi, 0, REG_0_MCU, &val);
	if (ret) {
		dev_err(&spi->dev, "MCU read (0) failed\n");
		return ret;
	}

	val &= ~(REG_0_MCU_RST_1 | REG_0_MCU_RST_0);

	ret = sx1301_page_write(spi, 0, REG_0_MCU, val);
	if (ret) {
		dev_err(&spi->dev, "MCU write (0) failed\n");
		return ret;
	}

	ret = sx1301_agc_ram_read(spi, 0x20, &val);
	if (ret) {
		dev_err(&spi->dev, "AGC RAM data read failed\n");
		return ret;
	}

	dev_info(&spi->dev, "AGC firmware version %u\n", (unsigned)val);

	if (val != 4) {
		dev_err(&spi->dev, "unexpected firmware version, expecting %u\n", 4);
		return -ENXIO;
	}

	ret = sx1301_arb_ram_read(spi, 0x20, &val);
	if (ret) {
		dev_err(&spi->dev, "ARB RAM data read failed\n");
		return ret;
	}

	dev_info(&spi->dev, "ARB firmware version %u\n", (unsigned)val);

	if (val != 1) {
		dev_err(&spi->dev, "unexpected firmware version, expecting %u\n", 1);
		return -ENXIO;
	}

	return 0;
}

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
	u8 val;

	rst = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(rst))
		return PTR_ERR(rst);

	gpiod_set_value_cansleep(rst, 1);
	msleep(100);
	gpiod_set_value_cansleep(rst, 0);
	msleep(100);

	spi->bits_per_word = 8;
	spi_setup(spi);

	ret = sx1301_read(spi, REG_VERSION, &val);
	if (ret) {
		dev_err(&spi->dev, "version read failed\n");
		goto err_version;
	}

	if (val != 103) {
		dev_err(&spi->dev, "unexpected version: %u\n", val);
		ret = -ENXIO;
		goto err_version;
	}

	netdev = alloc_loradev(sizeof(*priv));
	if (!netdev) {
		ret = -ENOMEM;
		goto err_alloc_loradev;
	}

	priv = netdev_priv(netdev);
	priv->rst_gpio = rst;
	priv->cur_page = 0xff;

	spi_set_drvdata(spi, netdev);
	SET_NETDEV_DEV(netdev, &spi->dev);

	ret = sx1301_write(spi, REG_PAGE_RESET, 0);
	if (ret) {
		dev_err(&spi->dev, "page/reset write failed\n");
		goto err_init_page;
	}

	ret = sx1301_soft_reset(spi);
	if (ret) {
		dev_err(&spi->dev, "soft reset failed\n");
		goto err_soft_reset;
	}

	ret = sx1301_read(spi, 16, &val);
	if (ret) {
		dev_err(&spi->dev, "16 read failed\n");
		goto err_read_global_en_0;
	}

	val &= ~REG_16_GLOBAL_EN;

	ret = sx1301_write(spi, 16, val);
	if (ret) {
		dev_err(&spi->dev, "16 write failed\n");
		goto err_write_global_en_0;
	}

	ret = sx1301_read(spi, 17, &val);
	if (ret) {
		dev_err(&spi->dev, "17 read failed\n");
		goto err_read_clk32m_0;
	}

	val &= ~REG_17_CLK32M_EN;

	ret = sx1301_write(spi, 17, val);
	if (ret) {
		dev_err(&spi->dev, "17 write failed\n");
		goto err_write_clk32m_0;
	}

	ret = sx1301_page_read(spi, 2, 43, &val);
	if (ret) {
		dev_err(&spi->dev, "2|43 read failed\n");
		return ret;
	}

	val |= REG_2_43_RADIO_B_EN | REG_2_43_RADIO_A_EN;

	ret = sx1301_page_write(spi, 2, 43, val);
	if (ret) {
		dev_err(&spi->dev, "2|43 write failed\n");
		return ret;
	}

	msleep(500);

	ret = sx1301_page_read(spi, 2, 43, &val);
	if (ret) {
		dev_err(&spi->dev, "2|43 read failed\n");
		return ret;
	}

	val |= REG_2_43_RADIO_RST;

	ret = sx1301_page_write(spi, 2, 43, val);
	if (ret) {
		dev_err(&spi->dev, "2|43 write failed\n");
		return ret;
	}

	msleep(5);

	ret = sx1301_page_read(spi, 2, 43, &val);
	if (ret) {
		dev_err(&spi->dev, "2|43 read failed\n");
		return ret;
	}

	val &= ~REG_2_43_RADIO_RST;

	ret = sx1301_page_write(spi, 2, 43, val);
	if (ret) {
		dev_err(&spi->dev, "2|43 write failed\n");
		return ret;
	}

	/* radio A */

	priv->radio_a_ctrl = spi_alloc_master(&spi->dev, sizeof(*radio));
	if (!priv->radio_a_ctrl) {
		ret = -ENOMEM;
		goto err_radio_a_alloc;
	}

	sx1301_radio_setup(priv->radio_a_ctrl);
	priv->radio_a_ctrl->dev.of_node = of_get_child_by_name(spi->dev.of_node, "radio-a");

	radio = spi_controller_get_devdata(priv->radio_a_ctrl);
	radio->page = 2;
	radio->regs = REG_2_SPI_RADIO_A_DATA;
	radio->parent = spi;

	ret = devm_spi_register_controller(&spi->dev, priv->radio_a_ctrl);
	if (ret) {
		dev_err(&spi->dev, "radio A SPI register failed\n");
		spi_controller_put(priv->radio_a_ctrl);
		goto err_radio_a_register;
	}

	/* radio B */

	priv->radio_b_ctrl = spi_alloc_master(&spi->dev, sizeof(*radio));
	if (!priv->radio_b_ctrl) {
		ret = -ENOMEM;
		goto err_radio_b_alloc;
	}

	sx1301_radio_setup(priv->radio_b_ctrl);
	priv->radio_b_ctrl->dev.of_node = of_get_child_by_name(spi->dev.of_node, "radio-b");

	radio = spi_controller_get_devdata(priv->radio_b_ctrl);
	radio->page = 2;
	radio->regs = REG_2_SPI_RADIO_B_DATA;
	radio->parent = spi;

	ret = devm_spi_register_controller(&spi->dev, priv->radio_b_ctrl);
	if (ret) {
		dev_err(&spi->dev, "radio B SPI register failed\n");
		spi_controller_put(priv->radio_b_ctrl);
		goto err_radio_b_register;
	}

	/* GPIO */

	ret = sx1301_read(spi, REG_GPIO_MODE, &val);
	if (ret) {
		dev_err(&spi->dev, "GPIO mode read failed\n");
		goto err_read_gpio_mode;
	}

	val |= GENMASK(4, 0);

	ret = sx1301_write(spi, REG_GPIO_MODE, val);
	if (ret) {
		dev_err(&spi->dev, "GPIO mode write failed\n");
		goto err_write_gpio_mode;
	}

	ret = sx1301_read(spi, REG_GPIO_SELECT_OUTPUT, &val);
	if (ret) {
		dev_err(&spi->dev, "GPIO select output read failed\n");
		goto err_read_gpio_select_output;
	}

	val &= ~GENMASK(3, 0);
	val |= 2;

	ret = sx1301_write(spi, REG_GPIO_SELECT_OUTPUT, val);
	if (ret) {
		dev_err(&spi->dev, "GPIO select output write failed\n");
		goto err_write_gpio_select_output;
	}

	/* TODO LBT */

	ret = sx1301_read(spi, 16, &val);
	if (ret) {
		dev_err(&spi->dev, "16 read (1) failed\n");
		goto err_read_global_en_1;
	}

	val |= REG_16_GLOBAL_EN;

	ret = sx1301_write(spi, 16, val);
	if (ret) {
		dev_err(&spi->dev, "16 write (1) failed\n");
		goto err_write_global_en_1;
	}

	ret = sx1301_read(spi, 17, &val);
	if (ret) {
		dev_err(&spi->dev, "17 read (1) failed\n");
		goto err_read_clk32m_1;
	}

	val |= REG_17_CLK32M_EN;

	ret = sx1301_write(spi, 17, val);
	if (ret) {
		dev_err(&spi->dev, "17 write (1) failed\n");
		goto err_write_clk32m_1;
	}

	/* calibration */

	ret = sx1301_agc_calibrate(spi);
	if (ret)
		goto err_agc_calibrate;

	/* TODO */

	ret = sx1301_load_all_firmware(spi);
	if (ret)
		goto err_load_firmware;

	dev_info(&spi->dev, "SX1301 module probed\n");

	return 0;

err_load_firmware:
err_agc_calibrate:
err_write_clk32m_1:
err_read_clk32m_1:
err_write_global_en_1:
err_read_global_en_1:
err_write_gpio_select_output:
err_read_gpio_select_output:
err_write_gpio_mode:
err_read_gpio_mode:
err_radio_b_register:
err_radio_b_alloc:
err_radio_a_register:
err_radio_a_alloc:
err_write_clk32m_0:
err_read_clk32m_0:
err_write_global_en_0:
err_read_global_en_0:
err_soft_reset:
err_init_page:
	free_loradev(netdev);
err_alloc_loradev:
err_version:
	return ret;
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
MODULE_LICENSE("GPL");
