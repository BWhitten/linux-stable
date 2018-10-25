/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Semtech SX1301 LoRa concentrator
 *
 * Copyright (c) 2018   Ben Whitten
 * Copyright (c) 2018 Andreas Färber
 */

#ifndef _SX1301_
#define _SX1301_

#include <linux/clk.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/lora/dev.h>

#define SX1301_CHIP_VERSION 103

#define SX1301_MCU_FW_BYTE 8192
#define SX1301_MCU_ARB_FW_VERSION 1
#define SX1301_MCU_AGC_FW_VERSION 4
#define SX1301_MCU_AGC_CAL_FW_VERSION 2

#define SX1301_AGC_CMD_WAIT 16
#define SX1301_AGC_CMD_ABORT 17

#define SX1301_TX_GAIN_LUT_MAX 16

/* Page independent */
#define SX1301_PAGE     0x00
#define SX1301_VER      0x01
#define SX1301_RX_DATA_BUF_ADDR 0x02 /* 16 wide */
#define SX1301_RX_DATA_BUF_DATA 0x04
#define SX1301_TX_DATA_BUF_ADDR 0x05
#define SX1301_TX_DATA_BUF_DATA 0x06
#define SX1301_MPA      0x09
#define SX1301_MPD      0x0A
#define SX1301_GEN      0x10
#define SX1301_CKEN     0x11
#define SX1301_GPSO     0x1C
#define SX1301_GPMODE   0x1D
#define SX1301_AGCSTS   0x20

#define SX1301_VIRT_BASE    0x100
#define SX1301_PAGE_LEN     0x80
#define SX1301_PAGE_BASE(n) (SX1301_VIRT_BASE + (SX1301_PAGE_LEN * n))

/* Page 0 */
#define SX1301_CHRS         (SX1301_PAGE_BASE(0) + 0x23)
#define SX1301_GAIN_OFFSET  (SX1301_PAGE_BASE(0) + 0x68)
#define SX1301_FORCE_CTRL   (SX1301_PAGE_BASE(0) + 0x69)
#define SX1301_MCU_CTRL     (SX1301_PAGE_BASE(0) + 0x6A)
#define SX1301_RSSI_BB_DEFAULT_VALUE 	(SX1301_PAGE_BASE(0) + 0x6C)
#define SX1301_RSSI_DEC_DEFAULT_VALUE 	(SX1301_PAGE_BASE(0) + 0x6D)
#define SX1301_RSSI_CHANN_DEFAULT_VALUE (SX1301_PAGE_BASE(0) + 0x6E)
#define SX1301_RSSI_BB_FILTER_ALPHA 	(SX1301_PAGE_BASE(0) + 0x6F)
#define SX1301_RSSI_DEC_FILTER_ALPHA 	(SX1301_PAGE_BASE(0) + 0x70)
#define SX1301_RSSI_CHANN_FILTER_ALPHA 	(SX1301_PAGE_BASE(0) + 0x71)

/* Page 1 */
#define SX1301_TX_TRIG      (SX1301_PAGE_BASE(1) + 0x21)

/* Page 2 */
#define SX1301_RADIO_A_SPI_DATA     (SX1301_PAGE_BASE(2) + 0x21)
#define SX1301_RADIO_A_SPI_DATA_RB  (SX1301_PAGE_BASE(2) + 0x22)
#define SX1301_RADIO_A_SPI_ADDR     (SX1301_PAGE_BASE(2) + 0x23)
#define SX1301_RADIO_A_SPI_CS       (SX1301_PAGE_BASE(2) + 0x25)
#define SX1301_RADIO_B_SPI_DATA     (SX1301_PAGE_BASE(2) + 0x26)
#define SX1301_RADIO_B_SPI_DATA_RB  (SX1301_PAGE_BASE(2) + 0x27)
#define SX1301_RADIO_B_SPI_ADDR     (SX1301_PAGE_BASE(2) + 0x28)
#define SX1301_RADIO_B_SPI_CS       (SX1301_PAGE_BASE(2) + 0x2A)
#define SX1301_RADIO_CFG            (SX1301_PAGE_BASE(2) + 0x2B)
#define SX1301_DBG_ARB_MCU_RAM_DATA (SX1301_PAGE_BASE(2) + 0x40)
#define SX1301_DBG_AGC_MCU_RAM_DATA (SX1301_PAGE_BASE(2) + 0x41)
#define SX1301_DBG_ARB_MCU_RAM_ADDR (SX1301_PAGE_BASE(2) + 0x50)
#define SX1301_DBG_AGC_MCU_RAM_ADDR (SX1301_PAGE_BASE(2) + 0x51)

/* Page 3 */
#define SX1301_EMERGENCY_FORCE_HOST_CTRL (SX1301_PAGE_BASE(3) + 0x7F)

#define SX1301_MAX_REGISTER         (SX1301_PAGE_BASE(3) + 0x7F)

enum sx1301_fields {
	F_SOFT_RESET,
	F_GLOBAL_EN,
	F_CLK32M_EN,
	F_RADIO_A_EN,
	F_RADIO_B_EN,
	F_RADIO_RST,

	F_MCU_RST_0,
	F_MCU_RST_1,
	F_MCU_SELECT_MUX_0,
	F_MCU_SELECT_MUX_1,

	F_FORCE_HOST_RADIO_CTRL,
	F_FORCE_HOST_FE_CTRL,
	F_FORCE_DEC_FILTER_GAIN,

	F_EMERGENCY_FORCE_HOST_CTRL,

	F_RSSI_BB_FILTER_ALPHA,
	F_RSSI_DEC_FILTER_ALPHA,
	F_RSSI_CHANN_FILTER_ALPHA,

	F_RSSI_BB_DEFAULT_VALUE,
	F_RSSI_DEC_DEFAULT_VALUE,
	F_RSSI_CHANN_DEFAULT_VALUE,

	F_DEC_GAIN_OFFSET,
	F_CHAN_GAIN_OFFSET,

	F_TX_TRIG_IMMEDIATE,
	F_TX_TRIG_DELAYED,
	F_TX_TRIG_GPS,
};

struct sx1301_fields_sequence {
	enum sx1301_fields field;
	u8 val;
};

static const struct sx1301_fields_sequence sx1301_regmap_fields_patch[] = {
    /* I/Q path setup */
	{F_RSSI_BB_FILTER_ALPHA,	6},
	{F_RSSI_DEC_FILTER_ALPHA,	7},
	{F_RSSI_CHANN_FILTER_ALPHA,	7},
	{F_RSSI_BB_DEFAULT_VALUE,	23},
	{F_RSSI_DEC_DEFAULT_VALUE,	85},
	{F_RSSI_CHANN_DEFAULT_VALUE,66},
	{F_DEC_GAIN_OFFSET,			7},
	{F_CHAN_GAIN_OFFSET,		6},
};

static const struct reg_field sx1301_regmap_fields[] = {
	/* PAGE */
	[F_SOFT_RESET]          = REG_FIELD(SX1301_PAGE, 7, 7),
	/* GEN */
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

	/* TX_TRIG */
	[F_TX_TRIG_IMMEDIATE] = REG_FIELD(SX1301_TX_TRIG, 0, 0),
	[F_TX_TRIG_DELAYED] = REG_FIELD(SX1301_TX_TRIG, 1, 1),
	[F_TX_TRIG_GPS] = REG_FIELD(SX1301_TX_TRIG, 2, 2),
};

struct sx1301_tx_gain_lut {
	unsigned int 	dig_gain:2,
			pa_gain:2,
			dac_gain:2,
			mix_gain:4;
	int rf_power;	/* dBm measured at board connector */
};

struct sx1301_priv {
	struct lora_dev_priv lora;
	struct device		*dev;
	struct clk		*clk32m;
	struct gpio_desc *rst_gpio;
	struct regmap		*regmap;
	struct regmap_field     *regmap_fields[ARRAY_SIZE(sx1301_regmap_fields)];

	struct sk_buff *tx_skb;

	struct workqueue_struct *wq;
	struct work_struct tx_work;

	struct sx1301_tx_gain_lut tx_gain_lut[SX1301_TX_GAIN_LUT_MAX];
	u8 tx_gain_lut_size;
};

int __init sx130x_radio_init(void);
void __exit sx130x_radio_exit(void);
int sx130x_register_radio_devices(struct device *dev);
int devm_sx130x_register_radio_devices(struct device *dev);
void sx130x_unregister_radio_devices(struct device *dev);
bool sx130x_radio_devices_okay(struct device *dev);

#endif
