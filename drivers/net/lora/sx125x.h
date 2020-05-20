/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Semtech SX1255/SX1257 LoRa transceiver
 *
 * Copyright (c) 2018 Ben Whitten
 */

#ifndef _SX125X_
#define _SX125X_

#define SX1255_VERSION		0x07
#define SX125X_TX_GAIN		0x08
#define SX125X_TX_BW		0x0A
#define SX125X_TX_DAC_BW	0x0B
#define SX125X_RX_ANA_GAIN	0x0C
#define SX125X_RX_BW		0x0D
#define SX125X_RX_PLL_BW	0x0E
#define SX125X_CLK_SELECT	0x10

#define SX1257_XOSC		0x26
#define SX1255_XOSC		0x28

#define SX125X_MAX_REGISTER	0x2A

#endif
