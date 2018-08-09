/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Semtech SX1301 LoRa concentrator
 *
 * Copyright (c) 2018   Ben Whitten
 */

#ifndef _SX1301_
#define _SX1301_

#define SX1301_CHIP_VERSION 103

#define SX1301_MCU_FW_BYTE 8192
#define SX1301_MCU_ARB_FW_VERSION 1
#define SX1301_MCU_AGC_FW_VERSION 4
#define SX1301_MCU_AGC_CAL_FW_VERSION 2

/* Page independent */
#define SX1301_PAGE     0x00
#define SX1301_VER      0x01

#define SX1301_VIRT_BASE    0x100
#define SX1301_PAGE_LEN     0x80
#define SX1301_PAGE_BASE(n) (SX1301_VIRT_BASE + (SX1301_PAGE_LEN * n))

#define SX1301_MAX_REGISTER         (SX1301_PAGE_BASE(3) + 0x7F)

#endif
