/* SPDX-License-Identifier: (GPL-2.0-or-later WITH Linux-syscall-note) */
/*
 * linux/lora.h
 *
 * Copyright (c) 2017-2018 Andreas Färber
 */
#ifndef _UAPI_LINUX_LORA_H
#define _UAPI_LINUX_LORA_H

#include <linux/types.h>
#include <linux/socket.h>

/* particular protocols of the protocol family PF_LORA */
#define LORA_PROTO_DATAGRAM	0
#define LORA_NPROTO		1

struct sockaddr_lora {
	__kernel_sa_family_t lora_family;
	int lora_ifindex;
	union {
	} lora_addr;
};

#define LORA_MAX_DLEN	256

enum lora_bw {
	LORA_BW_125KHZ,
	LORA_BW_250KHZ,
	LORA_BW_500KHZ,
};

enum lora_cr {
	LORA_CR_4_5,
	LORA_CR_4_6,
	LORA_CR_4_7,
	LORA_CR_4_8,
};

enum lora_sf {
	LORA_SF_6,
	LORA_SF_7,
	LORA_SF_8,
	LORA_SF_9,
	LORA_SF_10,
	LORA_SF_11,
	LORA_SF_12,
};

/**
 * struct lora_frame - LoRa frame structure
 * @freq: Frequency of LoRa transmission in Hz
 * @power: Power of transmission in dBm
 * @bw:  bandwith, 125, 250 or 500 KHz
 * @cr:  coding rate, 4/5 to 4/8
 * @sf:  spreading factor from SF6 to 12
 * @data:   LoRa frame payload (up to LORA_MAX_DLEN byte)
 */
struct lora_frame {
	__u32			freq; /* transmission frequency in Hz */
	__u8			power; /* transmission power in dBm */
	enum lora_bw 	bw; /* bandwith */
	enum lora_cr 	cr; /* coding rate */
	enum lora_sf 	sf; /* spreading factor */
	__u8			len;
	__u8			data[LORA_MAX_DLEN] __attribute__((aligned(8)));
};

#define LORA_MTU (sizeof(struct lora_frame))

#endif /* _UAPI_LINUX_LORA_H */
