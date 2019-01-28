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

/* TX addressing definition */
struct tx_addr {
	__u64 freq;
	__u8 sf;
	__u8 cr;
	__u16 bw;

	__u8 sync;

	__s8 power;
};

/* particular protocols of the protocol family PF_LORA */
#define LORA_PROTO_DATAGRAM	0
#define LORA_NPROTO		1

struct sockaddr_lora {
	__kernel_sa_family_t lora_family;
	int lora_ifindex;
	__u8 lora_protocol;
	union {
		struct tx_addr	tx;
	} lora_addr;
};

#endif /* _UAPI_LINUX_LORA_H */
