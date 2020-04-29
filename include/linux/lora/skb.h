/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * linux/lora/skb.h
 *
 * Copyright (c) 2017-2018 Andreas Färber
 */
#ifndef _LORA_SKB_H
#define _LORA_SKB_H

#include <linux/types.h>
#include <linux/skbuff.h>

enum lora_crc_stat {
	LORA_CRC_OK,
	LORA_CRC_BAD,
	LORA_NO_CRC,
	UNDEFINED
};

struct lora_skb_priv {
	int ifindex;

	u64 freq;
	u8 sf;
	u8 cr;
	u16 bw;

	u8 sync;

	s8 power;

	s8 snr;
	s8 rssi;
	enum lora_crc_stat crc;
};

static inline struct lora_skb_priv *lora_skb_prv(struct sk_buff *skb)
{
	return (struct lora_skb_priv *)(skb->head);
}

static inline void lora_skb_reserve(struct sk_buff *skb)
{
	skb_reserve(skb, sizeof(struct lora_skb_priv));
}

struct sk_buff *alloc_lora_skb(struct net_device *dev, u8 **data);

#endif /* _LORA_SKB_H */
