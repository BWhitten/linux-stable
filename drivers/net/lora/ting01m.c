// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Widora   Ting-01M
 * Himalaya HIMO-01M
 *
 * Copyright (c) 2017-2018 Andreas Färber
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/lora.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/serdev.h>
#include <linux/lora/dev.h>

#include "ting01m.h"

struct ting01m_priv {
	struct lora_priv lora;
};

static void widora_reset_mcu(struct widora_device *widev)
{
	gpiod_set_value_cansleep(widev->rst, 0);
	msleep(200);
	gpiod_set_value_cansleep(widev->rst, 1);
	msleep(500);
}

static netdev_tx_t ting01m_loradev_start_xmit(struct sk_buff *skb, struct net_device *netdev)
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

static int ting01m_loradev_open(struct net_device *netdev)
{
	int ret;

	netdev_dbg(netdev, "%s", __func__);

	ret = open_loradev(netdev);
	if (ret)
		return ret;

	netif_start_queue(netdev);

	return 0;
}

static int ting01m_loradev_stop(struct net_device *netdev)
{
	netdev_dbg(netdev, "%s", __func__);

	netif_stop_queue(netdev);
	close_loradev(netdev);

	return 0;
}

static const struct net_device_ops ting01m_net_device_ops = {
	.ndo_open = ting01m_loradev_open,
	.ndo_stop = ting01m_loradev_stop,
	.ndo_start_xmit = ting01m_loradev_start_xmit,
};

static int widora_receive_buf(struct serdev_device *sdev, const u8 *data, size_t count)
{
	struct widora_device *widev = serdev_device_get_drvdata(sdev);
	size_t i = 0;
	int len = 0;

	dev_dbg(&sdev->dev, "Receive (%d)\n", (int)count);

	for (i = 0; i < count; i++) {
		dev_dbg(&sdev->dev, "Receive: 0x%02x\n", (int)data[i]);
	}

	if (completion_done(&widev->line_recv_comp)) {
		dev_info(&sdev->dev, "RX waiting on completion\n");
		return 0;
	}
	if (widev->rx_len == sizeof(widev->rx_buf) - 1) {
		dev_warn(&sdev->dev, "RX buffer full\n");
		return 0;
	}

	i = min(count, sizeof(widev->rx_buf) - 1 - widev->rx_len);
	if (i > 0) {
		memcpy(&widev->rx_buf[widev->rx_len], data, i);
		widev->rx_len += i;
		len += i;
	}
	if (widev->rx_len >= 2 && strncmp(&widev->rx_buf[widev->rx_len - 2], "\r\n", 2) == 0) {
		widev->rx_len -= 2;
		widev->rx_buf[widev->rx_len] = '\0';
		complete(&widev->line_recv_comp);
	}

	return len;
}

static const struct serdev_device_ops widora_serdev_client_ops = {
	.receive_buf = widora_receive_buf,
};

static int widora_probe(struct serdev_device *sdev)
{
	struct widora_device *widev;
	char *sz;
	int ret;

	dev_info(&sdev->dev, "Probing\n");

	widev = devm_kzalloc(&sdev->dev, sizeof(struct widora_device), GFP_KERNEL);
	if (!widev)
		return -ENOMEM;

	widev->rst = devm_gpiod_get_optional(&sdev->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(widev->rst))
		return PTR_ERR(widev->rst);

	widora_reset_mcu(widev);

	widev->serdev = sdev;
	init_completion(&widev->line_recv_comp);
	serdev_device_set_drvdata(sdev, widev);

	ret = serdev_device_open(sdev);
	if (ret) {
		dev_err(&sdev->dev, "Failed to open (%d)\n", ret);
		goto err_serdev_open;
	}

	serdev_device_set_baudrate(sdev, 115200);
	serdev_device_set_flow_control(sdev, false);
	serdev_device_set_client_ops(sdev, &widora_serdev_client_ops);

	ret = widora_do_reset(widev, HZ);
	if (ret) {
		dev_err(&sdev->dev, "Failed to reset (%d)\n", ret);
		goto err_reset;
	}

	ret = widora_get_version(widev, &sz, HZ);
	if (ret) {
		dev_err(&sdev->dev, "Failed to get version (%d)\n", ret);
		goto err_version;
	}

	dev_info(&sdev->dev, "firmware version: %s\n", sz);
	kfree(sz);

	ret = widora_set_gpio_pb0(widev, true, HZ);
	if (ret) {
		dev_err(&sdev->dev, "Failed to set GPIO PB0 (%d)\n", ret);
		goto err_gpio;
	}

	ret = widora_set_gpio_pd0(widev, true, HZ);
	if (ret) {
		dev_err(&sdev->dev, "Failed to set GPIO PD0 (%d)\n", ret);
		goto err_gpio;
	}

	widev->netdev = alloc_loradev(sizeof(struct ting01m_priv));
	if (!widev->netdev) {
		ret = -ENOMEM;
		goto err_alloc_loradev;
	}

	widev->netdev->netdev_ops = &ting01m_net_device_ops;
	SET_NETDEV_DEV(widev->netdev, &sdev->dev);

	ret = register_loradev(widev->netdev);
	if (ret)
		goto err_register_loradev;

	dev_info(&sdev->dev, "Done.\n");

	return 0;

err_register_loradev:
	free_loradev(widev->netdev);
err_alloc_loradev:
err_gpio:
err_version:
err_reset:
	serdev_device_close(sdev);
err_serdev_open:
	gpiod_set_value_cansleep(widev->rst, 0);
	return ret;
}

static void widora_remove(struct serdev_device *sdev)
{
	struct widora_device *widev = serdev_device_get_drvdata(sdev);

	unregister_loradev(widev->netdev);
	free_loradev(widev->netdev);

	serdev_device_close(sdev);

	gpiod_set_value_cansleep(widev->rst, 0);

	dev_info(&sdev->dev, "Removed\n");
}

static const struct of_device_id widora_of_match[] = {
	{ .compatible = "himalaya,himo-01m" },
	{ .compatible = "widora,ting-01m" },
	{}
};
MODULE_DEVICE_TABLE(of, widora_of_match);

static struct serdev_device_driver widora_serdev_driver = {
	.probe = widora_probe,
	.remove = widora_remove,
	.driver = {
		.name = "ting-01m",
		.of_match_table = widora_of_match,
	},
};

static int __init widora_init(void)
{
	int ret;

	ret = serdev_device_driver_register(&widora_serdev_driver);
	if (ret)
		return ret;

	return 0;
}

static void __exit widora_exit(void)
{
	serdev_device_driver_unregister(&widora_serdev_driver);
}

module_init(widora_init);
module_exit(widora_exit);

MODULE_DESCRIPTION("Widora Ting-01M serdev driver");
MODULE_AUTHOR("Andreas Färber <afaerber@suse.de>");
MODULE_LICENSE("GPL");
