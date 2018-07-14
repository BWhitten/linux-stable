// SPDX-License-Identifier: GPL-2.0-or-later
/*
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

struct widora_device {
	struct serdev_device *serdev;

	struct gpio_desc *rst;

	char rx_buf[4096];
	int rx_len;

	struct completion line_recv_comp;
};

static int widora_send_command(struct widora_device *widev, const char *cmd, char **data, unsigned long timeout)
{
	struct serdev_device *sdev = widev->serdev;
	const char *crlf = "\r\n";
	char *resp;

	serdev_device_write_buf(sdev, cmd, strlen(cmd));
	serdev_device_write_buf(sdev, crlf, 2);

	timeout = wait_for_completion_timeout(&widev->line_recv_comp, timeout);
	if (!timeout)
		return -ETIMEDOUT;

	resp = widev->rx_buf;
	dev_dbg(&sdev->dev, "Received: '%s'\n", resp);
	if (data)
		*data = kstrdup(resp, GFP_KERNEL);

	widev->rx_len = 0;
	reinit_completion(&widev->line_recv_comp);

	return 0;
}

static int widora_simple_cmd(struct widora_device *widev, const char *cmd, unsigned long timeout)
{
	char *resp;
	int ret;

	ret = widora_send_command(widev, cmd, &resp, timeout);
	if (ret)
		return ret;

	if (strcmp(resp, "AT,OK") == 0) {
		kfree(resp);
		return 0;
	}

	kfree(resp);

	return -EINVAL;
}

static void widora_reset_mcu(struct widora_device *widev)
{
	gpiod_set_value_cansleep(widev->rst, 0);
	msleep(200);
	gpiod_set_value_cansleep(widev->rst, 1);
	msleep(500);
}

static int widora_do_reset(struct widora_device *widev, unsigned long timeout)
{
	char *resp;
	int ret;

	ret = widora_simple_cmd(widev, "AT+RST", timeout);
	if (ret)
		return ret;

	timeout = wait_for_completion_timeout(&widev->line_recv_comp, timeout);
	if (!timeout)
		return -ETIMEDOUT;

	resp = widev->rx_buf;

	dev_info(&widev->serdev->dev, "reset: '%s'\n", resp);

	widev->rx_len = 0;
	reinit_completion(&widev->line_recv_comp);

	return 0;
}

static int widora_get_version(struct widora_device *widev, char **version, unsigned long timeout)
{
	char *resp;
	int ret, len;

	ret = widora_send_command(widev, "AT+VER", &resp, timeout);
	if (ret)
		return ret;

	len = strlen(resp);

	if ((strncmp(resp, "AT,", 3) == 0) && (strncmp(resp + len - 3, ",OK", 3) == 0)) {
		*version = kstrndup(resp + 3, len - 3 - 3, GFP_KERNEL);
		kfree(resp);
		return 0;
	}

	kfree(resp);

	return -EINVAL;
}

static int widora_set_gpio(struct widora_device *widev, char bank, char pin, bool enabled, unsigned long timeout)
{
	char cmd[] = "AT+Pxx=x";

	cmd[4] = bank;
	cmd[5] = pin;
	cmd[7] = enabled ? '1' : '0';

	return widora_simple_cmd(widev, cmd, timeout);
}

static int widora_set_gpio_pb0(struct widora_device *widev, bool enabled, unsigned long timeout)
{
	return widora_set_gpio(widev, 'B', '0', enabled, timeout);
}

static int widora_set_gpio_pd0(struct widora_device *widev, bool enabled, unsigned long timeout)
{
	return widora_set_gpio(widev, 'D', '0', enabled, timeout);
}

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
		return ret;
	}

	serdev_device_set_baudrate(sdev, 115200);
	serdev_device_set_flow_control(sdev, false);
	serdev_device_set_client_ops(sdev, &widora_serdev_client_ops);

	ret = widora_do_reset(widev, HZ);
	if (ret) {
		dev_err(&sdev->dev, "Failed to reset (%d)\n", ret);
		serdev_device_close(sdev);
		return ret;
	}

	ret = widora_get_version(widev, &sz, HZ);
	if (ret) {
		dev_err(&sdev->dev, "Failed to get version (%d)\n", ret);
		serdev_device_close(sdev);
		return ret;
	}

	dev_info(&sdev->dev, "firmware version: %s\n", sz);
	kfree(sz);

	ret = widora_set_gpio_pb0(widev, true, HZ);
	if (ret) {
		dev_err(&sdev->dev, "Failed to set GPIO PB0 (%d)\n", ret);
		serdev_device_close(sdev);
		return ret;
	}

	ret = widora_set_gpio_pd0(widev, true, HZ);
	if (ret) {
		dev_err(&sdev->dev, "Failed to set GPIO PD0 (%d)\n", ret);
		serdev_device_close(sdev);
		return ret;
	}

	dev_info(&sdev->dev, "Done.\n");

	return 0;
}

static void widora_remove(struct serdev_device *sdev)
{
	serdev_device_close(sdev);

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
