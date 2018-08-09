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

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/lora/sx130x.h>

#include "sx1301.h"

static int sx1301_regmap_bus_write(void *context, unsigned int reg,
		unsigned int val)
{
	struct sx130x_radio_device *rdev = context;
	struct net_device *netdev = dev_get_drvdata(rdev->concentrator);
	struct sx1301_priv *priv = netdev_priv(netdev);
	unsigned int addr, data, cs, rb;
	int ret;

	if (rdev->nr == 0) {
		addr = SX1301_RADIO_A_SPI_ADDR;
		data = SX1301_RADIO_A_SPI_DATA;
		cs = SX1301_RADIO_A_SPI_CS;
		rb = SX1301_RADIO_A_SPI_DATA_RB;
	} else {
		addr = SX1301_RADIO_B_SPI_ADDR;
		data = SX1301_RADIO_B_SPI_DATA;
		cs = SX1301_RADIO_B_SPI_CS;
		rb = SX1301_RADIO_B_SPI_DATA_RB;
	}

	ret = regmap_write(priv->regmap, cs, 0);
	if (ret)
		return ret;
	ret = regmap_write(priv->regmap, addr, reg);
	if (ret)
		return ret;
	ret = regmap_write(priv->regmap, data, val);
	if (ret)
		return ret;
	ret = regmap_write(priv->regmap, cs, 1);
	if (ret)
		return ret;
	ret = regmap_write(priv->regmap, cs, 0);
	if (ret)
		return ret;

	return 0;
}

static int sx1301_regmap_bus_read(void *context, unsigned int reg,
		unsigned int *val)
{
	struct sx130x_radio_device *rdev = context;
	struct net_device *netdev = dev_get_drvdata(rdev->concentrator);
	struct sx1301_priv *priv = netdev_priv(netdev);
	unsigned int addr, data, cs, rb;
	int ret;

	if (rdev->nr == 0) {
		addr = SX1301_RADIO_A_SPI_ADDR;
		data = SX1301_RADIO_A_SPI_DATA;
		cs = SX1301_RADIO_A_SPI_CS;
		rb = SX1301_RADIO_A_SPI_DATA_RB;
	} else {
		addr = SX1301_RADIO_B_SPI_ADDR;
		data = SX1301_RADIO_B_SPI_DATA;
		cs = SX1301_RADIO_B_SPI_CS;
		rb = SX1301_RADIO_B_SPI_DATA_RB;
	}

	ret = regmap_write(priv->regmap, cs, 0);
	if (ret)
		return ret;
	/* address to tx */
	ret = regmap_write(priv->regmap, addr, reg);
	if (ret)
		return ret;
	ret = regmap_write(priv->regmap, data, 0);
	if (ret)
		return ret;
	ret = regmap_write(priv->regmap, cs, 1);
	if (ret)
		return ret;
	ret = regmap_write(priv->regmap, cs, 0);
	if (ret)
		return ret;

	/* reading */
	return regmap_read(priv->regmap, rb, val);
}

static const struct regmap_bus sx1301_regmap_bus = {
	.reg_write = sx1301_regmap_bus_write,
	.reg_read = sx1301_regmap_bus_read,
};

static int sx130x_radio_match_device(struct device *dev, struct device_driver *drv)
{
	return of_driver_match_device(dev, drv);
}

static struct bus_type sx130x_radio_bus_type = {
	.name = "sx130x_radio",
	.match = sx130x_radio_match_device,
	.uevent = of_device_uevent_modalias,
};

int __init sx130x_radio_init(void)
{
	int ret;

	ret = bus_register(&sx130x_radio_bus_type);
	if (ret < 0)
		return ret;

	return 0;
}

void __exit sx130x_radio_exit(void)
{
	bus_unregister(&sx130x_radio_bus_type);
}

static int sx130x_radio_drv_probe(struct device *dev)
{
	const struct sx130x_radio_driver *rdrv = to_sx130x_radio_driver(dev->driver);
	int ret;

	ret = rdrv->probe(to_sx130x_radio_device(dev));

	return ret;
}

static int sx130x_radio_drv_remove(struct device *dev)
{
	const struct sx130x_radio_driver *rdrv = to_sx130x_radio_driver(dev->driver);
	int ret;

	ret = rdrv->remove(to_sx130x_radio_device(dev));

	return ret;
}

int __sx130x_register_radio_driver(struct module *owner, struct sx130x_radio_driver *rdrv)
{
	rdrv->driver.owner = owner;
	rdrv->driver.bus = &sx130x_radio_bus_type;

	if (rdrv->probe)
		rdrv->driver.probe = sx130x_radio_drv_probe;
	if (rdrv->remove)
		rdrv->driver.remove = sx130x_radio_drv_remove;

	return driver_register(&rdrv->driver);
}
EXPORT_SYMBOL_GPL(__sx130x_register_radio_driver);

static void sx130x_radio_release(struct device *dev)
{
	struct sx130x_radio_device *radio = to_sx130x_radio_device(dev);

	put_device(radio->concentrator);
	kfree(radio);
}

static struct sx130x_radio_device *sx130x_alloc_radio_device(struct device *dev)
{
	struct sx130x_radio_device *radio;

	if (!get_device(dev))
		return NULL;

	radio = kzalloc(sizeof(*radio), GFP_KERNEL);
	if (!radio) {
		put_device(dev);
		return NULL;
	}

	radio->dev.parent = dev;
	radio->dev.bus = &sx130x_radio_bus_type;
	radio->dev.release = sx130x_radio_release;

	radio->concentrator = dev;
	radio->regmap_bus = &sx1301_regmap_bus;

	device_initialize(&radio->dev);
	return radio;
}

static void sx130x_radio_dev_set_name(struct sx130x_radio_device *radio)
{
	dev_set_name(&radio->dev, "%s-%c", dev_name(radio->concentrator), 'a' + radio->nr);
}

static int sx130x_radio_dev_check(struct device *dev, void *data)
{
	struct sx130x_radio_device *radio = to_sx130x_radio_device(dev);
	struct sx130x_radio_device *new_radio = data;

	if (radio->concentrator == new_radio->concentrator &&
	    radio->nr == new_radio->nr)
		return -EBUSY;

	return 0;
}

static int sx130x_add_radio_device(struct sx130x_radio_device *radio)
{
	static DEFINE_MUTEX(sx130x_radio_add_lock);
	int ret;

	if (radio->nr >= 2)
		return -EINVAL;

	sx130x_radio_dev_set_name(radio);

	mutex_lock(&sx130x_radio_add_lock);

	ret = bus_for_each_dev(&sx130x_radio_bus_type, NULL, radio, sx130x_radio_dev_check);
	if (ret)
		goto done;

	ret = device_add(&radio->dev);
	if (ret < 0)
		dev_err(&radio->dev, "can't add %s (%d)\n", dev_name(&radio->dev), ret);
	else
		dev_dbg(&radio->dev, "added child %s\n", dev_name(&radio->dev));

done:
	mutex_unlock(&sx130x_radio_add_lock);

	return ret;
}

static int sx130x_radio_parse_dt(struct device *dev, struct sx130x_radio_device *radio, struct device_node *node)
{
	u32 value;
	int ret;

	ret = of_property_read_u32(node, "reg", &value);
	if (ret) {
		dev_err(dev, "%pOF has no valid reg property (%d)\n", node, ret);
		return ret;
	}
	radio->nr = value;

	return 0;
}

static struct sx130x_radio_device *sx130x_register_radio_device(struct device *dev, struct device_node *node)
{
	struct sx130x_radio_device *radio;
	int ret;

	radio = sx130x_alloc_radio_device(dev);
	if (!radio)
		return ERR_PTR(-ENOMEM);

	ret = sx130x_radio_parse_dt(dev, radio, node);
	if (ret) {
		sx130x_radio_put(radio);
		return ERR_PTR(ret);
	}

	of_node_get(node);
	radio->dev.of_node = node;

	ret = sx130x_add_radio_device(radio);
	if (ret) {
		of_node_put(node);
		sx130x_radio_put(radio);
		return ERR_PTR(ret);
	}

	return radio;
}

static void sx130x_unregister_radio_device(struct sx130x_radio_device *radio)
{
	if (!radio)
		return;

	if (radio->dev.of_node) {
		of_node_clear_flag(radio->dev.of_node, OF_POPULATED);
		of_node_put(radio->dev.of_node);
	}
	device_unregister(&radio->dev);
}

int sx130x_register_radio_devices(struct device *dev)
{
	struct device_node *spi, *node;
	struct sx130x_radio_device *radio;
	unsigned int found = 0;

	spi = of_get_child_by_name(dev->of_node, "radio-spi");
	if (IS_ERR(spi))
		return PTR_ERR(spi);

	for_each_available_child_of_node(spi, node) {
		if (of_node_test_and_set_flag(node, OF_POPULATED))
			continue;

		radio = sx130x_register_radio_device(dev, node);
		if (IS_ERR(radio)) {
			dev_warn(dev, "failed to create radio device for %pOF\n", node);
			of_node_clear_flag(node, OF_POPULATED);
			return PTR_ERR(radio);
		}
		found++;
	}

	if (found < 2) {
		dev_err(dev, "found %u radio devices, expected 2\n", found);
		return -EINVAL;
	}

	return 0;
}

static int __sx130x_check_radio_device(struct device *dev, void *data)
{
	struct device *host = data;

	if (dev->bus == &sx130x_radio_bus_type && device_attach(dev) != 1) {
		dev_err(host, "radio %s not attached to driver\n", dev_name(dev));
		return -EBUSY;
	}

	return 0;
}

bool sx130x_radio_devices_okay(struct device *dev)
{
	int ret;

	ret = device_for_each_child(dev, dev, __sx130x_check_radio_device);
	if (ret)
		return false;

	return true;
}

static int __sx130x_unregister_radio_device(struct device *dev, void *data)
{
	sx130x_unregister_radio_device(to_sx130x_radio_device(dev));

	return 0;
}

void sx130x_unregister_radio_devices(struct device *dev)
{
	device_for_each_child(dev, NULL, __sx130x_unregister_radio_device);
}

static void devm_sx130x_unregister_radio_devices(struct device *dev, void *res)
{
	struct device **ptr = res;

	sx130x_unregister_radio_devices(*ptr);
}

int devm_sx130x_register_radio_devices(struct device *dev)
{
	struct device **ptr;
	int ret;

	ptr = devres_alloc(devm_sx130x_unregister_radio_devices, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return -ENOMEM;

	ret = sx130x_register_radio_devices(dev);
	if (ret) {
		devres_free(ptr);
		return ret;
	}

	*ptr = dev;
	devres_add(dev, ptr);

	return 0;
}
