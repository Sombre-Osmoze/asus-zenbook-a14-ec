// SPDX-License-Identifier: GPL-2.0-only
/*
 * ASUS Zenbook A14 (UX3407RA) Embedded Controller driver — PoC
 *
 * Step 1 scaffold: registers as a platform driver, finds the I²C
 * adapter behind "b94000.i2c", instantiates an i2c_client at the
 * EC address (0x5b), and exposes nothing yet. Verifies bind/unbind
 * and adapter discovery without touching the EC.
 *
 * Hardware details (see system-snapshot.md):
 *  - SoC: Qualcomm X1E80100
 *  - EC sits at I²C address 0x5b on platform device "b94000.i2c"
 *  - Companion fan controller sits at 0x76 on the same bus
 *
 * Copyright (C) 2026 osmoze
 */

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define DRV_NAME		"asus_zenbook_a14_ec"

#define EC_I2C_BUS_NAME		"b94000.i2c"
#define EC_I2C_ADDR		0x5b
#define FAN_I2C_ADDR		0x76

struct asus_ec {
	struct device		*dev;
	struct i2c_adapter	*adapter;
	struct i2c_client	*ec_client;
	struct i2c_client	*fan_client;
	struct mutex		lock;	/* serialises EC access */
};

/*
 * Locate the i2c adapter parented under the platform device named
 * EC_I2C_BUS_NAME. We don't have a DT child node for the EC (yet),
 * so we look up the platform device via its name, then resolve the
 * i2c_adapter via its OF node.
 */
static struct i2c_adapter *asus_ec_find_adapter(struct device *dev)
{
	struct device *plat_dev;
	struct device_node *np;
	struct i2c_adapter *adap;

	plat_dev = bus_find_device_by_name(&platform_bus_type, NULL,
					   EC_I2C_BUS_NAME);
	if (!plat_dev) {
		dev_err(dev, "platform device '%s' not found\n",
			EC_I2C_BUS_NAME);
		return NULL;
	}

	np = plat_dev->of_node;
	if (!np) {
		dev_err(dev, "'%s' has no OF node\n", EC_I2C_BUS_NAME);
		put_device(plat_dev);
		return NULL;
	}

	adap = of_find_i2c_adapter_by_node(np);
	put_device(plat_dev);

	if (!adap) {
		dev_dbg(dev, "i2c adapter for '%s' not yet registered\n",
			EC_I2C_BUS_NAME);
		return NULL;
	}

	return adap;
}

static int asus_ec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct asus_ec *ec;
	struct i2c_board_info ec_info = {
		I2C_BOARD_INFO("asus_zenbook_a14_ec", EC_I2C_ADDR),
	};
	struct i2c_board_info fan_info = {
		I2C_BOARD_INFO("asus_zenbook_a14_fan", FAN_I2C_ADDR),
	};

	ec = devm_kzalloc(dev, sizeof(*ec), GFP_KERNEL);
	if (!ec)
		return -ENOMEM;

	ec->dev = dev;
	mutex_init(&ec->lock);

	ec->adapter = asus_ec_find_adapter(dev);
	if (!ec->adapter)
		return -EPROBE_DEFER;

	ec->ec_client = i2c_new_client_device(ec->adapter, &ec_info);
	if (IS_ERR(ec->ec_client)) {
		dev_err(dev, "failed to register EC client at 0x%02x\n",
			EC_I2C_ADDR);
		i2c_put_adapter(ec->adapter);
		return PTR_ERR(ec->ec_client);
	}

	ec->fan_client = i2c_new_client_device(ec->adapter, &fan_info);
	if (IS_ERR(ec->fan_client)) {
		dev_err(dev, "failed to register FAN client at 0x%02x\n",
			FAN_I2C_ADDR);
		i2c_unregister_device(ec->ec_client);
		i2c_put_adapter(ec->adapter);
		return PTR_ERR(ec->fan_client);
	}

	platform_set_drvdata(pdev, ec);

	dev_info(dev,
		 "scaffold OK: adapter=%s, EC@0x%02x FAN@0x%02x (no I/O yet)\n",
		 ec->adapter->name, EC_I2C_ADDR, FAN_I2C_ADDR);

	return 0;
}

static void asus_ec_remove(struct platform_device *pdev)
{
	struct asus_ec *ec = platform_get_drvdata(pdev);

	if (!ec)
		return;

	if (!IS_ERR_OR_NULL(ec->fan_client))
		i2c_unregister_device(ec->fan_client);
	if (!IS_ERR_OR_NULL(ec->ec_client))
		i2c_unregister_device(ec->ec_client);
	if (ec->adapter)
		i2c_put_adapter(ec->adapter);

	dev_info(&pdev->dev, "removed\n");
}

static struct platform_driver asus_ec_driver = {
	.driver = {
		.name = DRV_NAME,
	},
	.probe	= asus_ec_probe,
	.remove	= asus_ec_remove,
};

/*
 * No DT match for now — we register a virtual platform device manually
 * at module init so the driver binds without any DT changes.
 */
static struct platform_device *asus_ec_pdev;

static int __init asus_ec_init(void)
{
	int ret;

	ret = platform_driver_register(&asus_ec_driver);
	if (ret)
		return ret;

	asus_ec_pdev = platform_device_register_simple(DRV_NAME, -1, NULL, 0);
	if (IS_ERR(asus_ec_pdev)) {
		platform_driver_unregister(&asus_ec_driver);
		return PTR_ERR(asus_ec_pdev);
	}

	return 0;
}

static void __exit asus_ec_exit(void)
{
	platform_device_unregister(asus_ec_pdev);
	platform_driver_unregister(&asus_ec_driver);
}

module_init(asus_ec_init);
module_exit(asus_ec_exit);

MODULE_AUTHOR("osmoze");
MODULE_DESCRIPTION("ASUS Zenbook A14 (UX3407RA) Embedded Controller driver (PoC)");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
