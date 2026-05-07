// SPDX-License-Identifier: GPL-2.0-only
/*
 * ASUS Zenbook A14 (UX3407RA) Embedded Controller driver — PoC
 *
 * Step 2: EC protocol layer (mutex-protected) + one read-only debug
 * sysfs attribute (`ec_battery`) that reads EC register (0x03, 0x01)
 * to cross-check against /sys/class/power_supply/qcom-battmgr-bat/capacity.
 *
 * Hardware details (see system-snapshot.md):
 *  - SoC: Qualcomm X1E80100
 *  - EC sits at I²C address 0x5b on platform device "b94000.i2c"
 *  - Companion fan controller sits at 0x76 on the same bus
 *
 * EC protocol (mirrors x1e-ec-tool/tool.py):
 *  - ecrb(maj, min)        : raw register read (1 byte)
 *  - ecwb(maj, min, val)   : raw register write (1 byte)
 *  - ec_settle()           : wait for register (0xc4, 0x30) == 0
 *  - eccr(a1, a2)          : compound read (atomic-exchange semantics:
 *                             returns previous (0xc4, 0x32),
 *                             writes (0xc4, 0x32) into target a1/a2)
 *  - eccw(a1, a2, val)     : compound write
 *
 * Copyright (C) 2026 osmoze
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>

#define DRV_NAME		"asus_zenbook_a14_ec"

#define EC_I2C_BUS_NAME		"b94000.i2c"
#define EC_I2C_ADDR		0x5b
#define FAN_I2C_ADDR		0x76

/* EC protocol opcodes */
#define EC_OP_ADDR		0x10	/* prefix: write [0x10, maj, min]	*/
#define EC_OP_DATA		0x11	/* prefix: write [0x11(, val)] / read 1	*/

/* Compound-op mailboxes at major 0xc4 */
#define EC_CC_BUSY		0x30	/* (0xc4, 0x30): poll == 0 to settle	*/
#define EC_CC_REGSEL		0x31	/* (0xc4, 0x31): a2 mailbox		*/
#define EC_CC_DATA		0x32	/* (0xc4, 0x32): payload mailbox	*/

/* Settle loop: poll (0xc4, 0x30) until 0; ~50ms / iter, 2s cap. */
#define EC_SETTLE_INTERVAL_US	50000
#define EC_SETTLE_TIMEOUT_MS	2000

/* Probe register: battery percent. Safe to read; matches power_supply.	*/
#define EC_REG_BATTERY_MAJ	0x03
#define EC_REG_BATTERY_MIN	0x01

struct asus_ec {
	struct device		*dev;
	struct i2c_adapter	*adapter;
	struct i2c_client	*ec_client;
	struct i2c_client	*fan_client;
	struct mutex		lock;	/* serialises EC access */
	/* DMA-safe scratch (kmalloc-backed via devm_kzalloc) */
	u8			tx[3];
	u8			rx[1];
};

/* ------------------------------------------------------------------ */
/* EC primitives — caller MUST hold ec->lock                          */
/* ------------------------------------------------------------------ */

/*
 * Raw EC register read: write [0x10, maj, min], then write [0x11], then
 * read 1 byte. Three i2c messages in one transaction.
 */
static int __ec_rb(struct asus_ec *ec, u8 maj, u8 min, u8 *out)
{
	struct i2c_msg msgs[3];
	int ret;

	ec->tx[0] = EC_OP_ADDR;
	ec->tx[1] = maj;
	ec->tx[2] = min;

	msgs[0].addr	= EC_I2C_ADDR;
	msgs[0].flags	= 0;
	msgs[0].len	= 3;
	msgs[0].buf	= ec->tx;

	/*
	 * Second message reuses the next byte of tx as a 1-byte write.
	 * Stash 0x11 separately so it doesn't clash with msg[0]'s buffer.
	 */
	{
		static const u8 op_data = EC_OP_DATA;

		msgs[1].addr  = EC_I2C_ADDR;
		msgs[1].flags = 0;
		msgs[1].len   = 1;
		msgs[1].buf   = (u8 *)&op_data;
	}

	msgs[2].addr	= EC_I2C_ADDR;
	msgs[2].flags	= I2C_M_RD;
	msgs[2].len	= 1;
	msgs[2].buf	= ec->rx;

	ret = i2c_transfer(ec->adapter, msgs, 3);
	if (ret < 0) {
		dev_dbg(ec->dev, "ecrb(0x%02x,0x%02x) failed: %d\n",
			maj, min, ret);
		return ret;
	}
	if (ret != 3)
		return -EIO;

	*out = ec->rx[0];
	return 0;
}

/*
 * Raw EC register write: write [0x10, maj, min], then write [0x11, val].
 * Two i2c messages in one transaction.
 */
static int __ec_wb(struct asus_ec *ec, u8 maj, u8 min, u8 val)
{
	struct i2c_msg msgs[2];
	u8 data[2];
	int ret;

	ec->tx[0] = EC_OP_ADDR;
	ec->tx[1] = maj;
	ec->tx[2] = min;

	data[0] = EC_OP_DATA;
	data[1] = val;

	msgs[0].addr  = EC_I2C_ADDR;
	msgs[0].flags = 0;
	msgs[0].len   = 3;
	msgs[0].buf   = ec->tx;

	msgs[1].addr  = EC_I2C_ADDR;
	msgs[1].flags = 0;
	msgs[1].len   = 2;
	msgs[1].buf   = data;

	ret = i2c_transfer(ec->adapter, msgs, 2);
	if (ret < 0) {
		dev_dbg(ec->dev, "ecwb(0x%02x,0x%02x,0x%02x) failed: %d\n",
			maj, min, val, ret);
		return ret;
	}
	if (ret != 2)
		return -EIO;
	return 0;
}

/* Poll (0xc4, 0x30) until it reads 0, with a hard timeout. */
static int __ec_settle(struct asus_ec *ec)
{
	unsigned long deadline;
	u8 v;
	int ret;

	deadline = jiffies + msecs_to_jiffies(EC_SETTLE_TIMEOUT_MS);

	for (;;) {
		ret = __ec_rb(ec, 0xc4, EC_CC_BUSY, &v);
		if (ret)
			return ret;
		if (v == 0)
			return 0;
		if (time_after(jiffies, deadline)) {
			dev_warn(ec->dev,
				 "ec_settle timeout (last busy=0x%02x)\n", v);
			return -ETIMEDOUT;
		}
		usleep_range(EC_SETTLE_INTERVAL_US,
			     EC_SETTLE_INTERVAL_US + 10000);
	}
}

/*
 * Compound read: returns the previous value of (0xc4, 0x32) AND writes
 * (0xc4, 0x32) into the target register a1/a2. The atomic-exchange
 * behaviour was confirmed during EC investigation (system-snapshot.md).
 *
 * For our purposes (reading a register without side-effects) the target
 * a1/a2 must be benign: a2 < 0x80 is safe; a2 >= 0x80 is destructive.
 */
static int __ec_cr(struct asus_ec *ec, u8 a1, u8 a2, u8 *out)
{
	int ret;
	u8 v;

	if (a2 >= 0x80) {
		dev_err(ec->dev, "eccr refused: a2=0x%02x >= 0x80 destructive\n",
			a2);
		return -EINVAL;
	}

	ret = __ec_settle(ec);
	if (ret)
		return ret;

	ret = __ec_wb(ec, 0xc4, EC_CC_REGSEL, a2);
	if (ret)
		return ret;
	ret = __ec_wb(ec, 0xc4, EC_CC_BUSY, a1);
	if (ret)
		return ret;

	ret = __ec_settle(ec);
	if (ret)
		return ret;

	ret = __ec_rb(ec, 0xc4, EC_CC_DATA, &v);
	if (ret)
		return ret;

	/* Clear the data mailbox so the next eccr returns a known value. */
	ret = __ec_wb(ec, 0xc4, EC_CC_DATA, 0x00);
	if (ret)
		return ret;

	*out = v;
	return 0;
}

/*
 * Compound write. Currently unused by Step 2 but kept available so
 * Step 3 (hwmon pwm1, pwm1_enable) can use it without re-touching this
 * file.
 */
static int __maybe_unused __ec_cw(struct asus_ec *ec, u8 a1, u8 a2, u8 val)
{
	int ret;

	ret = __ec_settle(ec);
	if (ret)
		return ret;

	ret = __ec_wb(ec, 0xc4, EC_CC_REGSEL, a2);
	if (ret)
		return ret;
	ret = __ec_wb(ec, 0xc4, EC_CC_DATA, val);
	if (ret)
		return ret;
	ret = __ec_wb(ec, 0xc4, EC_CC_BUSY, a1);
	if (ret)
		return ret;

	return __ec_settle(ec);
}

/* ------------------------------------------------------------------ */
/* Public, locked wrappers                                            */
/* ------------------------------------------------------------------ */

static int asus_ec_read_reg(struct asus_ec *ec, u8 maj, u8 min, u8 *out)
{
	int ret;

	mutex_lock(&ec->lock);
	ret = __ec_cr(ec, maj, min, out);
	mutex_unlock(&ec->lock);
	return ret;
}

/* ------------------------------------------------------------------ */
/* sysfs: ec_battery (debug, root-only read)                          */
/* ------------------------------------------------------------------ */

static ssize_t ec_battery_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	u8 v;
	int ret;

	ret = asus_ec_read_reg(ec, EC_REG_BATTERY_MAJ, EC_REG_BATTERY_MIN, &v);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%u\n", v);
}
static DEVICE_ATTR(ec_battery, 0400, ec_battery_show, NULL);

static struct attribute *asus_ec_attrs[] = {
	&dev_attr_ec_battery.attr,
	NULL,
};
ATTRIBUTE_GROUPS(asus_ec);

/* ------------------------------------------------------------------ */
/* Adapter discovery                                                  */
/* ------------------------------------------------------------------ */

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

/* ------------------------------------------------------------------ */
/* probe / remove                                                     */
/* ------------------------------------------------------------------ */

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
	u8 batt;
	int ret;

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

	/* One probe read so dmesg shows we can talk to the EC. */
	ret = asus_ec_read_reg(ec, EC_REG_BATTERY_MAJ,
			       EC_REG_BATTERY_MIN, &batt);
	if (ret) {
		dev_warn(dev, "EC probe read failed: %d\n", ret);
	} else {
		dev_info(dev,
			 "online: adapter=%s EC@0x%02x FAN@0x%02x battery=%u%%\n",
			 ec->adapter->name, EC_I2C_ADDR, FAN_I2C_ADDR, batt);
	}

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
		.name		= DRV_NAME,
		.dev_groups	= asus_ec_groups,
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

MODULE_AUTHOR("Sombre-Osmoze <sombre@osmoze.xyz>");
MODULE_DESCRIPTION("ASUS Zenbook A14 (UX3407RA) Embedded Controller driver (PoC)");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
