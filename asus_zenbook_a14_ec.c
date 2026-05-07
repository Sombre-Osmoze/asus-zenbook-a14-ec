// SPDX-License-Identifier: GPL-2.0-only
/*
 * ASUS Zenbook A14 (UX3407RA) Embedded Controller driver — PoC
 *
 * Step 3: hwmon registration.
 *
 *   fan1_input   eccr(0x01, 0x09) × 88        RPM (calibrated)
 *   fan1_label   "fan"
 *   pwm1         eccr(0x01, 0x0a)             0-255 (RO in this step)
 *   pwm1_enable  eccr(0x01, 0x02) → mapped    1=manual, 2=auto (RO)
 *   temp1_input  eccr(0x05, 0x02) × 1000      m°C
 *   temp1_label  "ec"
 *
 * pwm1 / pwm1_enable are read-only here. Writing PWM (and switching
 * pwm1_enable to manual) is a Step 4 change once the temp-watchdog
 * kthread is in place — without it, the EC will hard-reset the system
 * if temp reporting stops.
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
 *  - eccr(a1, a2)          : compound read with atomic-exchange semantics
 *  - eccw(a1, a2, val)     : compound write
 *
 * Copyright (C) 2026 Sombre-Osmoze <sombre@osmoze.xyz>
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/hwmon.h>
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

/* EC protocol opcodes */
#define EC_OP_ADDR		0x10
#define EC_OP_DATA		0x11

/* Compound-op mailboxes at major 0xc4 */
#define EC_CC_BUSY		0x30
#define EC_CC_REGSEL		0x31
#define EC_CC_DATA		0x32

#define EC_SETTLE_INTERVAL_US	50000
#define EC_SETTLE_TIMEOUT_MS	2000

/* hwmon-exposed registers (validated during EC investigation) */
#define EC_REG_FAN_MODE_MAJ	0x01
#define EC_REG_FAN_MODE_MIN	0x02	/* 0=auto, 2=manual */
#define EC_REG_FAN_TACH_MAJ	0x01
#define EC_REG_FAN_TACH_MIN	0x09	/* RPM = value × 88 */
#define EC_REG_PWM_MAJ		0x01
#define EC_REG_PWM_MIN		0x0a	/* 0-255 */
#define EC_REG_TEMP_MAJ		0x05
#define EC_REG_TEMP_MIN		0x02	/* °C */

/* Tach-to-RPM conversion: empirically RPM ≈ tach × 88
 * (audio FFT calibration, system-snapshot.md).
 */
#define EC_TACH_RPM_MULT	88

/* Fan mode encoding observed in (0x01, 0x02). */
#define EC_FAN_MODE_AUTO	0
#define EC_FAN_MODE_MANUAL	2

struct asus_ec {
	struct device		*dev;
	struct i2c_adapter	*adapter;
	struct i2c_client	*ec_client;
	struct i2c_client	*fan_client;
	struct device		*hwmon_dev;
	struct mutex		lock;	/* serialises EC access */
	/* DMA-safe scratch (kmalloc-backed via devm_kzalloc) */
	u8			tx[3];
	u8			rx[1];
};

/* ------------------------------------------------------------------ */
/* EC primitives — caller MUST hold ec->lock                          */
/* ------------------------------------------------------------------ */

static int __ec_rb(struct asus_ec *ec, u8 maj, u8 min, u8 *out)
{
	struct i2c_msg msgs[3];
	static const u8 op_data = EC_OP_DATA;
	int ret;

	ec->tx[0] = EC_OP_ADDR;
	ec->tx[1] = maj;
	ec->tx[2] = min;

	msgs[0].addr  = EC_I2C_ADDR;
	msgs[0].flags = 0;
	msgs[0].len   = 3;
	msgs[0].buf   = ec->tx;

	msgs[1].addr  = EC_I2C_ADDR;
	msgs[1].flags = 0;
	msgs[1].len   = 1;
	msgs[1].buf   = (u8 *)&op_data;

	msgs[2].addr  = EC_I2C_ADDR;
	msgs[2].flags = I2C_M_RD;
	msgs[2].len   = 1;
	msgs[2].buf   = ec->rx;

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

static int __ec_cr(struct asus_ec *ec, u8 a1, u8 a2, u8 *out)
{
	int ret;
	u8 v;

	if (a2 >= 0x80) {
		dev_err(ec->dev,
			"eccr refused: a2=0x%02x >= 0x80 destructive\n", a2);
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

	ret = __ec_wb(ec, 0xc4, EC_CC_DATA, 0x00);
	if (ret)
		return ret;

	*out = v;
	return 0;
}

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

static int asus_ec_read_reg(struct asus_ec *ec, u8 maj, u8 min, u8 *out)
{
	int ret;

	mutex_lock(&ec->lock);
	ret = __ec_cr(ec, maj, min, out);
	mutex_unlock(&ec->lock);
	return ret;
}

/* ------------------------------------------------------------------ */
/* hwmon callbacks                                                    */
/* ------------------------------------------------------------------ */

static umode_t asus_ec_hwmon_is_visible(const void *drvdata,
					enum hwmon_sensor_types type,
					u32 attr, int channel)
{
	switch (type) {
	case hwmon_fan:
		switch (attr) {
		case hwmon_fan_input:
		case hwmon_fan_label:
			return 0444;
		default:
			return 0;
		}
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
		case hwmon_pwm_enable:
			return 0444;	/* RO in step 3, RW in step 4 */
		default:
			return 0;
		}
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
		case hwmon_temp_label:
			return 0444;
		default:
			return 0;
		}
	default:
		return 0;
	}
}

static int asus_ec_hwmon_read(struct device *dev,
			      enum hwmon_sensor_types type,
			      u32 attr, int channel, long *val)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	u8 v;
	int ret;

	switch (type) {
	case hwmon_fan:
		if (attr != hwmon_fan_input)
			return -EOPNOTSUPP;
		ret = asus_ec_read_reg(ec, EC_REG_FAN_TACH_MAJ,
				       EC_REG_FAN_TACH_MIN, &v);
		if (ret)
			return ret;
		*val = (long)v * EC_TACH_RPM_MULT;
		return 0;

	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
			ret = asus_ec_read_reg(ec, EC_REG_PWM_MAJ,
					       EC_REG_PWM_MIN, &v);
			if (ret)
				return ret;
			*val = v;
			return 0;
		case hwmon_pwm_enable:
			ret = asus_ec_read_reg(ec, EC_REG_FAN_MODE_MAJ,
					       EC_REG_FAN_MODE_MIN, &v);
			if (ret)
				return ret;
			/* hwmon convention: 1=manual, 2=auto (matches us) */
			if (v == EC_FAN_MODE_MANUAL)
				*val = 1;
			else if (v == EC_FAN_MODE_AUTO)
				*val = 2;
			else
				*val = 0;	/* unknown */
			return 0;
		default:
			return -EOPNOTSUPP;
		}

	case hwmon_temp:
		if (attr != hwmon_temp_input)
			return -EOPNOTSUPP;
		ret = asus_ec_read_reg(ec, EC_REG_TEMP_MAJ,
				       EC_REG_TEMP_MIN, &v);
		if (ret)
			return ret;
		*val = (long)v * 1000;	/* °C → m°C */
		return 0;

	default:
		return -EOPNOTSUPP;
	}
}

static int asus_ec_hwmon_read_string(struct device *dev,
				     enum hwmon_sensor_types type,
				     u32 attr, int channel,
				     const char **str)
{
	switch (type) {
	case hwmon_fan:
		if (attr == hwmon_fan_label) {
			*str = "fan";
			return 0;
		}
		break;
	case hwmon_temp:
		if (attr == hwmon_temp_label) {
			*str = "ec";
			return 0;
		}
		break;
	default:
		break;
	}
	return -EOPNOTSUPP;
}

static const struct hwmon_ops asus_ec_hwmon_ops = {
	.is_visible	= asus_ec_hwmon_is_visible,
	.read		= asus_ec_hwmon_read,
	.read_string	= asus_ec_hwmon_read_string,
};

static const struct hwmon_channel_info * const asus_ec_hwmon_info[] = {
	HWMON_CHANNEL_INFO(fan,
			   HWMON_F_INPUT | HWMON_F_LABEL),
	HWMON_CHANNEL_INFO(pwm,
			   HWMON_PWM_INPUT | HWMON_PWM_ENABLE),
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT | HWMON_T_LABEL),
	NULL
};

static const struct hwmon_chip_info asus_ec_hwmon_chip_info = {
	.ops	= &asus_ec_hwmon_ops,
	.info	= asus_ec_hwmon_info,
};

/* ------------------------------------------------------------------ */
/* Adapter discovery                                                  */
/* ------------------------------------------------------------------ */

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
	u8 tach, pwm, temp, mode;
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

	/* Probe-time sanity reads. */
	(void)asus_ec_read_reg(ec, EC_REG_FAN_TACH_MAJ,
			       EC_REG_FAN_TACH_MIN, &tach);
	(void)asus_ec_read_reg(ec, EC_REG_PWM_MAJ, EC_REG_PWM_MIN, &pwm);
	(void)asus_ec_read_reg(ec, EC_REG_TEMP_MAJ, EC_REG_TEMP_MIN, &temp);
	(void)asus_ec_read_reg(ec, EC_REG_FAN_MODE_MAJ,
			       EC_REG_FAN_MODE_MIN, &mode);

	dev_info(dev,
		 "online: tach=%u (~%u RPM) pwm=%u temp=%u°C mode=0x%02x\n",
		 tach, tach * EC_TACH_RPM_MULT, pwm, temp, mode);

	ec->hwmon_dev = devm_hwmon_device_register_with_info(dev,
				DRV_NAME, ec,
				&asus_ec_hwmon_chip_info, NULL);
	if (IS_ERR(ec->hwmon_dev)) {
		ret = PTR_ERR(ec->hwmon_dev);
		dev_err(dev, "hwmon registration failed: %d\n", ret);
		i2c_unregister_device(ec->fan_client);
		i2c_unregister_device(ec->ec_client);
		i2c_put_adapter(ec->adapter);
		return ret;
	}

	dev_info(dev, "hwmon registered\n");
	return 0;
}

static void asus_ec_remove(struct platform_device *pdev)
{
	struct asus_ec *ec = platform_get_drvdata(pdev);

	if (!ec)
		return;

	/* hwmon_dev is devm-managed and torn down automatically. */

	if (!IS_ERR_OR_NULL(ec->fan_client))
		i2c_unregister_device(ec->fan_client);
	if (!IS_ERR_OR_NULL(ec->ec_client))
		i2c_unregister_device(ec->ec_client);
	if (ec->adapter)
		i2c_put_adapter(ec->adapter);

	dev_info(&pdev->dev, "removed\n");
}

static struct platform_driver asus_ec_driver = {
	.driver	= {
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

MODULE_AUTHOR("Sombre-Osmoze <sombre@osmoze.xyz>");
MODULE_DESCRIPTION("ASUS Zenbook A14 (UX3407RA) Embedded Controller driver (PoC)");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
