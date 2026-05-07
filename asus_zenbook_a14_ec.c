// SPDX-License-Identifier: GPL-2.0-only
/*
 * ASUS Zenbook A14 (UX3407RA) Embedded Controller driver — PoC
 *
 * Step 5: platform_profile (low-power / balanced / performance).
 *
 *   fan1_input   eccr(0x01, 0x09) × 88        RPM (calibrated)
 *   fan1_label   "fan"
 *   pwm1         eccr(0x01, 0x0a)             0-255       (RW)
 *   pwm1_enable  eccr(0x01, 0x02) → mapped    1=manual, 2=auto (RW)
 *   temp1_input  eccr(0x05, 0x02) × 1000      m°C
 *   temp1_label  "ec"
 *
 * Manual mode is gated by the watchdog kthread: when userspace selects
 * pwm1_enable=1, the driver starts a 2s loop that reads the max SoC
 * thermal-zone temperature and pushes it to the fan controller via
 * Request(0x76).write(0x20,0x01,0x02,lo,hi). If this stream stops for
 * more than ~2 minutes while the EC is in manual mode, the EC will
 * hard-reset the system. The driver therefore:
 *   - kicks one temp send synchronously BEFORE switching to manual
 *   - tears manual mode down (back to auto) BEFORE stopping the kthread
 *   - forces auto mode in remove() and on suspend
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
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/thermal.h>

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
#define EC_REG_FAN_MODE_RMIN	0x02	/* read:  0=auto, 2=manual */
#define EC_REG_FAN_MODE_WMIN	0x82	/* write: 0=auto, 2=manual */
#define EC_REG_FAN_TACH_MAJ	0x01
#define EC_REG_FAN_TACH_MIN	0x09	/* RPM = value × 88 */
#define EC_REG_PWM_MAJ		0x01
#define EC_REG_PWM_RMIN		0x0a	/* read:  0-255 */
#define EC_REG_PWM_WMIN		0x8a	/* write: 0-255 */
#define EC_REG_FAN_SEL_WMIN	0x8c	/* fan-id selector for PWM write */
#define EC_REG_TEMP_MAJ		0x05
#define EC_REG_TEMP_MIN		0x02	/* °C */

/* Tach-to-RPM conversion: empirically RPM ≈ tach × 88
 * (audio FFT calibration, system-snapshot.md).
 */
#define EC_TACH_RPM_MULT	88

/* PWM floor below which the fan does not spin (calibration). */
#define EC_PWM_SPIN_FLOOR	0x4b	/* 75 */

/* Fan mode encoding observed in (0x01, 0x02). */
#define EC_FAN_MODE_AUTO	0
#define EC_FAN_MODE_MANUAL	2

/* Fan-controller (0x76) opcodes (mirrors x1e-ec-tool/tool.py) */
#define FAN_OP_PUSH_TEMP	0x20	/* [0x20, 0x01, 0x02, lo, hi] */
#define FAN_OP_SUSPEND		0x23	/* [0x23, mode] */
#define FAN_OP_PROFILE		0x24	/* [0x24, profile_idx]; Vivobook proto */

/* Profile readback hypothesis: eccr(0x01, 0x0b) returns current idx
 * (0..3). Validated empirically on Zenbook A14 by writing via 0x24
 * and observing the readback. See system-snapshot.md.
 */
#define EC_REG_PROFILE_MAJ	0x01
#define EC_REG_PROFILE_RMIN	0x0b

/* EC profile indices (matches MODELS["ASUS Vivobook S 15"].profiles) */
#define EC_PROFILE_WHISPER	0
#define EC_PROFILE_STANDARD	1
#define EC_PROFILE_PERFORMANCE	2
#define EC_PROFILE_FULL_SPEED	3
#define EC_PROFILE_MAX		3

/* Watchdog: must be well below the EC's ~2 min timeout. */
#define WATCHDOG_PERIOD_MS	2000

/* Thermal zones to feed to the EC (max of). */
#define ASUS_EC_MAX_ZONES	4
static const char * const asus_ec_thermal_zones[] = {
	"cpu0-0-top-thermal",
	"cpu1-0-top-thermal",
	"cpu2-0-top-thermal",
	"gpuss-0-thermal",
};

struct asus_ec {
	struct device		*dev;
	struct i2c_adapter	*adapter;
	struct i2c_client	*ec_client;
	struct i2c_client	*fan_client;
	struct device		*hwmon_dev;
	struct mutex		lock;	/* serialises EC compound access */

	/* Watchdog state */
	struct task_struct	*watchdog_task;
	struct mutex		mode_lock;	/* gates pwm_enable transitions */
	bool			manual_active;
	struct thermal_zone_device *zones[ASUS_EC_MAX_ZONES];
	int			n_zones;

	/* Profile state. ACPI's platform_profile framework requires
	 * acpi_kobj which doesn't exist on DT-only ARM64 systems, so we
	 * expose a compatible interface as our own sysfs group on the
	 * platform device. Userspace tooling can bridge to PPD later.
	 */
	u8			profile_cached;	/* last value we wrote */

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

static int __ec_cw(struct asus_ec *ec, u8 a1, u8 a2, u8 val)
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

static int asus_ec_write_reg(struct asus_ec *ec, u8 maj, u8 min, u8 val)
{
	int ret;

	mutex_lock(&ec->lock);
	ret = __ec_cw(ec, maj, min, val);
	mutex_unlock(&ec->lock);
	return ret;
}

/* High-level EC operations */

static int asus_ec_set_fan_mode(struct asus_ec *ec, u8 mode)
{
	return asus_ec_write_reg(ec, EC_REG_FAN_MODE_MAJ,
				 EC_REG_FAN_MODE_WMIN, mode);
}

static int asus_ec_set_pwm(struct asus_ec *ec, u8 speed)
{
	int ret;

	mutex_lock(&ec->lock);
	ret = __ec_cw(ec, EC_REG_PWM_MAJ, EC_REG_FAN_SEL_WMIN, 0);
	if (ret)
		goto out;
	ret = __ec_cw(ec, EC_REG_PWM_MAJ, EC_REG_PWM_WMIN, speed);
out:
	mutex_unlock(&ec->lock);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Fan-controller (0x76) direct writes                                */
/* ------------------------------------------------------------------ */

static int fan_send_temp_dc(struct asus_ec *ec, u16 deci_celsius)
{
	u8 buf[5];
	int ret;

	if (deci_celsius > 2000)
		deci_celsius = 2000;

	buf[0] = FAN_OP_PUSH_TEMP;
	buf[1] = 0x01;
	buf[2] = 0x02;
	buf[3] = deci_celsius & 0xff;
	buf[4] = (deci_celsius >> 8) & 0xff;

	ret = i2c_master_send(ec->fan_client, buf, sizeof(buf));
	return ret < 0 ? ret : 0;
}

static int fan_set_suspend(struct asus_ec *ec, u8 mode)
{
	u8 buf[2] = { FAN_OP_SUSPEND, mode };
	int ret;

	ret = i2c_master_send(ec->fan_client, buf, sizeof(buf));
	return ret < 0 ? ret : 0;
}

static int fan_set_profile(struct asus_ec *ec, u8 profile)
{
	u8 buf[2] = { FAN_OP_PROFILE, profile };
	int ret;

	if (profile > EC_PROFILE_MAX)
		return -EINVAL;

	ret = i2c_master_send(ec->fan_client, buf, sizeof(buf));
	return ret < 0 ? ret : 0;
}

/* ------------------------------------------------------------------ */
/* Profile sysfs (custom; mirrors platform_profile string semantics)  */
/* ------------------------------------------------------------------ */

/*
 * Two protocol hypotheses for setting the profile on the X1E80100 EC:
 *   (a) tool.py / Vivobook: Request(0x76).write(0x24, idx)   ← used here
 *   (b) +0x80 convention:    eccw(0x01, 0x8b, idx)            ← unverified
 *
 * (a) is the documented vendor approach and is known to work on the
 * sibling Vivobook S 15. (b) is hypothesised because eccr(0x01, 0x0b)
 * returns 0x02 by default on the A14, matching "Performance" idx 2.
 *
 * We pick (a) and *verify* it by reading back 0x0b after every set:
 * if the readback tracks the written value, both hypotheses are
 * consistent with each other (probably (b) is just the underlying
 * register write that 0x24 performs).
 *
 * The kernel's platform_profile framework would normally own the
 * /sys/firmware/acpi/platform_profile sysfs node, but it requires
 * ACPI (acpi_disabled check + acpi_kobj as sysfs root). This box
 * is DT-only, so we expose the same string vocabulary as a custom
 * sysfs group on our platform device:
 *
 *   /sys/devices/platform/asus_zenbook_a14_ec/profile
 *   /sys/devices/platform/asus_zenbook_a14_ec/profile_choices
 *
 * Once a non-ACPI platform_profile path lands upstream (or once we
 * patch it ourselves), this whole block becomes a thin wrapper
 * around devm_platform_profile_register().
 */

static const char * const profile_names[] = {
	[EC_PROFILE_WHISPER]	 = "quiet",
	[EC_PROFILE_STANDARD]	 = "balanced",
	[EC_PROFILE_PERFORMANCE] = "balanced-performance",
	[EC_PROFILE_FULL_SPEED]	 = "performance",
};

static int profile_name_to_idx(const char *name, size_t len)
{
	int i;

	/* Strip trailing whitespace/newline. */
	while (len && (name[len - 1] == '\n' || name[len - 1] == ' ' ||
		       name[len - 1] == '\t'))
		len--;

	/* Allow numeric "0".."3" too, for scripts. */
	if (len == 1 && name[0] >= '0' && name[0] <= '3')
		return name[0] - '0';

	for (i = 0; i < ARRAY_SIZE(profile_names); i++) {
		if (strlen(profile_names[i]) == len &&
		    !strncmp(profile_names[i], name, len))
			return i;
	}
	return -EINVAL;
}

static int asus_ec_profile_apply(struct asus_ec *ec, u8 idx)
{
	u8 readback;
	int ret;

	if (idx > EC_PROFILE_MAX)
		return -EINVAL;

	ret = fan_set_profile(ec, idx);
	if (ret) {
		dev_err(ec->dev, "profile_set(%u) failed: %d\n", idx, ret);
		return ret;
	}
	ec->profile_cached = idx;

	/* Verify the (0x01, 0x0b) readback hypothesis. Best-effort. */
	if (!asus_ec_read_reg(ec, EC_REG_PROFILE_MAJ,
			      EC_REG_PROFILE_RMIN, &readback)) {
		if (readback != idx)
			dev_dbg(ec->dev,
				"profile readback (0x%02x) != written (0x%02x)\n",
				readback, idx);
	}

	dev_dbg(ec->dev, "profile set to %u (%s)\n",
		idx, profile_names[idx]);
	return 0;
}

static ssize_t profile_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	u8 v;

	if (asus_ec_read_reg(ec, EC_REG_PROFILE_MAJ,
			     EC_REG_PROFILE_RMIN, &v) || v > EC_PROFILE_MAX)
		v = ec->profile_cached;

	return sysfs_emit(buf, "%s\n", profile_names[v]);
}

static ssize_t profile_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	int idx = profile_name_to_idx(buf, count);
	int ret;

	if (idx < 0)
		return idx;

	ret = asus_ec_profile_apply(ec, (u8)idx);
	return ret ? ret : count;
}

static ssize_t profile_choices_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	return sysfs_emit(buf, "%s %s %s %s\n",
			  profile_names[EC_PROFILE_WHISPER],
			  profile_names[EC_PROFILE_STANDARD],
			  profile_names[EC_PROFILE_PERFORMANCE],
			  profile_names[EC_PROFILE_FULL_SPEED]);
}

static DEVICE_ATTR_RW(profile);
static DEVICE_ATTR_RO(profile_choices);

static struct attribute *asus_ec_profile_attrs[] = {
	&dev_attr_profile.attr,
	&dev_attr_profile_choices.attr,
	NULL,
};

static const struct attribute_group asus_ec_profile_group = {
	.attrs = asus_ec_profile_attrs,
};

/* ------------------------------------------------------------------ */
/* Thermal zone bookkeeping + watchdog                                */
/* ------------------------------------------------------------------ */

static void asus_ec_lookup_thermal_zones(struct asus_ec *ec)
{
	int i;

	ec->n_zones = 0;
	for (i = 0; i < ARRAY_SIZE(asus_ec_thermal_zones); i++) {
		struct thermal_zone_device *tz;

		tz = thermal_zone_get_zone_by_name(asus_ec_thermal_zones[i]);
		if (IS_ERR(tz)) {
			dev_warn(ec->dev, "thermal zone '%s' not found: %ld\n",
				 asus_ec_thermal_zones[i], PTR_ERR(tz));
			continue;
		}
		ec->zones[ec->n_zones++] = tz;
		dev_dbg(ec->dev, "thermal zone '%s' attached\n",
			asus_ec_thermal_zones[i]);
	}

	if (ec->n_zones == 0)
		dev_warn(ec->dev,
			 "no thermal zones available; manual mode will fall back to EC temp\n");
}

/* Returns max temp in m°C across registered zones, or -1 on total failure. */
static int asus_ec_max_temp_mc(struct asus_ec *ec)
{
	int max = -1;
	int i;

	for (i = 0; i < ec->n_zones; i++) {
		int t;

		if (thermal_zone_get_temp(ec->zones[i], &t))
			continue;
		if (t > max)
			max = t;
	}

	if (max < 0) {
		/* Fallback: EC's own temp register, in °C → m°C. */
		u8 v;

		if (!asus_ec_read_reg(ec, EC_REG_TEMP_MAJ,
				      EC_REG_TEMP_MIN, &v))
			max = (int)v * 1000;
	}

	return max;
}

static int asus_ec_send_current_temp(struct asus_ec *ec)
{
	int mc = asus_ec_max_temp_mc(ec);
	u16 dc;

	if (mc < 0)
		return -ENODATA;

	/* m°C → deci-°C. Clamp to 2000 (200 °C) — guaranteed cold otherwise. */
	dc = (u16)clamp(mc / 100, 0, 2000);
	return fan_send_temp_dc(ec, dc);
}

static int asus_ec_watchdog_fn(void *data)
{
	struct asus_ec *ec = data;

	dev_info(ec->dev, "watchdog: started (period %d ms)\n",
		 WATCHDOG_PERIOD_MS);

	while (!kthread_should_stop()) {
		int ret = asus_ec_send_current_temp(ec);

		if (ret)
			dev_warn_ratelimited(ec->dev,
					     "watchdog: temp send failed: %d\n",
					     ret);

		set_current_state(TASK_INTERRUPTIBLE);
		if (kthread_should_stop()) {
			__set_current_state(TASK_RUNNING);
			break;
		}
		schedule_timeout(msecs_to_jiffies(WATCHDOG_PERIOD_MS));
	}

	dev_info(ec->dev, "watchdog: stopped\n");
	return 0;
}

/* Caller MUST hold ec->mode_lock */
static int asus_ec_start_watchdog(struct asus_ec *ec)
{
	struct task_struct *t;
	int ret;

	if (ec->watchdog_task)
		return 0;

	/* Send one temp synchronously so the EC's watchdog starts fresh. */
	ret = asus_ec_send_current_temp(ec);
	if (ret) {
		dev_err(ec->dev,
			"watchdog: refusing to start, initial temp send failed: %d\n",
			ret);
		return ret;
	}

	t = kthread_run(asus_ec_watchdog_fn, ec, "asus_ec_wdt");
	if (IS_ERR(t))
		return PTR_ERR(t);

	ec->watchdog_task = t;
	return 0;
}

/* Caller MUST hold ec->mode_lock */
static void asus_ec_stop_watchdog(struct asus_ec *ec)
{
	if (!ec->watchdog_task)
		return;

	kthread_stop(ec->watchdog_task);
	ec->watchdog_task = NULL;
}

/* Caller MUST hold ec->mode_lock. Order matters for safety. */
static int asus_ec_enter_manual(struct asus_ec *ec)
{
	int ret;

	ret = asus_ec_start_watchdog(ec);
	if (ret)
		return ret;

	ret = asus_ec_set_fan_mode(ec, EC_FAN_MODE_MANUAL);
	if (ret) {
		asus_ec_stop_watchdog(ec);
		return ret;
	}

	ec->manual_active = true;
	return 0;
}

/* Caller MUST hold ec->mode_lock. */
static int asus_ec_leave_manual(struct asus_ec *ec)
{
	int ret;

	/* Always tell EC auto FIRST; only then is it safe to stop feeding. */
	ret = asus_ec_set_fan_mode(ec, EC_FAN_MODE_AUTO);
	if (ret) {
		dev_err(ec->dev,
			"failed to restore auto fan mode: %d (watchdog kept alive)\n",
			ret);
		return ret;
	}

	asus_ec_stop_watchdog(ec);
	ec->manual_active = false;
	return 0;
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
			return 0644;
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
					       EC_REG_PWM_RMIN, &v);
			if (ret)
				return ret;
			*val = v;
			return 0;
		case hwmon_pwm_enable:
			ret = asus_ec_read_reg(ec, EC_REG_FAN_MODE_MAJ,
					       EC_REG_FAN_MODE_RMIN, &v);
			if (ret)
				return ret;
			/* hwmon convention: 1=manual, 2=auto */
			if (v == EC_FAN_MODE_MANUAL)
				*val = 1;
			else if (v == EC_FAN_MODE_AUTO)
				*val = 2;
			else
				*val = 0;
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
		*val = (long)v * 1000;
		return 0;

	default:
		return -EOPNOTSUPP;
	}
}

static int asus_ec_hwmon_write(struct device *dev,
			       enum hwmon_sensor_types type,
			       u32 attr, int channel, long val)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	int ret;
	u8 speed;

	if (type != hwmon_pwm)
		return -EOPNOTSUPP;

	switch (attr) {
	case hwmon_pwm_enable:
		mutex_lock(&ec->mode_lock);
		switch (val) {
		case 1: /* manual */
			if (ec->manual_active)
				ret = 0;
			else
				ret = asus_ec_enter_manual(ec);
			break;
		case 2: /* auto */
			if (!ec->manual_active)
				ret = 0;
			else
				ret = asus_ec_leave_manual(ec);
			break;
		default:
			ret = -EINVAL;
		}
		mutex_unlock(&ec->mode_lock);
		return ret;

	case hwmon_pwm_input:
		if (val < 0 || val > 255)
			return -EINVAL;
		speed = (u8)val;

		mutex_lock(&ec->mode_lock);
		if (!ec->manual_active) {
			mutex_unlock(&ec->mode_lock);
			return -EBUSY;	/* set pwm1_enable=1 first */
		}
		ret = asus_ec_set_pwm(ec, speed);
		mutex_unlock(&ec->mode_lock);
		if (ret)
			return ret;

		if (speed > 0 && speed < EC_PWM_SPIN_FLOOR)
			dev_info_ratelimited(ec->dev,
				"pwm=%u below spin floor (%u); fan likely idle\n",
				speed, EC_PWM_SPIN_FLOOR);
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
	.write		= asus_ec_hwmon_write,
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
	mutex_init(&ec->mode_lock);

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

	asus_ec_lookup_thermal_zones(ec);

	/* Probe-time sanity reads. */
	(void)asus_ec_read_reg(ec, EC_REG_FAN_TACH_MAJ,
			       EC_REG_FAN_TACH_MIN, &tach);
	(void)asus_ec_read_reg(ec, EC_REG_PWM_MAJ, EC_REG_PWM_RMIN, &pwm);
	(void)asus_ec_read_reg(ec, EC_REG_TEMP_MAJ, EC_REG_TEMP_MIN, &temp);
	(void)asus_ec_read_reg(ec, EC_REG_FAN_MODE_MAJ,
			       EC_REG_FAN_MODE_RMIN, &mode);

	dev_info(dev,
		 "online: tach=%u (~%u RPM) pwm=%u temp=%u°C mode=0x%02x zones=%d\n",
		 tach, tach * EC_TACH_RPM_MULT, pwm, temp, mode, ec->n_zones);

	if (mode == EC_FAN_MODE_MANUAL) {
		dev_warn(dev,
			 "EC found in MANUAL mode at probe; forcing AUTO for safety\n");
		(void)asus_ec_set_fan_mode(ec, EC_FAN_MODE_AUTO);
	}

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

	/* Initialise profile cache from EC readback (best-effort). */
	{
		u8 v;

		if (!asus_ec_read_reg(ec, EC_REG_PROFILE_MAJ,
				      EC_REG_PROFILE_RMIN, &v) &&
		    v <= EC_PROFILE_MAX)
			ec->profile_cached = v;
		else
			ec->profile_cached = EC_PROFILE_STANDARD;
	}

	/*
	 * Profile sysfs disabled on A14: register (0x01,0x0b) is read-only;
	 * writes via eccw(0x01,0x8b,n) succeed but produce no state change.
	 * Profile appears firmware-controlled (may require ACPI method or is
	 * baked into thermal tables). Vivobook opcode 0x24/0x76 also NACK on A14.
	 * TODO: investigate ACPI WMI methods or Windows driver behavior.
	 */
	dev_info(dev, "profile read-only (fw-controlled); current=%s\n",
		 profile_names[ec->profile_cached]);

	/* Uncomment when A14 profile write protocol is discovered:
	ret = devm_device_add_group(dev, &asus_ec_profile_group);
	if (ret)
		dev_warn(dev,
			 "profile sysfs registration failed: %d (continuing)\n",
			 ret);
	else
		dev_info(dev,
			 "profile sysfs registered (current=%s)\n",
			 profile_names[ec->profile_cached]);
	*/

	return 0;
}

static void asus_ec_remove(struct platform_device *pdev)
{
	struct asus_ec *ec = platform_get_drvdata(pdev);

	if (!ec)
		return;

	/* Always restore auto + stop watchdog before tearing down. */
	mutex_lock(&ec->mode_lock);
	if (ec->manual_active)
		(void)asus_ec_leave_manual(ec);
	else
		asus_ec_stop_watchdog(ec);
	mutex_unlock(&ec->mode_lock);

	/* hwmon_dev is devm-managed and torn down automatically. */

	if (!IS_ERR_OR_NULL(ec->fan_client))
		i2c_unregister_device(ec->fan_client);
	if (!IS_ERR_OR_NULL(ec->ec_client))
		i2c_unregister_device(ec->ec_client);
	if (ec->adapter)
		i2c_put_adapter(ec->adapter);

	dev_info(&pdev->dev, "removed\n");
}

/* ------------------------------------------------------------------ */
/* PM                                                                 */
/* ------------------------------------------------------------------ */

static int __maybe_unused asus_ec_suspend(struct device *dev)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	bool was_manual;
	int ret;

	mutex_lock(&ec->mode_lock);
	was_manual = ec->manual_active;
	if (was_manual) {
		/* Drop to auto + stop kthread before the system freezes. */
		ret = asus_ec_set_fan_mode(ec, EC_FAN_MODE_AUTO);
		if (ret)
			dev_warn(dev,
				 "suspend: failed to set auto: %d (proceeding)\n",
				 ret);
		asus_ec_stop_watchdog(ec);
		/* Keep manual_active=true so resume restores it. */
	}
	mutex_unlock(&ec->mode_lock);

	ret = fan_set_suspend(ec, 0x01);
	if (ret)
		dev_warn(dev, "suspend: fan_set_suspend(1) failed: %d\n", ret);

	return 0;
}

static int __maybe_unused asus_ec_resume(struct device *dev)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	int ret;

	ret = fan_set_suspend(ec, 0x00);
	if (ret)
		dev_warn(dev, "resume: fan_set_suspend(0) failed: %d\n", ret);

	mutex_lock(&ec->mode_lock);
	if (ec->manual_active) {
		/* Re-enter manual cleanly: kick watchdog + set EC mode. */
		ret = asus_ec_start_watchdog(ec);
		if (ret) {
			dev_err(dev,
				"resume: cannot restart watchdog (%d); leaving in auto\n",
				ret);
			ec->manual_active = false;
		} else {
			ret = asus_ec_set_fan_mode(ec, EC_FAN_MODE_MANUAL);
			if (ret) {
				dev_err(dev,
					"resume: cannot restore manual (%d); reverting\n",
					ret);
				asus_ec_stop_watchdog(ec);
				ec->manual_active = false;
			}
		}
	}
	mutex_unlock(&ec->mode_lock);

	return 0;
}

static SIMPLE_DEV_PM_OPS(asus_ec_pm_ops, asus_ec_suspend, asus_ec_resume);

static struct platform_driver asus_ec_driver = {
	.driver	= {
		.name	= DRV_NAME,
		.pm	= &asus_ec_pm_ops,
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
