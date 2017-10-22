/* Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/iio/consumer.h>
#include <linux/proc_fs.h>
#include <linux/alarmtimer.h>
#include "bq27546_gauge.h"

#ifdef CONFIG_I2C_STRESS_TEST
#include <linux/i2c_testcase.h>
#define I2C_TEST_FAIL (-1)
#endif

#define DRIVER_VERSION			"1.1.0"
#define BQ27546_I2C_NAME		"bq27546"

/* Bq27546 standard data commands */
#define BQ27546_REG_CNTL		0x00
#define BQ27546_REG_TEMP		0x06
#define BQ27546_REG_VOLT		0x08
#define BQ27546_REG_FLAGS		0x0a
#define BQ27546_REG_NAC		0x0c
#define BQ27546_REG_FCC		0x12
#define BQ27546_REG_AI			0x14
#define BQ27546_REG_TTE		0x16
#define BQ27546_REG_ITEMP		0x28
#define BQ27546_REG_CC			0x2a
#define BQ27546_REG_SOC		0x2c
#define BQ27546_REG_SOH		0x2e
/* Control subcommands */
#define BQ27546_SUBCMD_CTNL_STATUS	0x0000
#define BQ27546_SUBCMD_DEVCIE_TYPE	0x0001
#define BQ27546_SUBCMD_FW_VER		0x0002
#define BQ27546_SUBCMD_HW_VER		0x0003
#define BQ27546_SUBCMD_RESET_DATA	0x0005
#define BQ27546_SUBCMD_PREV_MACW	0x0007
#define BQ27546_SUBCMD_CHEM_ID		0x0008
#define BQ27546_SUBCMD_BD_OFFSET		0x0009
#define BQ27546_SUBCMD_CC_OFFSET		0x000a
#define BQ27546_SUBCMD_DF_VER			0x000c

#define BQ27546_FLAG_DSG	BIT(0)
#define BQ27546_FLAG_SOCF	BIT(1)
#define BQ27546_FLAG_SOC1	BIT(2)
#define BQ27546_FLAG_FC	BIT(9)

#define GAUGE_POLLING_TIME_S	30
#define GAUGE_PRINT_TIME_S		170


struct delayed_work update_gauge_status_work;
static struct timespec g_last_update_time;
static struct timespec g_last_print_time;
static struct bq27546_chip *this_chip;

enum bq27546_driverStatus g_bq27546_status = BQ27546_I2C_UNINIT;
struct bq27546_reg_cache {
	int temperature;
	int time_to_empty;
	int charge_full;
	int charge_now;
	int cycle_count;
	int capacity;
	int flags;
};
struct bq27546_chip {
	struct i2c_client *client;
	struct power_supply	*bat_psy;
	struct bq27546_reg_cache cache;
	struct iio_channel	*batt_id_chan;
	struct alarm bat_alarm;
	int batt_id;
};
static DEFINE_SPINLOCK(bat_alarm_slock);
static int bq27546_read_reg(uint8_t reg, uint16_t *val)
{
	s32 ret;
	if (this_chip) {
		ret = i2c_smbus_read_word_data(this_chip->client, reg);
		if (ret < 0) {
			BAT_DBG_E("%s: can't read from %02x: %d\n", __func__, reg, ret);
			return ret;
		} else {
			*val = ret;
		}
		return 0;
	} else{
		BAT_DBG_E("%s: driver initialization is not finished!\n", __func__);
		return -1;
	}
}
static enum alarmtimer_restart batAlarm_handler(struct alarm *alarm, ktime_t now)
{
	BAT_DBG("%s\n", __func__);
	return ALARMTIMER_NORESTART;
}
static int s16_to_s32(int input)
{
	int value = 0x0000FFFF & input;
	int mask = 0x00008000;

	if (mask & input) {
		value += 0xFFFF0000;
	}
	return value;
}
static int bq27546_get_present(int flags)
{
	if (flags < 0) {
		return 0;
	} else{
		return 1;
	}
}
static int bq27546_get_capacity_level(int flags)
{
	if (flags & BQ27546_FLAG_FC) {
		return POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	} else if (flags & BQ27546_FLAG_SOC1) {
		return POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	} else if (flags & BQ27546_FLAG_SOCF) {
		return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	} else{
		return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	}
}
static int bq27546_read_current(int *value)
{
	int ret;
 	uint16_t adc_data;

	ret = bq27546_read_reg(BQ27546_REG_AI, &adc_data);
	*value = s16_to_s32((int)adc_data);
	return ret;
}
static int bq27546_read_voltage(int *value)
{
	int ret;
 	uint16_t adc_data;

	ret = bq27546_read_reg(BQ27546_REG_VOLT, &adc_data);
	*value = adc_data;
	return ret;
}
static int bq27546_read_capacity(int *value)
{
	int ret;
 	uint16_t adc_data;

	ret = bq27546_read_reg(BQ27546_REG_SOC, &adc_data);
	*value = adc_data;
	return ret;
}
static int bq27546_read_flags(int *value)
{
	int ret;
 	uint16_t adc_data;

	ret = bq27546_read_reg(BQ27546_REG_FLAGS, &adc_data);
	*value = adc_data;
	return ret;
}
static int bq27546_read_temperature(int *value)
{
	int ret;
 	uint16_t adc_data;

	ret = bq27546_read_reg(BQ27546_REG_TEMP, &adc_data);
	*value = adc_data - 2730;
	return ret;
}
static int bq27546_read_timeToEmpty(int *value)
{
	int ret;
 	uint16_t adc_data;

	ret = bq27546_read_reg(BQ27546_REG_TTE, &adc_data);
	if (adc_data == 65535) {
		return -ENODATA;
	} else{
		*value = adc_data * 60;
		return ret;
	}
}
static int bq27546_read_chargeNow(int *value)
{
	int ret;
 	uint16_t adc_data;

	ret = bq27546_read_reg(BQ27546_REG_NAC, &adc_data);
	*value = adc_data;
	return ret;
}
static int bq27546_read_chargeFull(int *value)
{
	int ret;
 	uint16_t adc_data;

	ret = bq27546_read_reg(BQ27546_REG_FCC, &adc_data);
	*value = adc_data;
	return ret;
}
static int bq27546_read_cycleCount(int *value)
{
	int ret;
 	uint16_t adc_data;

	ret = bq27546_read_reg(BQ27546_REG_CC, &adc_data);
	*value = adc_data;
	return ret;
}
void bq27546_polling_data_worker(int time_s)
{
	cancel_delayed_work(&update_gauge_status_work);
	schedule_delayed_work(&update_gauge_status_work, time_s * HZ);
}
void bq27546_setAlarm(int l_soc)
{
	unsigned long batflags;
	struct timespec new_batAlarm_time;
	struct timespec mtNow;
	int RTCSetInterval = 180;

	mtNow = current_kernel_time();
	new_batAlarm_time.tv_sec = 0;
	new_batAlarm_time.tv_nsec = 0;
	if (l_soc == 100) {
		RTCSetInterval = 36000;
	} else if (l_soc > 15 || l_soc == 0) {
		RTCSetInterval = 80 * (100 - l_soc);
	} else{
		RTCSetInterval = l_soc * 450;
	}
	if (RTCSetInterval < 450) {
		RTCSetInterval = 450;
	} else if (RTCSetInterval > 36000) {
		RTCSetInterval = 36000;
	}
	new_batAlarm_time.tv_sec = mtNow.tv_sec + RTCSetInterval;
	BAT_DBG("%s: alarm start after %ds\n", __func__, RTCSetInterval);
	spin_lock_irqsave(&bat_alarm_slock, batflags);
	alarm_start(&this_chip->bat_alarm,
		timespec_to_ktime(new_batAlarm_time));
	spin_unlock_irqrestore(&bat_alarm_slock, batflags);
}
void update_battery_status(void)
{
 	int value, ret;
	
	struct bq27546_reg_cache cache = {0, };
	
	if (!this_chip) {
		BAT_DBG_E("%s: driver initialization is not finished!\n", __func__);
		return;
	}
	ret = bq27546_read_capacity(&value);
	if (ret == 0) {
		cache.capacity = value;
	}
	ret = bq27546_read_flags(&value);
	if (ret == 0) {
		if ((value & 0xff) == 0xff) {
			cache.flags = -1;
		} else{
			cache.flags = value;
		}
	}
	ret = bq27546_read_temperature(&value);
	if (ret == 0) {
		cache.temperature = value;
	}
	ret = bq27546_read_timeToEmpty(&value);
	if (ret == 0) {
		cache.time_to_empty= value;
	}
	ret = bq27546_read_chargeNow(&value);
	if (ret == 0) {
		cache.charge_now= value;
	}
	ret = bq27546_read_chargeFull(&value);
	if (ret == 0) {
		cache.charge_full= value;
	}
	ret = bq27546_read_cycleCount(&value);
	if (ret == 0) {
		cache.cycle_count= value;
	}

	g_last_update_time = current_kernel_time();
	if (memcmp(&this_chip->cache, &cache, sizeof(cache)) != 0) {
		this_chip->cache = cache;
	}
}
void print_battery_status(void)
{
	char battInfo[256];
	int bat_vol, bat_cur;
	struct timespec mtNow;
	mtNow = current_kernel_time();
	if (g_last_print_time.tv_sec == 0 || mtNow.tv_sec - g_last_print_time.tv_sec >= GAUGE_PRINT_TIME_S) {
		bq27546_read_voltage(&bat_vol);
		bq27546_read_current(&bat_cur);
		snprintf(battInfo, sizeof(battInfo), "report Capacity ==>%d, FCC:%dmAh, BMS:%d, V:%dmV, Cur:%dmA, ",
			this_chip->cache.capacity,
			this_chip->cache.charge_full,
			this_chip->cache.capacity,
			bat_vol,
			bat_cur);
		snprintf(battInfo, sizeof(battInfo), "%sTemp:%d.%dC, BATID:%d\n",
			battInfo,
			this_chip->cache.temperature/10,
			this_chip->cache.temperature%10,
			this_chip->batt_id);
		/*ASUSEvtlog("[BAT][Ser]%s", battInfo);*/
		printk("[BAT][Ser]%s", battInfo);
		g_last_print_time = mtNow;
	}
}
void update_gauge_status_worker(struct work_struct *dat)
{
	static int l_lastSoc = -1;
	update_battery_status();
	if (l_lastSoc != this_chip->cache.capacity) {
		l_lastSoc = this_chip->cache.capacity;
		bq27546_setAlarm(l_lastSoc);
		power_supply_changed(this_chip->bat_psy);
	}
	print_battery_status();

	bq27546_polling_data_worker(GAUGE_POLLING_TIME_S);
}
static void bq27546_getBatID(struct bq27546_chip *chip)
{
	int rc, batt_id = -EINVAL;

	if (!chip->batt_id_chan) {
		BAT_DBG_E("%s: batt_id_chan isn't exist\n", __func__);
		return;
	}
	/*get batt id*/
	rc = iio_read_channel_processed(chip->batt_id_chan, &batt_id);
	if (rc < 0) {
		BAT_DBG_E("%s: Error in reading batt_id channel, rc:%d\n", __func__, rc);
	} else{
		BAT_DBG("%s: batt_id: %d\n", __func__, batt_id);
		chip->batt_id = batt_id;
	}
}
static int bat_psy_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct bq27546_chip *chip = power_supply_get_drvdata(psy);
	int ret = 0;
	int value;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = bq27546_read_voltage(&value);
		if (ret == 0) {
			val->intval = value * 1000;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = bq27546_read_current(&value);
		if (ret == 0) {
			val->intval = value * 1000;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq27546_get_present(chip->cache.flags);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = bq27546_get_capacity_level(chip->cache.flags);
		break;
	case POWER_SUPPLY_PROP_RESISTANCE_ID:
		bq27546_getBatID(chip);
		val->intval = chip->batt_id;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = chip->cache.capacity;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = chip->cache.temperature;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		val->intval = chip->cache.time_to_empty;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = chip->cache.cycle_count;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = chip->cache.charge_now * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = chip->cache.charge_full * 1000;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int bat_psy_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *pval)
{
	/*struct bq27546_chip *chip = power_supply_get_drvdata(psy);*/

	switch (psp) {
	default:
		break;
	}

	return 0;
}

static int bat_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	default:
		break;
	}

	return 0;
}
static enum power_supply_property bat_psy_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_RESISTANCE_ID,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static const struct power_supply_desc bat_psy_desc = {
	.name = "bms",
	.type = POWER_SUPPLY_TYPE_BMS,
	.properties = bat_psy_props,
	.num_properties = ARRAY_SIZE(bat_psy_props),
	.get_property = bat_psy_get_property,
	.set_property = bat_psy_set_property,
	.property_is_writeable = bat_property_is_writeable,
};
static void bq27546_register_psy(struct bq27546_chip *chip)
{
	struct power_supply_config bat_psy_cfg;
	
	bat_psy_cfg.drv_data = chip;
	bat_psy_cfg.of_node = NULL;
	bat_psy_cfg.supplied_to = NULL;
	bat_psy_cfg.num_supplicants = 0;
	chip->bat_psy = devm_power_supply_register(&chip->client->dev, &bat_psy_desc, &bat_psy_cfg);
}
static int bq27546_i2cTest(void)
{
 	int value, ret;
	ret = bq27546_read_capacity(&value);
	return ret;
}
static int bq27546_initBatID(struct bq27546_chip *chip)
{
	int rc;

	/*get batt id channel*/
	chip->batt_id_chan = iio_channel_get(&chip->client->dev, "rradc_batt_id");
	if (IS_ERR(chip->batt_id_chan)) {
		if (PTR_ERR(chip->batt_id_chan) != -EPROBE_DEFER) {
			BAT_DBG_E("%s: batt_id_chan unavailable %ld\n", __func__, PTR_ERR(chip->batt_id_chan));
			rc = 0;
		} else{
			BAT_DBG_E("%s: batt_id_chan DEFER\n", __func__);
			rc = -EPROBE_DEFER;
		}
		chip->batt_id_chan = NULL;
	} else{
		bq27546_getBatID(chip);
		rc = 0;
	}
	return rc;
}
/*+++BSP David proc batt_current Interface+++*/
static int batt_current_proc_read(struct seq_file *buf, void *v)
{
	int result = 0;

	bq27546_read_current(&result);
	BAT_DBG("%s: %d\n", __func__, result);
	seq_printf(buf, "%d\n", result);
	return 0;
}
static int batt_current_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, batt_current_proc_read, NULL);
}

static void create_batt_current_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open =  batt_current_proc_open,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/batt_current", 0444, NULL, &proc_fops);
	if (!proc_file) {
		BAT_DBG_E("[Proc]%s failed!\n", __func__);
	}
	return;
}
/*---BSP David proc batt_current Interface---*/
/*+++BSP David proc batt_voltage Interface+++*/
static int batt_voltage_proc_read(struct seq_file *buf, void *v)
{
	int result = 0;
	bq27546_read_voltage(&result);
	BAT_DBG("%s: %d\n", __func__, result);
	seq_printf(buf, "%d\n", result);
	return 0;
}
static int batt_voltage_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, batt_voltage_proc_read, NULL);
}

static void create_batt_voltage_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open =  batt_voltage_proc_open,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/batt_voltage", 0444, NULL, &proc_fops);
	if (!proc_file) {
		BAT_DBG_E("[Proc]%s failed!\n", __func__);
	}
	return;
}
/*---BSP David proc batt_voltage Interface---*/
/*+++BSP David proc gaugeIC_status Interface+++*/
static int gaugeIC_status_proc_read(struct seq_file *buf, void *v)
{
	int result = 0;
	if (bq27546_i2cTest() < 0) {
		result = 0;
	} else{
		result = 1;
	}
	BAT_DBG("%s: %d\n", __func__, result);
	seq_printf(buf, "%d\n", result);
	return 0;
}
static int gaugeIC_status_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, gaugeIC_status_proc_read, NULL);
}

static void create_gaugeIC_status_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open =  gaugeIC_status_proc_open,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/gaugeIC_status", 0444, NULL, &proc_fops);
	if (!proc_file) {
		BAT_DBG_E("[Proc]%s failed!\n", __func__);
	}
	return;
}
/*---BSP David proc gaugeIC_status Interface---*/
static void bq27546_cleanup(struct bq27546_chip *chip)
{
	if (chip->batt_id_chan) {
		iio_channel_release(chip->batt_id_chan);
	}
	kfree(chip);
}
#ifdef CONFIG_I2C_STRESS_TEST
static int I2CTestReadWrite(struct i2c_client *client)
{
	bool statusValid = false;
	i2c_log_in_test_case("[BAT][BMS][Test]%s start\n", __func__);
	if (bq27546_i2cTest() < 0) {
		statusValid = false;
	} else{
		statusValid = true;
	}
	i2c_log_in_test_case("[BAT][BMS][Test]%s: %s\n", __func__, statusValid ? "PASS" : "FAIL");
	return statusValid ? I2C_TEST_PASS : I2C_TEST_FAIL;
};
static struct i2c_test_case_info gI2CTestCaseInfo[] =
{
	__I2C_STRESS_TEST_CASE_ATTR(I2CTestReadWrite),
};
static void i2c_stress_test(void)
{
	BAT_DBG("%s\n", __func__);
	i2c_add_test_case(this_chip->client, "bq27546", ARRAY_AND_SIZE(gI2CTestCaseInfo));
}
#endif
static int bq27546_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq27546_chip *chip;
	int rc = 0;
	BAT_DBG("%s: Probe start!\n", __func__);

	/*allocate chip*/
	chip = kzalloc(sizeof(struct bq27546_chip), GFP_KERNEL);
	if (!chip) {
		rc = -ENOMEM;
		BAT_DBG_E("%s: failed to allocate chip, rc = %d\n", __func__, rc);
		goto exit_without_clean;
	}

	chip->client = client;
	i2c_set_clientdata(client, chip);
	this_chip = chip;

	/*test bq27546 i2c*/
	rc = bq27546_i2cTest();
	if (rc < 0) {
		g_bq27546_status = BQ27546_I2C_FAIL;
		BAT_DBG_E("%s: i2c test fail!\n", __func__);
		goto exit;
	} else{
		g_bq27546_status = BQ27546_I2C_SUCCESS;
	}

	/*get batt ID*/
	rc = bq27546_initBatID(chip);
	if (rc == -EPROBE_DEFER) {
		goto exit;
	}

	/*register power supply*/
	bq27546_register_psy(chip);
	if (IS_ERR(chip->bat_psy)) {
		rc = PTR_ERR(chip->bat_psy);
		BAT_DBG_E("%s: failed to register bat_psy rc = %d\n", __func__, rc);
		goto exit;
	}

	/*initialize polling worker*/
	alarm_init(&chip->bat_alarm, ALARM_REALTIME, batAlarm_handler);
	INIT_DELAYED_WORK(&update_gauge_status_work, update_gauge_status_worker);
	g_last_print_time.tv_sec = 0;
	bq27546_polling_data_worker(0);

	create_gaugeIC_status_proc_file();
	create_batt_current_proc_file();
	create_batt_voltage_proc_file();

#ifdef CONFIG_I2C_STRESS_TEST
	i2c_stress_test();
#endif

	BAT_DBG("%s: Probe success!\n", __func__);
	return 0;
	
exit:
	bq27546_cleanup(chip);
exit_without_clean:
	return rc;
}
static int bq27546_battery_remove(struct i2c_client *client)
{
	return 0;
}
static int bq27546_suspend(struct device *dev)
{
	cancel_delayed_work(&update_gauge_status_work);
	return 0;
}
static int bq27546_resume(struct device *dev)
{
	struct timespec mtNow;
	int l_timeSinceLastUpdate_s;

	mtNow = current_kernel_time();
	l_timeSinceLastUpdate_s = mtNow.tv_sec - g_last_update_time.tv_sec;
	if (l_timeSinceLastUpdate_s >= GAUGE_POLLING_TIME_S) {
		bq27546_polling_data_worker(0);
	} else{
		bq27546_polling_data_worker(GAUGE_POLLING_TIME_S - l_timeSinceLastUpdate_s);
	}
	return 0;
}
static const struct i2c_device_id bq27546_id[] = {
	{ "bq27546", 1 },
	{},
};
MODULE_DEVICE_TABLE(i2c, BQ27546_id);

static struct of_device_id bq27546_match_table[] = {
	{ .compatible = "bq27546",},
	{ },
};
static const struct dev_pm_ops bq27546_pm_ops = {
	.suspend	= bq27546_suspend,
	.resume	= bq27546_resume,
};
static struct i2c_driver bq27546_battery_driver = {
	.id_table	= bq27546_id,
	.probe		= bq27546_battery_probe,
	.driver		= {
		.name = BQ27546_I2C_NAME,
		.owner = THIS_MODULE,
 		.of_match_table = of_match_ptr(bq27546_match_table),
		.pm	= &bq27546_pm_ops,
	},
	.remove		= bq27546_battery_remove,
};
static int __init bq27546_battery_init(void)
{
	BAT_DBG("%s\n", __func__);
	return i2c_add_driver(&bq27546_battery_driver);
}
module_init(bq27546_battery_init);
static void __exit bq27546_battery_exit(void)
{
	i2c_del_driver(&bq27546_battery_driver);
}
module_exit(bq27546_battery_exit);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Qualcomm Innovation Center, Inc.");
MODULE_DESCRIPTION("BQ27546 battery monitor driver");
