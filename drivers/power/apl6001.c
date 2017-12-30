/*
APL6001 ADC Driver
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/of.h>

#include <linux/gpio.h>

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/spmi.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/semaphore.h>
#include <linux/device.h>
#include <linux/syscalls.h>
#include <asm/uaccess.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/sched.h>
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/clk.h>
#include <linux/wakelock.h>

//#include <linux/i2c/ads1015.h>

//ASUS BSP Austin_T : global apl6001_READY +++
bool apl6001_ready;
EXPORT_SYMBOL(apl6001_ready);

//Define register addresses of apl6001 0X38
#define apl6001_raddr 0x38
#define PW_ADC_EN_GPIO	90

struct apl6001_data
{
	u32 apl6001;
};

struct i2c_client *apl6001_client;

/*
apl6001_write_reg():	write 8 bits reg function
slave_addr:	SMBus address (7 bits)
cmd_reg   :	cmd register for programming
write_val  :	the value will be written
*/
int apl6001_write_reg(uint8_t slave_addr, uint8_t cmd_reg, uint8_t write_val)
{
	int ret = 0;

	printk("[BAT][CHG] apl6001_write_reg start\n");
	apl6001_client->addr = slave_addr; //real SMBus address (8 bits)
	ret = i2c_smbus_write_byte_data(apl6001_client, cmd_reg, write_val);
	if (ret < 0) {
		printk("%s: failed to write i2c addr=%x\n",
			__func__, slave_addr);
	}

	return ret;
}

/*
apl6001_read_reg():	read 8 bits reg function
slave_addr:	SMBus address (7 bits)
cmd_reg   :	cmd register for programming
store_read_val  :	value be read will store here

*/
int apl6001_read_reg(uint8_t slave_addr, uint8_t cmd_reg, uint8_t *store_read_val)
{
	int ret = 0;

	apl6001_client->addr = slave_addr;
	ret = i2c_smbus_read_byte_data(apl6001_client, cmd_reg);
	if (ret < 0) {
		printk("%s: failed to read i2c addr=%x\n",	__func__, slave_addr);
	}

	*store_read_val = (uint8_t) ret;

	return ret;
}

/*u8 apl6001_PVC_EN_write(int en)
{
	int ret;
	u8 reg;
	u8 mask = 0x01;

	apl6001_read_reg(apl6001_raddr, 0x00000004, &reg);

	reg &= ~mask;

	if (en == 0)
		reg |= 0x00 & mask;
	else if (en == 1)
		reg |= 0x01 & mask;

	ret = apl6001_write_reg(apl6001_raddr, 0x00000004, reg);

	return ret;
}*/

u8 apl6001_adapter_value(void)
{
	u8 my_read_value = 0;

	apl6001_read_reg(apl6001_raddr, 0x00000004, &my_read_value);
	printk("[BAT][CHG] apl6001_value = 0x%xh\n", my_read_value);

	return my_read_value;
}
EXPORT_SYMBOL(apl6001_adapter_value);

u8 apl6001_thermal_value(void)
{
	u8 my_read_value = 0;

	apl6001_read_reg(apl6001_raddr, 0x00000003, &my_read_value);
	printk("[BAT][CHG] apl6001_value = 0x%xh\n", my_read_value);

	return my_read_value;
}
EXPORT_SYMBOL(apl6001_thermal_value);

/*static ssize_t apl6001_pvc_ctrl_show(struct device *dev, struct device_attribute *da,
	char *buf)
{
	u16 ret;

	ret = apl6001_PVC_CTRL_dump_value();

	return sprintf(buf, "0x%xh\n", ret);
}

static ssize_t apl6001_pvc_ctrl_write_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	int tmp = 0;
	int ret;

	tmp = buf[0] - 48;
	ret = apl6001_PVC_EN_write(tmp);

	return len;
}*/

static ssize_t adapter_value_show(struct device *dev, struct device_attribute *da,
	char *buf)
{
	u8 val;

	val = apl6001_adapter_value();

	return sprintf(buf, "0x%xh\n", val);
}

static ssize_t thermal_value_show(struct device *dev, struct device_attribute *da,
	char *buf)
{
	u16 val;

	val = apl6001_thermal_value();

	return sprintf(buf, "0x%xh\n", val);
}

static ssize_t adc_ack_show(struct device *dev, struct device_attribute *da,
	char *buf)
{
	u8 val = 0;
	int ret = 0;
	bool ack = 1;
	int rc;

	rc = gpio_direction_output(PW_ADC_EN_GPIO, 0);
	if (rc)
		printk("[BAT][CHG] adc_ack_show : Failed to pull-low PW_ADC_EN-gpios90\n");

	msleep(100);

	ret = apl6001_read_reg(apl6001_raddr, 0x00000004, &val);
	if (ret < 0)
		ack = 0;
	else
		ack = 1;

	rc = gpio_direction_output(PW_ADC_EN_GPIO, 1);
	if (rc)
		printk("[BAT][CHG] adc_ack_show : Failed to pull-high PW_ADC_EN-gpios90\n");

	return sprintf(buf, "%d\n", ack);
}

static DEVICE_ATTR(adapter_value, 0664, adapter_value_show, NULL);
static DEVICE_ATTR(thermal_value, 0664, thermal_value_show, NULL);
static DEVICE_ATTR(adc_ack, 0664, adc_ack_show, NULL);

static struct attribute *dump_reg_attrs[] = {
	&dev_attr_adapter_value.attr,
	&dev_attr_thermal_value.attr,
	&dev_attr_adc_ack.attr,
	NULL
};

static const struct attribute_group dump_reg_attr_group = {
	.attrs = dump_reg_attrs,
};

static int apl6001_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct apl6001_data *data;
	int rc;

	printk("[BAT][CHG] %s start\n", __FUNCTION__);

	apl6001_ready = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk("[BAT][CHG] %s: i2c bus does not support the apl6001\n", __FUNCTION__);
	}

	data = devm_kzalloc(&client->dev, sizeof(struct apl6001_data), GFP_KERNEL);

	if (!data)
		return -ENOMEM;

	apl6001_client = client;
	i2c_set_clientdata(client, data);

	rc = sysfs_create_group(&client->dev.kobj, &dump_reg_attr_group);
	if (rc)
		goto exit_remove;

	apl6001_ready = 1;

	printk("[BAT][CHG] %s end\n", __FUNCTION__);

	return 0;

exit_remove:
		sysfs_remove_group(&client->dev.kobj, &dump_reg_attr_group);
	return rc;

}

static struct of_device_id apl6001_match_table[] = {
	{ .compatible = "qcom,apl6001",},
	{ },
};

static const struct i2c_device_id apl6001_id[] = {
	{ "apl6001", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, apl6001_id);

static struct i2c_driver apl6001_driver = {
	.driver = {
		.name = "apl6001",
		.owner		= THIS_MODULE,
		.of_match_table	= apl6001_match_table,
	},
	.probe = apl6001_probe,
	.id_table = apl6001_id,
};

module_i2c_driver(apl6001_driver);

MODULE_AUTHOR("Dirk Eibach <eibach@gdsys.de>");
MODULE_DESCRIPTION("apl6001 driver");
MODULE_LICENSE("GPL");

