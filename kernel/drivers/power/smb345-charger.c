/*
  * drivers/power/Smb345-charger.c
  *
  * Charger driver for Summit SMB345
  *
  * Copyright (c) 2012, ASUSTek Inc.
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 2 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful, but WITHOUT
  * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  * more details.
  */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/smb345-charger.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/usb/otg.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <mach/board_asustek.h>

#define smb345_CHARGE		0x00
#define smb345_CHRG_CRNTS	0x01
#define smb345_VRS_FUNC		0x02
#define smb345_FLOAT_VLTG	0x03
#define smb345_CHRG_CTRL	0x04
#define smb345_STAT_TIME_CTRL	0x05
#define smb345_PIN_CTRL		0x06
#define smb345_THERM_CTRL	0x07
#define smb345_SYSOK_USB3	0x08
#define smb345_CTRL_REG		0x09

#define smb345_OTG_TLIM_REG	0x0A
#define smb345_HRD_SFT_TEMP	0x0B
#define smb345_FAULT_INTR	0x0C
#define smb345_STS_INTR_1	0x0D
#define smb345_I2C_ADDR	0x0E
#define smb345_IN_CLTG_DET	0x10
#define smb345_STS_INTR_2	0x11

/* Command registers */
#define smb345_CMD_REG		0x30
#define smb345_CMD_REG_B	0x31
#define smb345_CMD_REG_c	0x33

/* Interrupt Status registers */
#define smb345_INTR_STS_A	0x35
#define smb345_INTR_STS_B	0x36
#define smb345_INTR_STS_C	0x37
#define smb345_INTR_STS_D	0x38
#define smb345_INTR_STS_E	0x39
#define smb345_INTR_STS_F	0x3A

/* Status registers */
#define smb345_STS_REG_A	0x3B
#define smb345_STS_REG_B	0x3C
#define smb345_STS_REG_C	0x3D
#define smb345_STS_REG_D	0x3E
#define smb345_STS_REG_E	0x3F
#define STAT_OUTPUT_EN		0x20

/* APQ8064 GPIO pin definition */
#define APQ_AP_CHAR	22
#define APQ_AP_ACOK	23

#define smb345_ENABLE_WRITE	1
#define smb345_DISABLE_WRITE	0
#define ENABLE_WRT_ACCESS	0x80
#define ENABLE_APSD		0x04
#define PIN_CTRL		0x10
#define PIN_ACT_LOW	0x20
#define ENABLE_CHARGE		0x02
#define USBIN		0x80
#define APSD_OK		0x08
#define APSD_RESULT		0x07
#define FLOAT_VOLT_MASK	0x3F
#define GPIO_AC_OK		APQ_AP_ACOK
#define GPIO_WPC_POK 34
#define MAX_DCIN 1
#define MAX_USBIN 0
#define REVERTBYTE(a)	(char)((((a)>>4) & 0x0F) |(((a)<<4) & 0xF0))
#define WPC_DEBOUNCE_INTERVAL	(1 * HZ)
#define WPC_SET_CURT_INTERVAL	(2 * HZ)
#define WPC_INIT_DET_INTERVAL	(22 * HZ)
#define WPC_SET_CURT_LIMIT_CNT	6

/* Functions declaration */
extern int  bq27541_battery_callback(unsigned usb_cable_state);
extern int  bq27541_wireless_callback(unsigned wireless_state);
extern int  bq27541_battery_current(void);
extern void touch_callback(unsigned cable_status);
static ssize_t smb345_reg_show(struct device *dev, struct device_attribute *attr, char *buf);
#ifdef FACTORY_IMAGE
static ssize_t smb345_chg_status_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t show_smb345_charger_status(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t smb345_AICL_result_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t smb345_input_AICL_result_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t smb345_led_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count);
static ssize_t smb345_reAICL_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count);
static ssize_t smb345_reAICL_show(struct device *dev,struct device_attribute *attr, char *buf);
static ssize_t smb345_check_current_show(struct device *dev,struct device_attribute *attr, char *buf);
#endif

/* Global variables */
static struct smb345_charger *charger;
static struct workqueue_struct *smb345_wq;
struct wake_lock charger_wakelock;
struct wake_lock wireless_wakelock;
bool wireless_on;
bool otg_on;
static int charge_en_flag = 1;
unsigned int smb345_charger_status = 0;
extern int ac_on;
static bool wpc_en;
#ifdef FACTORY_IMAGE
int AICL_result = -1;
unsigned int check_current_count = 0;
#endif

/* Sysfs interface */
static DEVICE_ATTR(reg_status, S_IWUSR | S_IRUGO, smb345_reg_show, NULL);
#ifdef FACTORY_IMAGE
static DEVICE_ATTR(charging_status, S_IWUSR | S_IRUGO, smb345_chg_status_show, NULL);
static DEVICE_ATTR(smb345_status, S_IWUSR | S_IRUGO, show_smb345_charger_status,NULL);
static DEVICE_ATTR(AICL_result, S_IWUSR | S_IRUGO, smb345_AICL_result_show,NULL);
static DEVICE_ATTR(input_AICL_result, S_IWUSR | S_IRUGO, smb345_input_AICL_result_show,NULL);
static DEVICE_ATTR(led_test, S_IWUSR | S_IRUGO, NULL, smb345_led_store);
static DEVICE_ATTR(smb345_reAICL, S_IWUSR | S_IRUGO, smb345_reAICL_show, smb345_reAICL_store);
static DEVICE_ATTR(check_current, S_IWUSR | S_IRUGO, smb345_check_current_show,NULL);
#endif

static struct attribute *smb345_attributes[] = {
	&dev_attr_reg_status.attr,
#ifdef FACTORY_IMAGE
	&dev_attr_charging_status.attr,
	&dev_attr_smb345_status.attr,
	&dev_attr_AICL_result.attr,
	&dev_attr_led_test.attr,
	&dev_attr_smb345_reAICL.attr,
	&dev_attr_input_AICL_result.attr,
	&dev_attr_check_current.attr,
#endif
NULL
};

static const struct attribute_group smb345_group = {
	.attrs = smb345_attributes,
};

#define MAXIMAL_RETAY 10
static int smb345_read(struct i2c_client *client, int reg)
{
	int ret;
	int retry = 0;

	do {
		ret = i2c_smbus_read_byte_data(client, reg);

		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d  retry =%d\n", __func__, ret,  retry);
			mdelay(20);
		} else
			break;
	} while (++retry < MAXIMAL_RETAY);

	return ret;
}

static int smb345_write(struct i2c_client *client, int reg, u8 value)
{
	int ret;
	int retry = 0;

	do {
		ret = i2c_smbus_write_byte_data(client, reg, value);

		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d retry =%d\n", __func__, ret, retry);
			mdelay(20);
		} else
			break;
	} while (++retry < MAXIMAL_RETAY);

	return ret;
}

static int smb345_update_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret, retval;

	retval = smb345_read(client, reg);
	if (retval < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, retval);
		return retval;
	}

	ret = smb345_write(client, reg, retval | value);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	return ret;
}

static int smb345_clear_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret, retval;

	retval = smb345_read(client, reg);
	if (retval < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, retval);
		return retval;
	}

	ret = smb345_write(client, reg, retval & (~value));
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	return ret;
}

int smb345_volatile_writes(struct i2c_client *client, uint8_t value)
{
	int ret = 0;

	if (value == smb345_ENABLE_WRITE) {
		/* Enable volatile write to config registers */
		ret = smb345_update_reg(client, smb345_CMD_REG,
						ENABLE_WRT_ACCESS);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing"
				"register 0x%02x\n", __func__, smb345_CMD_REG);
			return ret;
		}
	} else {
		ret = smb345_read(client, smb345_CMD_REG);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return ret;
		}

		ret = smb345_write(client, smb345_CMD_REG, ret & (~(1<<7)));
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return ret;
		}
	}
	return ret;
}

int check_charging_status(void)
{
	struct i2c_client *client = charger->client;
	int ret;

	ret = i2c_smbus_read_byte_data(client, smb345_STS_REG_C);
	if (ret < 0)
		return -1;
	else
		return (ret &= 0x06);
}
EXPORT_SYMBOL(check_charging_status);

int wireless_is_plugged(void)
{
	if (charger->wpc_pok_gpio != -1)
		return !(gpio_get_value(charger->wpc_pok_gpio));
	else
		return 0;
}

static int smb345_pin_control(bool state)
{
	struct i2c_client *client = charger->client;
	u8 ret = 0;

	mutex_lock(&charger->pinctrl_lock);

	if (state) {
		/*Pin Controls -active low */
		ret = smb345_update_reg(client, smb345_PIN_CTRL, PIN_ACT_LOW);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed to"
						"enable charger\n", __func__);
		}
	} else {
		/*Pin Controls -active high */
		ret = smb345_clear_reg(client, smb345_PIN_CTRL, PIN_ACT_LOW);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed to"
						"disable charger\n", __func__);
		}
	}

	mutex_unlock(&charger->pinctrl_lock);
	return ret;
}

int usb_switch_remove_usb_path_setting(void)
{
	struct i2c_client *client = charger->client;
	int ret = 0;

	ret = smb345_volatile_writes(client, smb345_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",__func__);
		goto error;
	}

	SMB_NOTICE("Set ICL=0x01\n");
	ret = smb345_write(client, smb345_CHRG_CRNTS, 0x01);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing 0x01 to register"
			"0x%02x\n", __func__, smb345_CHRG_CRNTS);
		goto error;
	}

	ret = smb345_volatile_writes(client, smb345_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",__func__);
		goto error;
	}

error:
	return ret;
}

int usb_switch_remove_ac_path_setting(void)
{
	struct i2c_client *client = charger->client;
	int ret = 0;

	ret = smb345_volatile_writes(client, smb345_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",__func__);
		goto error;
	}

	SMB_NOTICE("Set USB to HC Mode\n");
	ret = smb345_update_reg(client, smb345_CMD_REG_B, 0x03);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",__func__);
		goto error;
	}

	ret = smb345_read(client, smb345_PIN_CTRL);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in reading 0x%02x\n",
				__func__, smb345_PIN_CTRL);
		goto error;
	}

	SMB_NOTICE("Set Register Control, retval=%x setting=%x\n", ret, (int)(ret & (~(BIT(4)))));
	ret = smb345_write(client, smb345_PIN_CTRL, (u8)(ret & (~(BIT(4)))));
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing 0x%02x to register"
			"0x%02x\n", __func__, (int)(ret & (~(BIT(4)))), smb345_PIN_CTRL);
		goto error;
	}

	ret = smb345_volatile_writes(client, smb345_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",__func__);
		goto error;
	}

error:
	return ret;

}

int smb345_charger_enable(bool state)
{
	struct i2c_client *client = charger->client;
	u8 ret = 0;

	ret = smb345_volatile_writes(client, smb345_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}
	charge_en_flag = state;
	smb345_pin_control(state);

	ret = smb345_volatile_writes(client, smb345_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}

error:
	return ret;
}
EXPORT_SYMBOL_GPL(smb345_charger_enable);

int
smb345_set_InputCurrentlimit(struct i2c_client *client, u32 current_setting, int usb_dc)
{
	int ret = 0, retval;
	u8 setting, tempval = 0;

	wake_lock(&charger_wakelock);

	ret = smb345_volatile_writes(client, smb345_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}

	if (charge_en_flag)
		smb345_pin_control(0);

	retval = smb345_read(client, smb345_VRS_FUNC);
	if (retval < 0) {
		dev_err(&client->dev, "%s(): Failed in reading 0x%02x",
				__func__, smb345_VRS_FUNC);
		goto error;
	}

	setting = retval & (~(BIT(4)));
	SMB_NOTICE("Disable AICL, retval=%x setting=%x\n", retval, setting);
	ret = smb345_write(client, smb345_VRS_FUNC, setting);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing 0x%02x to register"
			"0x%02x\n", __func__, setting, smb345_VRS_FUNC);
		goto error;
	}

	if(current_setting != 0)
	{
		retval = smb345_read(client, smb345_CHRG_CRNTS);
		if (retval < 0) {
			dev_err(&client->dev, "%s(): Failed in reading 0x%02x",
				__func__, smb345_CHRG_CRNTS);
			goto error;
		}

		if(usb_dc == MAX_DCIN)
			tempval = REVERTBYTE(retval);
		else if(usb_dc == MAX_USBIN)
			tempval = retval;

		setting = tempval & 0xF0;
		if(current_setting == 2000)
			setting |= 0x07;
		else if(current_setting == 1800)
			setting |= 0x06;
		else if(current_setting == 1200)
			setting |= 0x04;
		else if(current_setting == 900)
			setting |= 0x03;
		else if(current_setting == 700)
			setting |= 0x02;
		else if(current_setting == 500)
			setting |= 0x01;

		if(usb_dc == MAX_DCIN)
			setting = REVERTBYTE(setting);

		SMB_NOTICE("Set ICL=%u retval =%x setting=%x\n", current_setting, retval, setting);

		ret = smb345_write(client, smb345_CHRG_CRNTS, setting);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing 0x%02x to register"
				"0x%02x\n", __func__, setting, smb345_CHRG_CRNTS);
			goto error;
		}

		if(current_setting == 2000)
			charger->curr_limit = 2000;
		else if(current_setting == 1800)
			charger->curr_limit = 1800;
		else if(current_setting == 1200)
			charger->curr_limit = 1200;
		else if(current_setting == 900)
			charger->curr_limit = 900;
		else if(current_setting == 700)
			charger->curr_limit = 700;
		else if(current_setting == 500)
			charger->curr_limit = 500;

		if (current_setting > 900) {
			charger->time_of_1800mA_limit = jiffies;
		} else{
			charger->time_of_1800mA_limit = 0;
		}

		retval = smb345_read(client, smb345_VRS_FUNC);
		if (retval < 0) {
			dev_err(&client->dev, "%s(): Failed in reading 0x%02x",
					__func__, smb345_VRS_FUNC);
			goto error;
		}
	}

	setting = retval | BIT(4);
	SMB_NOTICE("Re-enable AICL, setting=%x\n", setting);
	msleep(20);
	ret = smb345_write(client, smb345_VRS_FUNC, setting);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing 0x%02x to register"
			"0x%02x\n", __func__, setting, smb345_VRS_FUNC);
			goto error;
	}

	if (charge_en_flag)
		smb345_pin_control(1);

	ret = smb345_volatile_writes(client, smb345_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}

error:
	wake_unlock(&charger_wakelock);
	return ret;
}

#ifdef SMB345_STRESS_TEST
static void smb345_stress_test_work_function(struct work_struct *dat) {
	struct i2c_client *client = charger->client;
	int ret;

	ret = smb345_read(client, 0x3c);
	if (ret < 0)
		SMB_NOTICE("Failed to read testing register !!\n");

	queue_delayed_work(smb345_wq, &charger->smb345_stress_test_work, 2*HZ);
}
#endif

static irqreturn_t smb345_inok_isr(int irq, void *dev_id)
{
	SMB_NOTICE("VBUS_DET = %s\n", gpio_get_value(GPIO_AC_OK) ? "H" : "L");

	return IRQ_HANDLED;
}

static irqreturn_t smb345_wireless_isr(int irq, void *dev_id)
{
	struct smb345_charger *smb = dev_id;

	queue_delayed_work(smb345_wq, &smb->wireless_isr_work, WPC_DEBOUNCE_INTERVAL);

	return IRQ_HANDLED;
}

static int smb345_wireless_irq(struct smb345_charger *smb)
{
	int err = 0 ;
	unsigned gpio = GPIO_WPC_POK;
	unsigned irq_num = gpio_to_irq(gpio);

	err = gpio_request(gpio, "wpc_pok");
	if (err) {
		SMB_ERR("gpio %d request failed \n", gpio);
	}

	err = gpio_direction_input(gpio);
	if (err) {
		SMB_ERR("gpio %d unavaliable for input \n", gpio);
	}

	err = request_irq(irq_num, smb345_wireless_isr, IRQF_TRIGGER_FALLING |IRQF_TRIGGER_RISING|IRQF_SHARED,
		"wpc_pok", smb);
	if (err < 0) {
		SMB_ERR("%s irq %d request failed \n","wpc_pok", irq_num);
		goto fault ;
	}
	enable_irq_wake(irq_num);
	SMB_NOTICE("GPIO pin irq %d requested ok, wpc_pok = %s\n", irq_num, gpio_get_value(gpio)? "H":"L");
	return 0;

fault:
	gpio_free(gpio);
	return err;
}

int
smb345_set_WCInputCurrentlimit(struct i2c_client *client, u32 current_setting)
{
	int ret = 0, retval;
	u8 setting;

	charger->wpc_curr_limit_count++;

	ret = smb345_volatile_writes(client, smb345_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}

	if(current_setting != 0)
	{
		retval = smb345_read(client, smb345_CHRG_CRNTS);
		if (retval < 0) {
			dev_err(&client->dev, "%s(): Failed in reading 0x%02x",
				__func__, smb345_CHRG_CRNTS);
			goto error;
		}

		setting = retval & 0x0F;
		if(current_setting == 2000)
			setting |= 0x70;
		else if(current_setting == 1800)
			setting |= 0x60;
		else if(current_setting == 1200)
			setting |= 0x40;
		else if(current_setting == 900)
			setting |= 0x30;
		else if(current_setting == 700)
			setting |= 0x20;
		else if(current_setting == 500)
			setting |= 0x10;
		else if(current_setting == 300)
			setting |= 0x00;
		else
			setting |= 0x20;

		SMB_NOTICE("Set ICL=%u retval =%x setting=%x\n", current_setting, retval, setting);

		ret = smb345_write(client, smb345_CHRG_CRNTS, setting);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing 0x%02x to register"
				"0x%02x\n", __func__, setting, smb345_CHRG_CRNTS);
			goto error;
		}

		charger->wpc_curr_limit = current_setting;
	}
#ifndef FACTORY_IMAGE
	if (current_setting == 300) {
#endif
		retval = smb345_read(client, smb345_VRS_FUNC);
		if (retval < 0) {
			dev_err(&client->dev, "%s(): Failed in reading 0x%02x",
					__func__, smb345_VRS_FUNC);
			goto error;
		}

		setting = retval & (~(BIT(4)));
		SMB_NOTICE("Disable AICL, retval=%x setting=%x\n", retval, setting);
		ret = smb345_write(client, smb345_VRS_FUNC, setting);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing 0x%02x to register"
				"0x%02x\n", __func__, setting, smb345_VRS_FUNC);
			goto error;
		}
#ifndef FACTORY_IMAGE
	}
#endif

	ret = smb345_volatile_writes(client, smb345_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}

error:
	return ret;
}

static void wireless_set(void)
{
	wireless_on = true;
	wake_lock(&wireless_wakelock);
	charger->wpc_curr_limit = 300;
	charger->wpc_curr_limit_count = 0;
#ifdef FACTORY_IMAGE
	AICL_result = -1;
	smb345_set_WCInputCurrentlimit(charger->client, 900);
#else
	smb345_set_WCInputCurrentlimit(charger->client, 300);
#endif
	bq27541_wireless_callback(wireless_on);
#ifndef FACTORY_IMAGE
	queue_delayed_work(smb345_wq, &charger->wireless_set_current_work, WPC_SET_CURT_INTERVAL);
#endif
}

static void wireless_reset(void)
{
	wireless_on = false;
	if (delayed_work_pending(&charger->wireless_set_current_work))
		cancel_delayed_work(&charger->wireless_set_current_work);
#ifdef FACTORY_IMAGE
	AICL_result = -1;
	check_current_count = 0;
	if (delayed_work_pending(&charger->wireless_current_work))
		cancel_delayed_work(&charger->wireless_current_work);
#endif
	charger->wpc_curr_limit = 300;
	charger->wpc_curr_limit_count = 0;
	if (ac_on)
		smb345_set_InputCurrentlimit(charger->client, 1200, MAX_USBIN);
	bq27541_wireless_callback(wireless_on);
	if (wake_lock_active(&wireless_wakelock))
		wake_unlock(&wireless_wakelock);
}

static void wireless_isr_work_function(struct work_struct *dat)
{
	if (delayed_work_pending(&charger->wireless_isr_work))
		cancel_delayed_work(&charger->wireless_isr_work);

	SMB_NOTICE("wireless state = %d\n", wireless_is_plugged());

	if(wireless_is_plugged())
		wireless_set();
	else
		wireless_reset();
}

static void wireless_det_work_function(struct work_struct *dat)
{
	if(wireless_is_plugged())
		wireless_set();
}

static void wireless_set_current_function(struct work_struct *dat)
{
	if (delayed_work_pending(&charger->wireless_set_current_work))
		cancel_delayed_work(&charger->wireless_set_current_work);

	if (charger->wpc_curr_limit == 700 || charger->wpc_curr_limit_count >= WPC_SET_CURT_LIMIT_CNT){
		wake_unlock(&wireless_wakelock);
		return;
	} else if (charger->wpc_curr_limit == 300)
		smb345_set_WCInputCurrentlimit(charger->client, 500);
	else if (charger->wpc_curr_limit == 500)
		smb345_set_WCInputCurrentlimit(charger->client, 700);

	queue_delayed_work(smb345_wq, &charger->wireless_set_current_work, WPC_SET_CURT_INTERVAL);
}

#ifdef FACTORY_IMAGE
static void wireless_current_work_function(struct work_struct *dat)
{
	int cur = 0;

	cur = bq27541_battery_current();
	SMB_NOTICE("current = %d\n", cur);

	if (AICL_result < cur)
		AICL_result = cur;

	check_current_count++;

	if (check_current_count < 5)
		queue_delayed_work(smb345_wq, &charger->wireless_current_work, 1*HZ);
	else
		check_current_count = 0;
}
#endif

void reconfig_AICL(void)
{
	struct i2c_client *client = charger->client;

	if (ac_on && !gpio_get_value(GPIO_AC_OK)) {
		int retval;
		retval = smb345_read(client, smb345_STS_REG_E);
		if (retval < 0)
			dev_err(&client->dev, "%s(): Failed in reading 0x%02x",
			__func__, smb345_STS_REG_E);
		else {
			SMB_NOTICE("Status Reg E=0x%02x\n", retval);

			if ((retval & 0xF) <= 0x1) {
				SMB_NOTICE("reconfig input current limit\n");
				if (machine_is_apq8064_duma()) {
					usb_switch_remove_ac_path_setting();
					smb345_set_InputCurrentlimit(client, 1800, MAX_USBIN);
				}
			}
		}
	}
}
EXPORT_SYMBOL(reconfig_AICL);

static int smb345_inok_irq(struct smb345_charger *smb)
{
	int err = 0 ;
	unsigned gpio = GPIO_AC_OK;
	unsigned irq_num = gpio_to_irq(gpio);

	err = gpio_request(gpio, "smb345_inok");
	if (err) {
		SMB_ERR("gpio %d request failed \n", gpio);
	}

	err = gpio_direction_input(gpio);
	if (err) {
		SMB_ERR("gpio %d unavaliable for input \n", gpio);
	}

	err = request_irq(irq_num, smb345_inok_isr, IRQF_TRIGGER_FALLING |IRQF_TRIGGER_RISING|IRQF_SHARED,
		"smb345_inok", smb);
	if (err < 0) {
		SMB_ERR("%s irq %d request failed \n","smb345_inok", irq_num);
		goto fault ;
	}
	enable_irq_wake(irq_num);
	SMB_NOTICE("GPIO pin irq %d requested ok, smb345_INOK = %s\n", irq_num, gpio_get_value(gpio)? "H":"L");
	return 0;

fault:
	gpio_free(gpio);
	return err;
}

static int smb345_configure_otg(struct i2c_client *client)
{
	int ret = 0;

	/*Enable volatile writes to registers and Allow fast charge */
	ret = smb345_write(client, smb345_CMD_REG, 0xc0);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing"
			"register 0x%02x\n", __func__, smb345_CMD_REG);
		goto error;
	}

	if (machine_is_apq8064_duma()){
		if (asustek_get_hw_rev() == HW_REV_C ||
			asustek_get_hw_rev() == HW_REV_D ) {
			ret = smb345_read(client, smb345_SYSOK_USB3);
			if (ret < 0) {
				dev_err(&client->dev, "%s: err %d\n", __func__, ret);
				goto error;
			}
			ret = smb345_write(client, smb345_SYSOK_USB3, (ret | 0x01));
			if (ret < 0) {
				dev_err(&client->dev, "%s: err %d\n", __func__, ret);
				goto error;
			}
		}
	}

	/* Change "OTG output current limit" to 250mA */
	ret = smb345_write(client, smb345_OTG_TLIM_REG, 0x34);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing"
			"register 0x%02x\n", __func__, smb345_OTG_TLIM_REG);
		goto error;
       }

	/* Enable OTG */
       ret = smb345_update_reg(client, smb345_CMD_REG, 0x10);
       if (ret < 0) {
	       dev_err(&client->dev, "%s: Failed in writing register"
			"0x%02x\n", __func__, smb345_CMD_REG);
		goto error;
       }

	/* Change "OTG output current limit" from 250mA to 750mA */
	ret = smb345_update_reg(client, smb345_OTG_TLIM_REG, 0x08);
       if (ret < 0) {
	       dev_err(&client->dev, "%s: Failed in writing register"
			"0x%02x\n", __func__, smb345_OTG_TLIM_REG);
		goto error;
       }

	/* Change OTG to Pin control */
       ret = smb345_write(client, smb345_CTRL_REG, 0x65);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing"
			"register 0x%02x\n", __func__, smb345_CTRL_REG);
		goto error;
       }

	/* Disable volatile writes to registers */
	ret = smb345_volatile_writes(client, smb345_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s error in configuring OTG..\n",
								__func__);
	       goto error;
	}
	return 0;
error:
	return ret;
}

void smb345_otg_status(bool on)
{
	struct i2c_client *client = charger->client;
	int ret;

	SMB_NOTICE("otg function: %s\n", on? "on":"off");

	if (on) {
		otg_on = true;
		/* ENABLE OTG */
		ret = smb345_configure_otg(client);
		if (ret < 0) {
			dev_err(&client->dev, "%s() error in configuring"
				"otg..\n", __func__);
			return;
		}
	}
	else
		otg_on = false;

	if(wireless_is_plugged())
		wireless_set();
}
EXPORT_SYMBOL_GPL(smb345_otg_status);

int smb345_float_volt_set(unsigned int val)
{
	struct i2c_client *client = charger->client;
	int ret = 0, retval;

	if (val > 4500 || val < 3500) {
		SMB_ERR("%s(): val=%d is out of range !\n",__func__, val);
	}

	printk("%s(): val=%d\n",__func__, val);

	ret = smb345_volatile_writes(client, smb345_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() charger enable write error..\n", __func__);
		goto fault;
	}

	retval = smb345_read(client, smb345_FLOAT_VLTG);
	if (retval < 0) {
		dev_err(&client->dev, "%s(): Failed in reading 0x%02x",
				__func__, smb345_FLOAT_VLTG);
		goto fault;
	}
	retval = retval & (~FLOAT_VOLT_MASK);
	val = clamp_val(val, 3500, 4500) - 3500;
	val /= 20;
	retval |= val;
	ret = smb345_write(client, smb345_FLOAT_VLTG, retval);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing"
			"register 0x%02x\n", __func__, smb345_FLOAT_VLTG);
		goto fault;
       }

	ret = smb345_volatile_writes(client, smb345_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() charger disable write error..\n", __func__);
	}
	return 0;
fault:
	return ret;
}
EXPORT_SYMBOL_GPL(smb345_float_volt_set);

/* workqueue function */

int usb_cable_type_detect(unsigned int chgr_type)
{
	struct i2c_client *client = charger->client;
	int  success = 0;

	mutex_lock(&charger->usb_lock);

	if (chgr_type == CHARGER_NONE) {
		SMB_NOTICE("INOK=H\n");
		success =  bq27541_battery_callback(non_cable);
		touch_callback(non_cable);
		if (wpc_en) {
			SMB_NOTICE("enable DCIN");
			gpio_set_value(charger->wpc_en1, 0);
			gpio_set_value(charger->wpc_en2, 0);
		}
	} else {
		SMB_NOTICE("INOK=L\n");

		if (chgr_type == CHARGER_SDP) {
			SMB_NOTICE("Cable: SDP\n");
			if (machine_is_apq8064_duma())
				usb_switch_remove_usb_path_setting();
			success =  bq27541_battery_callback(usb_cable);
			touch_callback(usb_cable);
		} else {
			if (chgr_type == CHARGER_CDP) {
				SMB_NOTICE("Cable: CDP\n");
			} else if (chgr_type == CHARGER_DCP) {
				SMB_NOTICE("Cable: DCP\n");
			} else if (chgr_type == CHARGER_OTHER) {
				SMB_NOTICE("Cable: OTHER\n");
			} else if (chgr_type == CHARGER_ACA) {
				SMB_NOTICE("Cable: ACA\n");
			} else {
				SMB_NOTICE("Cable: TBD\n");
				success =  bq27541_battery_callback(usb_cable);
				touch_callback(usb_cable);
				goto done;
			}
			if (machine_is_apq8064_duma()) {
				usb_switch_remove_ac_path_setting();
				smb345_set_InputCurrentlimit(client, 1800, MAX_USBIN);
			}
			else
				smb345_set_InputCurrentlimit(client, 1200, MAX_USBIN);
			success =  bq27541_battery_callback(ac_cable);
			touch_callback(ac_cable);
			if(wpc_en) {
				if (delayed_work_pending(&charger->wireless_set_current_work))
					cancel_delayed_work(&charger->wireless_set_current_work);
				SMB_NOTICE("AC cable detect, disable DCIN");
				gpio_set_value(charger->wpc_en1, 1);
				gpio_set_value(charger->wpc_en2, 1);
			}
		}
	}

done:
	mutex_unlock(&charger->usb_lock);
	return success;
}
EXPORT_SYMBOL_GPL(usb_cable_type_detect);

/* Sysfs function */
static ssize_t smb345_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = charger->client;
	uint8_t config_reg[15], cmd_reg[2], status_reg[11];
	char tmp_buf[64];
	int i, cfg_ret, cmd_ret, sts_ret = 0;

	cfg_ret = i2c_smbus_read_i2c_block_data(client, smb345_CHARGE, 15, config_reg);
	cmd_ret = i2c_smbus_read_i2c_block_data(client, smb345_CMD_REG, 2, cmd_reg);
	sts_ret = i2c_smbus_read_i2c_block_data(client, smb345_INTR_STS_A, 11, status_reg);

	sprintf(tmp_buf, "SMB34x Configuration Registers Detail\n"
					"==================\n");
	strcpy(buf, tmp_buf);

	if (cfg_ret > 0) {
		for(i=0;i<=14;i++) {
			sprintf(tmp_buf, "Reg%02xh:\t0x%02x\n", i, config_reg[i]);
			strcat(buf, tmp_buf);
		}
	}
	if (cmd_ret > 0) {
		for(i=0;i<=1;i++) {
			sprintf(tmp_buf, "Reg%02xh:\t0x%02x\n", 48+i, cmd_reg[i]);
			strcat(buf, tmp_buf);
		}
	}
	if (sts_ret > 0) {
		for(i=0;i<=10;i++) {
			sprintf(tmp_buf, "Reg%02xh:\t0x%02x\n", 53+i, status_reg[i]);
			strcat(buf, tmp_buf);
		}
	}
	return strlen(buf);
}

#ifdef FACTORY_IMAGE
static ssize_t smb345_chg_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        struct i2c_client *client = charger->client;
        int ret;

        ret = i2c_smbus_read_byte_data(client, smb345_STS_REG_C);
        if (ret < 0)
                return sprintf(buf, "i2c read register fail\n");

        return sprintf(buf, "%d\n", (ret & 0x01));
}

static ssize_t show_smb345_charger_status(struct device *dev, struct device_attribute *devattr, char *buf)
{
	if(!smb345_charger_status)
		return sprintf(buf, "%d\n", 0);
	else
		return sprintf(buf, "%d\n", 1);
}

static ssize_t smb345_input_AICL_result_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = charger->client;
	int ret, index, result = 0;

	ret = i2c_smbus_read_byte_data(client, smb345_STS_REG_E);
	if (ret < 0)
		return sprintf(buf, "i2c read register fail\n");

	if(ret & 0x10)
	{
		index = ret & 0x0F;
		switch(index) {
		case 0:
			result = 300;
			break;
		case 1:
			result = 500;
			break;
		case 2:
			result = 700;
			break;
		case 3:
			result = 900;
			break;
		case 4:
			result = 1200;
			break;
		case 5:
			result = 1500;
			break;
		case 6:
			result = 1800;
			break;
		case 7:
			result = 2000;
			break;
		case 8:
			result = 2200;
			break;
		default:
			result = 2500;
		}
	}else
		return sprintf(buf, "AICL is not completed\n");

	return sprintf(buf, "%d\n", result);
}

static ssize_t smb345_AICL_result_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%d\n", AICL_result);
}

static ssize_t smb345_led_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
       struct i2c_client *client = charger->client;
       int ret = 0;

       ret = smb345_volatile_writes(client, smb345_ENABLE_WRITE);
       if (ret < 0) {
               dev_err(&client->dev, "%s() charger enable write error..\n", __func__);
               goto fault;
       }

       if(buf[0] == '0') {
               ret = smb345_update_reg(client, smb345_STAT_TIME_CTRL, STAT_OUTPUT_EN);
               if (ret < 0)
                       SMB_ERR("Failed to Disable STAT pin\n");
               else
                       SMB_NOTICE("Succeed to Disable STAT pin\n");
       }

       if(buf[0] == '1') {
               ret = smb345_clear_reg(client, smb345_STAT_TIME_CTRL, STAT_OUTPUT_EN);
               if (ret < 0)
                       SMB_ERR("Failed to Enable STAT pin\n");
               else
                       SMB_NOTICE("Succeed to Enable STAT pin\n");
       }

       ret = smb345_volatile_writes(client, smb345_DISABLE_WRITE);
       if (ret < 0) {
               dev_err(&client->dev, "%s() charger disable write error..\n", __func__);
       }
fault:
       return strlen(buf);
}

static ssize_t smb345_reAICL_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
       if(buf[0] == '1')
			smb345_set_InputCurrentlimit(charger->client, 0, MAX_DCIN);

       return strlen(buf);
}

static ssize_t smb345_reAICL_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", smb345_set_InputCurrentlimit(charger->client, 0, MAX_DCIN)?0:1);
}

static ssize_t smb345_check_current_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	AICL_result = -1;
	queue_delayed_work(smb345_wq, &charger->wireless_current_work, 0);
	return sprintf(buf, "1");
}
#endif

#ifdef SMB345_STRESS_TEST
static long smb345_ioctl(struct file *flip, unsigned int cmd, unsigned long arg) {

	if(_IOC_TYPE(cmd) != SMB345_IOC_MAGIC ) return -ENOTTY;
	if(_IOC_NR(cmd) > SMB345_IOC_MAXNR) return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		if( !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd)))
			 return -EFAULT;

	switch(cmd)
	{
		case SMB345_POLLING_ONOFF:
			if (arg==POLLING_START) {
				SMB_NOTICE("smb345 stress test start\n");
				queue_delayed_work(smb345_wq, &charger->smb345_stress_test_work, 2*HZ);
			}
			else if (arg==POLLING_STOP) {
				SMB_NOTICE("smb345 stress test stop\n");
				cancel_delayed_work_sync(&charger->smb345_stress_test_work);
			}
			break;
		default:
			return -ENOTTY;
	}
	return 0;
}

static int smb345_open(struct inode *inode, struct file *filp){
    printk("%s():\n", __FUNCTION__);
    return 0;
}

static struct file_operations smb345_fops = {
	.owner = THIS_MODULE,
	.open = smb345_open,
	.unlocked_ioctl = smb345_ioctl,
};

static struct miscdevice smb345_miscdev = {
	.minor      = MISC_DYNAMIC_MINOR,
	.name       = "smb345",
	.fops       = &smb345_fops,
};
#endif

static int smb345_otg_setting(struct i2c_client *client)
{
	int ret;

	/*Enable volatile writes to registers and Allow fast charge */
	ret = smb345_update_reg(client, smb345_CMD_REG, 0xc0);
       if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing"
			"register 0x%02x\n", __func__, smb345_CMD_REG);
		goto error;
       }

	/* Change OTG to I2C control */
	ret = smb345_update_reg(client, smb345_CTRL_REG, 0x25);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing"
			"register 0x%02x\n", __func__, smb345_CTRL_REG);
		goto error;
       }

	/*Enable volatile writes to registers and Allow fast charge */
	ret = smb345_update_reg(client, smb345_CMD_REG, 0xc0);
       if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing"
			"register 0x%02x\n", __func__, smb345_CMD_REG);
		goto error;
       }

	/* Change "OTG output current limit" to 250mA */
	ret = smb345_update_reg(client, smb345_OTG_TLIM_REG, 0x34);
       if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing"
			"register 0x%02x\n", __func__, smb345_OTG_TLIM_REG);
		goto error;
       }

	/* Disable volatile writes to registers */
	ret = smb345_volatile_writes(client, smb345_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s error in configuring OTG..\n",
								__func__);
	       goto error;
	}
	return 0;
error:
	return ret;
}

static int smb345_wireless_en_config(struct smb345_charger *smb)
{
	int err = 0 ;

	err = gpio_request(charger->wpc_en1, "WPC_EN1");
	if (err)
		return -ENODEV;
	err = gpio_request(charger->wpc_en2, "WPC_EN2");
	if (err)
		goto fault;

	gpio_set_value(charger->wpc_en1, 0);
	gpio_set_value(charger->wpc_en2, 0);

	SMB_NOTICE("GPIO pin %d, %d requested ok, wpc_en1 = %s, wpc_en2 = %s\n",
		charger->wpc_en1, charger->wpc_en2, gpio_get_value(charger->wpc_en1)? "H" : "L",
		gpio_get_value(charger->wpc_en2)? "H" : "L");

	return 0;
fault:
	gpio_free(charger->wpc_en1);
	return err;
}

static int __devinit smb345_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct smb345_platform_data *platform_data;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	charger = kzalloc(sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	printk("smb345_probe+\n");
	charger->client = client;
	charger->dev = &client->dev;
	i2c_set_clientdata(client, charger);

	smb345_otg_setting(charger->client);

	ret = sysfs_create_group(&client->dev.kobj, &smb345_group);
	if (ret) {
		SMB_ERR("Unable to create the sysfs\n");
		goto error;
	}

	mutex_init(&charger->apsd_lock);
	mutex_init(&charger->usb_lock);
	mutex_init(&charger->pinctrl_lock);

	smb345_wq = create_singlethread_workqueue("smb345_wq");

	INIT_DELAYED_WORK_DEFERRABLE(&charger->wireless_isr_work, wireless_isr_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&charger->wireless_det_work, wireless_det_work_function);
#ifdef FACTORY_IMAGE
	INIT_DELAYED_WORK_DEFERRABLE(&charger->wireless_current_work, wireless_current_work_function);
#endif
	INIT_DELAYED_WORK_DEFERRABLE(&charger->wireless_set_current_work, wireless_set_current_function);

	wake_lock_init(&charger_wakelock, WAKE_LOCK_SUSPEND,
			"charger_configuration");
	wake_lock_init(&wireless_wakelock, WAKE_LOCK_SUSPEND,
			"wireless_charger");
	charger->curr_limit = UINT_MAX;
	charger->wpc_curr_limit = 300;
	charger->wpc_curr_limit_count = 0;
	charger->cur_cable_type = non_cable;
	charger->old_cable_type = non_cable;
	charger->wpc_pok_gpio = -1;
	charger->wpc_en1 = -1;
	charger->wpc_en2 = -1;
	wireless_on = false;
	otg_on = false;
	wpc_en = false;

	ret = smb345_inok_irq(charger);
	if (ret) {
		SMB_ERR("Failed in requesting ACOK# pin isr\n");
		goto error;
	}

	smb345_charger_status = 1;

	#ifdef SMB345_STRESS_TEST
	INIT_DELAYED_WORK_DEFERRABLE(&charger->smb345_stress_test_work, smb345_stress_test_work_function);
	ret = misc_register(&smb345_miscdev);
	if (ret) {
		dev_err(&client->dev, "%s(): misc register smb345 miscdev fail\n", __func__);
		goto error;
	}
	#endif
	if (client->dev.platform_data) {
		platform_data = client->dev.platform_data;
		if (platform_data->wpc_pok_gpio != -1) {
			charger->wpc_pok_gpio = platform_data->wpc_pok_gpio;
			ret = smb345_wireless_irq(charger);
			if (ret) {
				SMB_ERR("Failed in requesting WPC_POK# pin isr\n");
				goto error;
			}
		}
		if (platform_data->wpc_en1 != -1 && platform_data->wpc_en2 != -1) {
			charger->wpc_en1 = platform_data->wpc_en1;
			charger->wpc_en2 = platform_data->wpc_en2;
			wpc_en = true;
			ret = smb345_wireless_en_config(charger);
			if (ret) {
				SMB_ERR("Failed in config WPC en gpio\n");
				goto error;
			}
		}
		queue_delayed_work(smb345_wq, &charger->wireless_det_work, WPC_INIT_DET_INTERVAL);
	}

	printk("smb345_probe-\n");

	return 0;
error:
	kfree(charger);
	return ret;
}

static int __devexit smb345_remove(struct i2c_client *client)
{
	struct smb345_charger *charger = i2c_get_clientdata(client);
	kfree(charger);
	return 0;
}

static int smb345_suspend(struct i2c_client *client, pm_message_t mesg)
{
	printk("smb345_suspend+\n");
	flush_workqueue(smb345_wq);
	printk("smb345_suspend-\n");
	return 0;
}

static int smb345_resume(struct i2c_client *client)
{
	return 0;
}

void smb345_shutdown(struct i2c_client *client)
{
	printk("smb345_shutdown+\n");
	printk("smb345_shutdown-\n");
}

static const struct i2c_device_id smb345_id[] = {
	{ "smb345", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, smb345_id);

static struct i2c_driver smb345_i2c_driver = {
	.driver	= {
		.name	= "smb345",
	},
	.probe		= smb345_probe,
	.remove		= __devexit_p(smb345_remove),
	.shutdown	= smb345_shutdown,
	.suspend 		= smb345_suspend,
	.resume 		= smb345_resume,
	.id_table	= smb345_id,
};

static int __init smb345_init(void)
{
	return i2c_add_driver(&smb345_i2c_driver);
}
module_init(smb345_init);

static void __exit smb345_exit(void)
{
	#ifdef SMB345_STRESS_TEST
	misc_deregister(&smb345_miscdev);
	#endif
	i2c_del_driver(&smb345_i2c_driver);
}
module_exit(smb345_exit);

MODULE_DESCRIPTION("smb345 Battery-Charger");
MODULE_LICENSE("GPL");
