/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include "msm_camera_eeprom.h"
#include "msm_camera_i2c.h"

DEFINE_MUTEX(ov5693_eeprom_mutex);
static struct msm_eeprom_ctrl_t ov5693_e_ctrl;
#define SENSOR_MAX_RETRIES      3 /* max counter for retry I2C access */

uint8_t ov5693_afcalib_data[10];
struct msm_calib_af ov5693_af_data;

static struct msm_camera_eeprom_info_t ov5693_calib_supp_info = {
	{TRUE, 10, 0, 1},
	{FALSE, 0, 0, 1},
	{FALSE, 0, 0, 1},
	{FALSE, 0, 0, 1},
};

static struct msm_camera_eeprom_read_t ov5693_eeprom_read_tbl[] = {
	{0x3D00, &ov5693_afcalib_data[0], 10, 0},
};

static struct msm_camera_eeprom_data_t ov5693_eeprom_data_tbl[] = {
	{&ov5693_af_data, sizeof(struct msm_calib_af)},
};

static int sensor_read_reg(struct i2c_client *client, u16 addr, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[4];
	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = 0x10;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = 0x10;
	msg[1].flags = I2C_M_RD;

	msg[1].len = 1;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	memcpy(val, data+2, 1);
	*val=*val&0xff;

	return 0;
}

static int sensor_write_reg(struct i2c_client *client, u16 addr, u16 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[4];
	int retry = 0;
	if (!client->adapter){
		pr_info("%s client->adapter is null",__func__);
		return -ENODEV;
	}

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);

	msg.addr = 0x10;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;
	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("yuv_sensor : i2c transfer failed, retrying %x %x\n",
		       addr, val);
	} while (retry <= SENSOR_MAX_RETRIES);

	if(err == 0) {
		printk("%s(%d): i2c_transfer error, but return 0!?\n", __FUNCTION__, __LINE__);
		err = 0xAAAA;
	}

	return err;
}

void ov5693_format_afdata(void)
{
	ov5693_af_data.inf_dac = (uint16_t)(ov5693_afcalib_data[0] << 8) |
		ov5693_afcalib_data[1];
	ov5693_af_data.macro_dac = (uint16_t)(ov5693_afcalib_data[4] << 8) |
		ov5693_afcalib_data[5];
	ov5693_af_data.start_dac = (uint16_t)(ov5693_afcalib_data[2] << 8) |
		ov5693_afcalib_data[3];
	pr_info("%s: inf_dac:0x%x macro_dac:0x%x start_dac:0x%x\n", __func__, ov5693_af_data.inf_dac, ov5693_af_data.macro_dac,
		ov5693_af_data.start_dac);
}

static void ov5693_format_calibrationdata(void)
{
	ov5693_format_afdata();
}

int32_t ov5693_opt_read_tbl(struct msm_eeprom_ctrl_t *ectrl,
	struct msm_camera_eeprom_read_t *read_tbl, uint16_t tbl_size)
{
	int i, rc = 0;
	unsigned short rdata = 0;

	if (read_tbl == NULL)
		return rc;

	sensor_write_reg(ov5693_e_ctrl.i2c_client.client , 0x0100, 0x01);
	msleep(3);

	sensor_write_reg(ov5693_e_ctrl.i2c_client.client , 0x3D84, 0xc3);
	sensor_write_reg(ov5693_e_ctrl.i2c_client.client , 0x3D81, 0x01);
	msleep(3);
	sensor_read_reg(ov5693_e_ctrl.i2c_client.client , 0x3D08, &rdata);
	if(rdata == 0){
		sensor_write_reg(ov5693_e_ctrl.i2c_client.client , 0x3D84, 0xc2);
		sensor_write_reg(ov5693_e_ctrl.i2c_client.client , 0x3D81, 0x01);
		msleep(3);
		sensor_read_reg(ov5693_e_ctrl.i2c_client.client , 0x3D08, &rdata);
		if(rdata == 0){
			sensor_write_reg(ov5693_e_ctrl.i2c_client.client , 0x3D84, 0xc1);
			sensor_write_reg(ov5693_e_ctrl.i2c_client.client , 0x3D81, 0x01);
			msleep(3);
			sensor_read_reg(ov5693_e_ctrl.i2c_client.client , 0x3D08, &rdata);
			if(rdata == 0)
				return rc;
		}
	}

	for (i = 0; i <  read_tbl[0].num_byte; i++) {
		rc = sensor_read_reg(ov5693_e_ctrl.i2c_client.client ,
			read_tbl[0].reg_addr + i, read_tbl[0].dest_ptr + i);
		if (rc < 0) {
			pr_err("%s: read failed\n", __func__);
			return rc;
		}
	}

	sensor_write_reg(ov5693_e_ctrl.i2c_client.client , 0x0100, 0x01);
	msleep(3);

	return rc;
}

int32_t ov5693_eeprom_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_eeprom_ctrl_t *e_ctrl_t = NULL;
	pr_info("%s called\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	e_ctrl_t = (struct msm_eeprom_ctrl_t *)(id->driver_data);
	e_ctrl_t->i2c_client.client = client;
	ov5693_e_ctrl.i2c_client.client = client;

	if (e_ctrl_t->i2c_addr != 0)
		e_ctrl_t->i2c_client.client->addr = e_ctrl_t->i2c_addr;

	/* Assign name for sub device */
	snprintf(e_ctrl_t->sdev.name, sizeof(e_ctrl_t->sdev.name),
		"%s", e_ctrl_t->i2c_driver->driver.name);

	if (e_ctrl_t->func_tbl.eeprom_init != NULL) {
		rc = e_ctrl_t->func_tbl.eeprom_init(e_ctrl_t,
			e_ctrl_t->i2c_client.client->adapter);
	}
	ov5693_opt_read_tbl(e_ctrl_t,
		e_ctrl_t->read_tbl,
		e_ctrl_t->read_tbl_size);

	if (e_ctrl_t->func_tbl.eeprom_format_data != NULL)
		e_ctrl_t->func_tbl.eeprom_format_data();

	if (e_ctrl_t->func_tbl.eeprom_release != NULL)
		rc = e_ctrl_t->func_tbl.eeprom_release(e_ctrl_t);


	/* Initialize sub device */
	v4l2_i2c_subdev_init(&e_ctrl_t->sdev,
		e_ctrl_t->i2c_client.client,
		e_ctrl_t->eeprom_v4l2_subdev_ops);
	return rc;

probe_failure:
	pr_err("%s failed! rc = %d\n", __func__, rc);
	return rc;
}



static const struct i2c_device_id ov5693_eeprom_i2c_id[] = {
	{"ov5693_eeprom", (kernel_ulong_t)&ov5693_e_ctrl},
	{ }
};

static struct i2c_driver ov5693_eeprom_i2c_driver = {
	.id_table = ov5693_eeprom_i2c_id,
	.probe  = ov5693_eeprom_i2c_probe,
	.remove = __exit_p(ov5693_eeprom_i2c_remove),
	.driver = {
		.name = "ov5693_eeprom",
	},
};

static int __init ov5693_eeprom_i2c_add_driver(void)
{
	int rc = 0;
	rc = i2c_add_driver(ov5693_e_ctrl.i2c_driver);
	return rc;
}

static struct v4l2_subdev_core_ops ov5693_eeprom_subdev_core_ops = {
	.ioctl = msm_eeprom_subdev_ioctl,
};

static struct v4l2_subdev_ops ov5693_eeprom_subdev_ops = {
	.core = &ov5693_eeprom_subdev_core_ops,
};

static struct msm_eeprom_ctrl_t ov5693_e_ctrl = {
	.i2c_driver = &ov5693_eeprom_i2c_driver,
	.i2c_addr = 0x11,
	.eeprom_v4l2_subdev_ops = &ov5693_eeprom_subdev_ops,

	.i2c_client = {
		.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	},

	.eeprom_mutex = &ov5693_eeprom_mutex,

	.func_tbl = {
		.eeprom_init = NULL,
		.eeprom_release = NULL,
		.eeprom_get_info = msm_camera_eeprom_get_info,
		.eeprom_get_data = msm_camera_eeprom_get_data,
		.eeprom_set_dev_addr = NULL,
		.eeprom_format_data = ov5693_format_calibrationdata,
	},
	.info = &ov5693_calib_supp_info,
	.info_size = sizeof(struct msm_camera_eeprom_info_t),
	.read_tbl = ov5693_eeprom_read_tbl,
	.read_tbl_size = ARRAY_SIZE(ov5693_eeprom_read_tbl),
	.data_tbl = ov5693_eeprom_data_tbl,
	.data_tbl_size = ARRAY_SIZE(ov5693_eeprom_data_tbl),
};

subsys_initcall(ov5693_eeprom_i2c_add_driver);
MODULE_DESCRIPTION("ov5693 EEPROM");
MODULE_LICENSE("GPL v2");

