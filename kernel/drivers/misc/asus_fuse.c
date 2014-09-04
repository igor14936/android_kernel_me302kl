/*
*drivers/misc/asus_fuse.c
*
* Copyright (C) 2013 ASUSTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/kernel.h>

unsigned fuse_info = 0;
static int __init get_fuse_info(char *p)
{
	if(*p == 'Y')
		fuse_info = 1;
	else if (*p == 'N')
                fuse_info = 0;
	else
		printk("No fuse Info!!");

	printk("FuseInfo = %d\n" , fuse_info);
	return 0;
}
early_param("fuse_info", get_fuse_info);

static ssize_t fuse_info_show(struct device *class, struct device_attribute *attr, char *buf){

	return sprintf(buf,"%d\n",fuse_info);
}
static DEVICE_ATTR(fuse_info, S_IRUSR | S_IRGRP| S_IROTH, fuse_info_show, NULL);
static struct attribute *asus_fuse_attributes[] = {
	&dev_attr_fuse_info.attr,
	NULL,
};
static const struct attribute_group asus_fuse_group = {
	.attrs = asus_fuse_attributes,
};
static int __init fuseinfo_driver_probe(struct platform_device *pdev){
	int ret = 0;
	if (!pdev)
		return -EINVAL;
	/* create a sysfs interface */
	ret = sysfs_create_group(&pdev->dev.kobj, &asus_fuse_group);
	if (ret)
		pr_err("ASUSTek: Failed to create sysfs group\n");
	return ret;
}

static int __devexit fuseinfo_driver_remove(struct platform_device *pdev){
	return 0;
}

static struct platform_driver asus_fuseinfo_driver __refdata = {
	.driver = {
		.name = "asus_fuseinfo",
		.owner = THIS_MODULE,
	},
	.probe = fuseinfo_driver_probe,
	.remove = fuseinfo_driver_remove,
};

static int __init asus_fuse_init(void){
	return platform_driver_register(&asus_fuseinfo_driver);
}

static void __exit asus_fuse_exit(void){
	platform_driver_unregister(&asus_fuseinfo_driver);
}

module_init(asus_fuse_init);
module_exit(asus_fuse_exit);

MODULE_DESCRIPTION("ASUSTek FuseInfo driver");
MODULE_AUTHOR("William Shih <william_shih@asus.com>");
MODULE_LICENSE("GPL");
