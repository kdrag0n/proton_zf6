/* 
 * Copyright (C) 2015 ASUSTek Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*****************************/
/* IR Sensor Report Module */
/***************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/input/ASH.h>

/*******************************/
/* Debug and Log System */
/*****************************/
#define MODULE_NAME			"ASH_Report"
#define SENSOR_TYPE_NAME		"light"

#undef dbg
#ifdef ASH_REPORT_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

/**********************/
/*Global Variables*/
/********************/
static struct input_dev *input_dev_als = NULL;

int lsensor_report_register(void)
{
	int ret = 0;

	/* Light Sensor Input event allocate */
	input_dev_als = input_allocate_device();
	if (!input_dev_als) {
		err("%s: lsensor input_allocate_device is return NULL Pointer. \n", __FUNCTION__);
		return -ENOMEM;
	}

	/* Set Light Sensor input device */
	input_dev_als->name = "ASUS Lightsensor";
	input_dev_als->id.bustype = BUS_I2C;

	input_set_capability(input_dev_als, EV_ABS, ABS_MISC);
	__set_bit(EV_ABS, input_dev_als->evbit);
	__set_bit(ABS_MISC, input_dev_als->absbit);
	input_set_abs_params(input_dev_als, ABS_MISC, 0, 65535, 0, 0);
	//input_set_drvdata(input_dev_als, g_als_data);

	/* Register Light Sensor input device */
	ret = input_register_device(input_dev_als);
	if (ret < 0) {
		err("%s: lsensor input_register_device ERROR(%d). \n", __FUNCTION__, ret);
		return -1;
	}

	dbg("Input Event Success Registration\n");
	return 0;
}
EXPORT_SYMBOL(lsensor_report_register);

void lsensor_report_unregister(void)
{	
	input_unregister_device(input_dev_als);
	input_free_device(input_dev_als);
}
EXPORT_SYMBOL(lsensor_report_unregister);

void lsensor_report_lux(int lux)
{
	input_report_abs(input_dev_als, ABS_MISC, lux);
	input_event(input_dev_als, EV_SYN, SYN_REPORT, 1);
	input_sync(input_dev_als);
}
EXPORT_SYMBOL(lsensor_report_lux);
