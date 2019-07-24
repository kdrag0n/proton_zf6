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

/******************************************/
/* Front RGB Sensor Report Module */
/*****************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/input/ASH.h>

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_Report"
#define SENSOR_TYPE_NAME		"FRGB"

#undef dbg
#ifdef ASH_REPORT_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

/*********************/
/*Global Variables*/
/********************/
static struct input_dev *input_dev_frgb = NULL;

int FRGBsensor_report_register(void)
{
	int ret = 0;

	/* FRGB Sensor Input event allocate */
	input_dev_frgb = input_allocate_device();
	if (!input_dev_frgb) {
		err("%s: FRGB input_allocate_device is return NULL Pointer. \n", __FUNCTION__);
		return -ENOMEM;
	}

	/* Set FRGB Sensor input device */
	input_dev_frgb->name = "ASUS FRGBsensor";
	input_dev_frgb->id.bustype = BUS_I2C;

	/* R raw data */
	input_set_capability(input_dev_frgb, EV_ABS, ABS_HAT0X);
	__set_bit(ABS_HAT0X, input_dev_frgb->absbit);
	input_set_abs_params(input_dev_frgb, ABS_HAT0X, 0, 65535, 0, 0);

	/* G raw data */
	input_set_capability(input_dev_frgb, EV_ABS, ABS_HAT0Y);
	__set_bit(ABS_HAT0Y, input_dev_frgb->absbit);
	input_set_abs_params(input_dev_frgb, ABS_HAT0Y, 0, 65535, 0, 0);

	/* B raw data */
	input_set_capability(input_dev_frgb, EV_ABS, ABS_HAT1X);
	__set_bit(ABS_HAT1X, input_dev_frgb->absbit);
	input_set_abs_params(input_dev_frgb, ABS_HAT1X, 0, 65535, 0, 0);

	/* IR raw data */
	input_set_capability(input_dev_frgb, EV_ABS, ABS_HAT1Y);
	__set_bit(ABS_HAT1Y, input_dev_frgb->absbit);
	input_set_abs_params(input_dev_frgb, ABS_HAT1Y, 0, 65535, 0, 0);

	/* Register FRGB Sensor input device */
	ret = input_register_device(input_dev_frgb);
	if (ret < 0) {
		err("%s: FRGB input_register_device ERROR(%d). \n", __FUNCTION__, ret);
		return -1;
	}

	dbg("Input Event Success Registration\n");
	return 0;
}
EXPORT_SYMBOL(FRGBsensor_report_register);

void FRGBsensor_report_unregister(void)
{	
	input_unregister_device(input_dev_frgb);
	input_free_device(input_dev_frgb);
}
EXPORT_SYMBOL(FRGBsensor_report_unregister);

void FRGBsensor_report_raw(int *data, int size)
{
	input_report_abs(input_dev_frgb, ABS_HAT0X, data[0]);	/* R */
	input_report_abs(input_dev_frgb, ABS_HAT0Y, data[1]);	/* G */
	input_report_abs(input_dev_frgb, ABS_HAT1X, data[2]);	/* B */
	input_report_abs(input_dev_frgb, ABS_HAT1Y, data[3]);	/* IR */
	input_event(input_dev_frgb, EV_SYN, SYN_REPORT, 4);
	input_sync(input_dev_frgb);
}
EXPORT_SYMBOL(FRGBsensor_report_raw);
