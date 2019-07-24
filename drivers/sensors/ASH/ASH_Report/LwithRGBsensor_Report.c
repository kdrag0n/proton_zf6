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
static int report_data[5] = {-1};
void lrgbsensor_report_lux(void);

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

	/* R raw data */
	input_set_capability(input_dev_als, EV_ABS, ABS_HAT0X);
	__set_bit(ABS_HAT0X, input_dev_als->absbit);
	input_set_abs_params(input_dev_als, ABS_HAT0X, 0, 65535, 0, 0);

	/* G raw data */
	input_set_capability(input_dev_als, EV_ABS, ABS_HAT0Y);
	__set_bit(ABS_HAT0Y, input_dev_als->absbit);
	input_set_abs_params(input_dev_als, ABS_HAT0Y, 0, 65535, 0, 0);

	/* B raw data */
	input_set_capability(input_dev_als, EV_ABS, ABS_HAT1X);
	__set_bit(ABS_HAT1X, input_dev_als->absbit);
	input_set_abs_params(input_dev_als, ABS_HAT1X, 0, 65535, 0, 0);

	/* IR raw data */
	input_set_capability(input_dev_als, EV_ABS, ABS_HAT1Y);
	__set_bit(ABS_HAT1Y, input_dev_als->absbit);
	input_set_abs_params(input_dev_als, ABS_HAT1Y, 0, 65535, 0, 0);

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
	report_data[0] = lux;
	lrgbsensor_report_lux();
}
EXPORT_SYMBOL(lsensor_report_lux);

void FRGBsensor_report_raw(int *data, int size)
{
	report_data[1] = data[0];	/* R */
	report_data[2] = data[1];	/* G */
	report_data[3] = data[2];	/* B */
	report_data[4] = data[3];	/* IR */
	lrgbsensor_report_lux();
}
EXPORT_SYMBOL(FRGBsensor_report_raw);

void lrgbsensor_report_lux(void)
{
	input_report_abs(input_dev_als, ABS_MISC, report_data[0]);		/* LUX */
	input_report_abs(input_dev_als, ABS_HAT0X, report_data[1]);	/* R */
	input_report_abs(input_dev_als, ABS_HAT0Y, report_data[2]);	/* G */
	input_report_abs(input_dev_als, ABS_HAT1X, report_data[3]);	/* B */
	input_report_abs(input_dev_als, ABS_HAT1Y, report_data[4]);	/* IR */
	input_event(input_dev_als, EV_SYN, SYN_REPORT, 5);
	input_sync(input_dev_als);
}

