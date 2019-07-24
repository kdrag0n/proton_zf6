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

/*****************************************/
/* Proximity Sensor Report Module */
/***************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/input/ASH.h>

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_Report"
#define SENSOR_TYPE_NAME		"SAR"
#define SAR_SENSOR_INIT -1

#undef dbg
#ifdef ASH_REPORT_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

/******************/
/*Global Variables*/
/*****************/
static struct input_dev *input_dev_sar = NULL;

int SAR_sensor_report_register(void)
{
	int ret = 0;
	
	/* SAR Input event allocate */
	input_dev_sar = input_allocate_device();
	if (!input_dev_sar) {		
		err("%s: SAR sensor input_allocate_device is return NULL Pointer. \n", __FUNCTION__);
		return -ENOMEM;
	}

	/* Set SAR input device */
	input_dev_sar->name = "ASUS SARsensor";
	input_dev_sar->id.bustype = BUS_I2C;
	input_set_capability(input_dev_sar, EV_ABS, ABS_DISTANCE);
	__set_bit(EV_ABS, input_dev_sar->evbit);
	__set_bit(ABS_DISTANCE, input_dev_sar->absbit);
	input_set_abs_params(input_dev_sar, ABS_DISTANCE, 0, 1, 0, 0);
	//input_set_drvdata(input_dev_ps, g_ps_data);

	/* Register SAR input device */
	ret = input_register_device(input_dev_sar);
	if (ret < 0) {
		err("%s: sar sensor input_register_device ERROR(%d). \n", __FUNCTION__, ret);
		return -1;
	}

	dbg("Input Event Success Registration\n");
	return 0;
}
EXPORT_SYMBOL(SAR_sensor_report_register);

void SAR_sensor_report_unregister(void)
{	
	input_unregister_device(input_dev_sar);
	input_free_device(input_dev_sar);	
}
EXPORT_SYMBOL(SAR_sensor_report_unregister);

void SAR_sensor_report_abs(int abs)
{

	if(abs != SAR_SENSOR_REPORT_AWAY &&
		abs != SAR_SENSOR_REPORT_CLOSE) {
		if (abs != SAR_SENSOR_INIT) {
			err("%s: Sar Detect Object ERROR.\n", __FUNCTION__);
		}
	}
	input_report_abs(input_dev_sar, ABS_DISTANCE, abs);
	input_sync(input_dev_sar);
}
EXPORT_SYMBOL(SAR_sensor_report_abs);

