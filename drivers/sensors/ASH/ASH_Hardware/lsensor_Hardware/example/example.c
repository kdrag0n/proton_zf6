/* 
 * Copyright (C) 2014 ASUSTek Inc.
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

/********************************/
/* IR Sensor CM36686 Module */
/******************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/input/ASH.h>
#include "example.h"

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_HW"
#define SENSOR_TYPE_NAME		"IRsensor"

#undef dbg
#ifdef ASH_HW_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)


static struct psensor_hw psensor_hw_example = {
	.proximity_low_threshold_default = EXAMPLE_PROXIMITY_THDL_DEFAULT,
	.proximity_hi_threshold_default = EXAMPLE_PROXIMITY_THDH_DEFAULT,

};

static struct lsensor_hw lsensor_hw_example = {
	.light_max_threshold = EXAMPLE_LIGHT_MAX_THRESHOLD,
	
};

static struct IRsensor_hw IRsensor_hw_example = {	
	.vendor = "ASUS",
	.module_number = "example",

	.mpsensor_hw = &psensor_hw_example,
	.mlsensor_hw = &lsensor_hw_example,
};

IRsensor_hw* IRsensor_hw_example_getHardware(void)
{
	IRsensor_hw* IRsensor_hw_client = NULL;
	IRsensor_hw_client = &IRsensor_hw_example;
	return IRsensor_hw_client;
}
