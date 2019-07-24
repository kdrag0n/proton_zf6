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

/*******************************************************/
/* ALSPS Front RGB Sensor CM36656 Module */
/******************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/input/ASH.h>
#include "example.h"

/******************************/
/* Debug and Log System */
/*****************************/
#define MODULE_NAME			"ASH_HW"
#define SENSOR_TYPE_NAME	"ALSPS_FRGB"

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
	.proximity_crosstalk_default = EXAMPLE_PROXIMITY_INF_DEFAULT,
	.proximity_autok_min = EXAMPLE_PROXIMITY_AUTOK_MIN,
	.proximity_autok_max = EXAMPLE_PROXIMITY_AUTOK_MAX,

};

static struct lsensor_hw lsensor_hw_example = {
	.light_max_threshold = EXAMPLE_LIGHT_MAX_THRESHOLD,
	.light_200lux_default = EXAMPLE_LIGHT_200LUX_DEFAULT,
	.light_1000lux_default = EXAMPLE_LIGHT_1000LUX_DEFAULT,
	
};

static struct FRGB_hw FRGB_hw_example = {
	.frgb_red_ratio_default = EXAMPLE_FRGB_RED_RATIO_DEFAULT,
	.frgb_green_ratio_default = EXAMPLE_FRGB_GREEN_RATIO_DEFAULT,
	.frgb_blue_ratio_default = EXAMPLE_FRGB_BLUE_RATIO_DEFAULT,
	
};

static struct ALSPS_FRGB_hw ALSPS_FRGB_hw_example = {	
	.vendor = "ASUS",
	.module_number = "example",

	.mpsensor_hw = &psensor_hw_example,
	.mlsensor_hw = &lsensor_hw_example,
	.mFRGB_hw = &FRGB_hw_example,
};

ALSPS_FRGB_hw* ALSPS_FRGB_hw_example_getHardware(void)
{
	ALSPS_FRGB_hw* ALSPS_FRGB_hw_client = NULL;
	ALSPS_FRGB_hw_client = &ALSPS_FRGB_hw_example;
	return ALSPS_FRGB_hw_client;
}
