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

/**************************/
/* Debug and Log System */
/************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/input/ASH.h>

int proximity_hi_cal = -1;
int proximity_lo_cal = -1;
int light_200_cal=-1;
int light_1000_cal=-1;
int light_shift = 5;
int light_gain = 1;
int proximity_adc = 50;
int light_adc = 200;
bool proximity_switch = false;
bool light_switch = false;
bool proximity_poll_mode = false;
int light_sensitivity = -1;

bool hall_enable=false;
int hall_debounce=500;

int mproximity_show_calibration_hi(void)
{
	return proximity_hi_cal;
}

int mproximity_store_calibration_hi(int calvalue)
{
	proximity_hi_cal = calvalue;
	return 0;
}

int mproximity_show_calibration_lo(void)
{
	return proximity_lo_cal;
}

int mproximity_store_calibration_lo(int calvalue)
{
	proximity_lo_cal = calvalue;
	return 0;
}

int mlight_show_calibration_200lux(void)
{
	return light_200_cal;
}

int mlight_store_calibration_200lux(int calvalue)
{
	light_200_cal = calvalue;
	return 0;
}

int mlight_show_calibration_1000lux(void)
{
	return light_1000_cal;
}

int mlight_store_calibration_1000lux(int calvalue)
{
	light_1000_cal = calvalue;
	return 0;
}

int mlight_show_shift(void)
{
	return light_shift;
}

int mlight_show_gain(void)
{
	return light_gain;
}

int mlight_show_adc(void)
{
	return light_adc;
}

int mproximity_show_adc(void)
{
	return proximity_adc;
}

static IRsensor_ATTR_Calibration mATTR_Calibration={
	.proximity_show_calibration_hi 	=mproximity_show_calibration_hi,
	.proximity_store_calibration_hi 	=mproximity_store_calibration_hi,
	.proximity_show_calibration_lo 	=mproximity_show_calibration_lo,
	.proximity_store_calibration_lo 	=mproximity_store_calibration_lo,
	.light_show_calibration_200lux	= mlight_show_calibration_200lux,
	.light_store_calibration_200lux	=mlight_store_calibration_200lux,
	.light_show_calibration_1000lux	=mlight_show_calibration_1000lux,
	.light_store_calibration_1000lux	=mlight_store_calibration_1000lux,
	.light_show_shift		=mlight_show_shift,
	.light_show_gain		=mlight_show_gain,
	.light_show_adc		=mlight_show_adc,
	.proximity_show_adc	=mproximity_show_adc,
};

bool mproximity_show_atd_test(void)
{
	return false;
}

bool mlight_show_atd_test(void)
{
	return true;
}

static IRsensor_ATTR_BMMI mATTR_BMMI={
	.proximity_show_atd_test	=mproximity_show_atd_test,
	.light_show_atd_test			=mlight_show_atd_test,
};

int mIRsensor_show_reg(uint8_t addr)
{
	uint8_t value=0;

	if(addr == 0x02)
		value=0x1e;
	else
		value=0x2f;

	log("mIRsensor_show_reg, addr=%02X, value=%02X.\n", addr, value);
	return value;
}

int mIRsensor_store_reg(uint8_t addr, int value)
{	
	log("mIRsensor_store_reg, addr=%02X, value=%02X.\n", addr, value);
	return 0;
}

static IRsensor_ATTR_Hardware mATTR_Hardware={
	.IRsensor_show_reg	=mIRsensor_show_reg,
	.IRsensor_store_reg	=mIRsensor_store_reg,
};

bool mproximity_show_switch_onoff(void)
{
	return proximity_switch;
}

int mproximity_store_switch_onoff(bool bOn)
{
	proximity_switch=bOn;
	return 0;
}

bool mproximity_show_status(void)
{
	return false;
}

bool mlight_show_switch_onoff(void)
{
	return light_switch;
}

int mlight_store_switch_onoff(bool bOn)
{
	light_switch=bOn;
	return 0;
}

int mlight_show_lux(void)
{
	return 9527;
}

static IRsensor_ATTR_HAL mATTR_HAL={
	.proximity_show_switch_onoff	=mproximity_show_switch_onoff,
	.proximity_store_switch_onoff	=mproximity_store_switch_onoff,
	.proximity_show_status			=mproximity_show_status,
	.light_show_switch_onoff			=mlight_show_switch_onoff,
	.light_store_switch_onoff		=mlight_store_switch_onoff,
	.light_show_lux					=mlight_show_lux,
};

bool mIRsensor_show_allreg(void)
{	
	log("use log to dump register.\n");
	return true;
}

bool mproximity_show_polling_mode(void)
{
	return proximity_poll_mode;
}

int mproximity_store_polling_mode(bool bOn)
{
	proximity_poll_mode=bOn;
	return 0;
}

int mlight_show_sensitivity(void)
{
	return light_sensitivity;
}

int mlight_store_sensitivity(int sensitivity)
{
	light_sensitivity = sensitivity;
	return 0;
}

static IRsensor_ATTR_Extension mATTR_Extension={
	//.IRsensor_show_allreg			=mIRsensor_show_allreg,
	.proximity_show_polling_mode	=mproximity_show_polling_mode,
	.proximity_store_polling_mode	=mproximity_store_polling_mode,
	.light_show_sensitivity			=mlight_show_sensitivity,
	.light_store_sensitivity			=mlight_store_sensitivity,
};

static IRsensor_ATTR mIRsensor_ATTR={
	.info_type={
		.vendor="ASUS",
	},
	.ATTR_Calibration=&mATTR_Calibration,
	.ATTR_BMMI=&mATTR_BMMI,
	.ATTR_Hardware=&mATTR_Hardware,
	.ATTR_HAL=&mATTR_HAL,
	.ATTR_Extension=&mATTR_Extension,
};

bool mshow_action_status(void)
{
	return true;
}

bool mshow_hall_sensor_enable(void)
{
	return hall_enable;
}

int mstore_hall_sensor_enable(bool bOn)
{
	hall_enable=bOn;
	return 0;
}

int mshow_hall_sensor_debounce(void)
{
	return hall_debounce;
}

int mstore_hall_sensor_debounce(int debounce)
{
	hall_debounce=debounce;
	return 0;
}

static HALLsensor_ATTR mHALLsensor_ATTR={
	.show_action_status=mshow_action_status,
	.show_hall_sensor_enable=mshow_hall_sensor_enable,
	.store_hall_sensor_enable=mstore_hall_sensor_enable,
	.show_hall_sensor_debounce=mshow_hall_sensor_debounce,
	.store_hall_sensor_debounce=mstore_hall_sensor_debounce,
};

static struct device_attribute hall_customize = 
	__ATTR(srTest, 0660, NULL, NULL);


bool proximity_check_status(void)
{
	
	return true;
}

EXPORT_SYMBOL(proximity_check_status);


static int __init testDriver_init(void)
{
	log("Driver INIT +++\n");

	IRsensor_ATTR_register(&mIRsensor_ATTR);
	HALLsensor_ATTR_register(&mHALLsensor_ATTR);		

	HALLsensor_ATTR_create(&hall_customize);

	IRsensor_report_register();
	
	log("Driver INIT ---\n");
	return 0;
}

static void __exit testDriver_exit(void)
{
	log("Driver EXIT +++\n");

	
	log("Driver EXIT ---\n");
}

module_init(testDriver_init);
module_exit(testDriver_exit);

MODULE_AUTHOR("sr_Huang <sr_Huang@asus.com>");
MODULE_DESCRIPTION("Asus Sensor Hub Test Driver");
MODULE_LICENSE("GPL");
