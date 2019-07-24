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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/input/ASH.h>
#include "IRsensor.h"

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_ALGO"
#define SENSOR_TYPE_NAME		"Lsensor"
#define SW_OFFSET 1
#undef dbg
#ifdef ASH_ALGO_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) do{	\
		printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args);	\
		sprintf(g_error_mesg, "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args);	\
	}while(0)

/******************************/
/* IR Sensor Global Variables */
/*****************************/
#define MAX_ADC 65535	/* Max adc value */
#define MAX_DIFF 19000	/* 20000 lux - 1000 lux */
static int ASUS_IR_SENSOR_IRQ;
static int ASUS_IR_SENSOR_INT;
static struct ASUS_light_sensor_data			*g_als_data;
static struct IRsensor_hw						*Lsensor_hw_client;
static struct workqueue_struct 					*Lsensor_workqueue;
static struct workqueue_struct 					*Lsensor_delay_workqueue;
static struct mutex 								g_ir_lock;
static struct mutex 								g_i2c_lock;
static struct wake_lock 							g_ir_wake_lock;
static struct i2c_client *g_i2c_client;
static int g_als_last_lux = 0;
static char *g_error_mesg;
static int resume_flag = 0;

/**********************/
/* L Sensor Functions*/
/*********************/
/*Device Layer Part*/
static int 	light_turn_onoff(bool bOn);
static int 	light_get_lux(int adc);
static int 	light_get_shift(void);
static int 	light_get_accuracy_gain(void);
static void light_polling_lux(struct work_struct *work);

/*Interrupt Service Routine Part*/
static void Lsensor_ist(struct work_struct *work);

/*Initialization Part*/
static int init_data(void);

/*Work Queue*/
static 		DECLARE_WORK(Lsensor_ist_work, Lsensor_ist);
static 		DECLARE_DELAYED_WORK(light_polling_lux_work, light_polling_lux);

/*******************************/
/* ALS data structure */
/******************************/
struct ASUS_light_sensor_data 
{	
	int g_als_calvalue_200lux;				/* Lightsensor 200lux calibration value(adc) */
	int g_als_calvalue_1000lux;				/* Lightsensor 1000lux calibration value(adc) */
	int g_als_calvalue_shift;					/* Lightsensor Shift calibration value */
	int g_als_maxlux_shift;					/* Lightsensor Shift for lux 20000 */
	int g_als_change_sensitivity;			/* Lightsensor Change sensitivity */
	int g_als_log_threshold;					/* Lightsensor Log Print Threshold */

	bool HAL_switch_on;						/* this var. means if HAL is turning on als or not */
	bool Device_switch_on;					/* this var. means if als hw is turn on or not */

	int int_counter;
	int event_counter;
};

/*=======================
 *|| I2c Stress Test Part ||
 *=======================
 */

#ifdef CONFIG_I2C_STRESS_TEST

#include <linux/i2c_testcase.h>

#define I2C_TEST_Lsensor_FAIL (-1)

static int IRsensor_I2C_stress_test(struct i2c_client *client)
{
	int lnResult = I2C_TEST_PASS;	
	int ret = 0;
	int r = 0;
	int g = 0;
	int b = 0;
	int adc = 0;
	int low_threshold = 0;
	int high_threshold = 0;

	i2c_log_in_test_case("TestIRSensorI2C ++\n");

	/* Check Hardware Support First */
	if(Lsensor_hw_client->mlsensor_hw->light_hw_turn_onoff== NULL) {
		err("light_hw_turn_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(Lsensor_hw_client->mlsensor_hw->light_hw_get_r == NULL) {
		err("light_hw_get_r NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(Lsensor_hw_client->mlsensor_hw->light_hw_get_g == NULL) {
		err("light_hw_get_g NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(Lsensor_hw_client->mlsensor_hw->light_hw_get_b == NULL) {
		err("light_hw_get_b NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(Lsensor_hw_client->mlsensor_hw->light_hw_get_ir == NULL) {
		err("light_hw_get_adc NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(Lsensor_hw_client->mlsensor_hw->light_hw_set_hi_threshold == NULL) {
		err("light_hw_set_hi_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(Lsensor_hw_client->mlsensor_hw->light_hw_set_lo_threshold == NULL) {
		err("light_hw_set_lo_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	
	/* Turn on Light Sensor */
	if(!g_als_data->Device_switch_on) {
		ret = Lsensor_hw_client->mlsensor_hw->light_hw_turn_onoff(true);
	}

	/* Light Sensor i2c read test */
	r = Lsensor_hw_client->mlsensor_hw->light_hw_get_r();
	if(r < 0){
		i2c_log_in_test_case("IRsensor Light Sensor Fail to get r\n");
		lnResult = I2C_TEST_Lsensor_FAIL;
		return lnResult;	
	}

	g = Lsensor_hw_client->mlsensor_hw->light_hw_get_g();
	if(g < 0){
		i2c_log_in_test_case("IRsensor Light Sensor Fail to get g\n");
		lnResult = I2C_TEST_Lsensor_FAIL;
		return lnResult;	
	}

	b = Lsensor_hw_client->mlsensor_hw->light_hw_get_b();
	if(b < 0){
		i2c_log_in_test_case("IRsensor Light Sensor Fail to get b\n");
		lnResult = I2C_TEST_Lsensor_FAIL;
		return lnResult;	
	}
	
	adc = Lsensor_hw_client->mlsensor_hw->light_hw_get_g();
	if(adc < 0){
		i2c_log_in_test_case("IRsensor Light Sensor Fail to get adc\n");
		lnResult = I2C_TEST_Lsensor_FAIL;
		return lnResult;	
	}
	/* Light Sensor Low Threshold */	
	low_threshold = adc * (100 - LIGHT_CHANGE_MID_SENSITIVITY) / 100;

	/* Light Sensor High Threshold */
	high_threshold = adc * (100 + LIGHT_CHANGE_MID_SENSITIVITY) / 100;	
	if (high_threshold > Lsensor_hw_client->mlsensor_hw->light_max_threshold)	
		high_threshold = Lsensor_hw_client->mlsensor_hw->light_max_threshold;	
	
	ret = Lsensor_hw_client->mlsensor_hw->light_hw_set_hi_threshold(high_threshold);
	if(ret < 0) {
		i2c_log_in_test_case("IRsensor Light Sensor Fail to set high threshold. \n");
		lnResult = I2C_TEST_Lsensor_FAIL;
		return lnResult;	
	}
	
	ret = Lsensor_hw_client->mlsensor_hw->light_hw_set_lo_threshold(low_threshold);
	if(ret < 0) {
		i2c_log_in_test_case("IRsensor Light Sensor Fail to set low threshold. \n");
		lnResult = I2C_TEST_Lsensor_FAIL;
		return lnResult;	
	}

	if(!g_als_data->HAL_switch_on) {
		ret = Lsensor_hw_client->mlsensor_hw->light_hw_turn_onoff(false);
	}
	
	i2c_log_in_test_case("TestLSensorI2C --\n");
	return lnResult;
}

static struct i2c_test_case_info IRSensorTestCaseInfo[] =	{
	__I2C_STRESS_TEST_CASE_ATTR(IRsensor_I2C_stress_test),
};
#endif

/*====================
 *|| Device Layer Part ||
 *====================
 */ 
static int light_turn_onoff(bool bOn)
{
	int ret=0;

	/* Check Hardware Support First */
	if(Lsensor_hw_client->mlsensor_hw->light_hw_turn_onoff == NULL) {
		err("light_hw_turn_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(Lsensor_hw_client->mlsensor_hw->light_hw_set_hi_threshold == NULL) {
		err("light_hw_set_hi_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(Lsensor_hw_client->mlsensor_hw->light_hw_set_lo_threshold == NULL) {
		err("light_hw_set_lo_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	
	if (bOn == 1)	{	/* power on */	
		if(1 == resume_flag){
			resume_flag=0;
		}else{
			light_get_shift();
		}
		log("[Cal] Light Sensor Set Shift calibration value : %d\n", g_als_data->g_als_calvalue_shift);

		if(g_als_data->Device_switch_on == false) {
			ret = Lsensor_hw_client->mlsensor_hw->light_hw_set_hi_threshold(0);
			if(ret < 0){
				err("light_hw_set_hi_threshold ERROR. \n");
				return -ENOENT;
			}
			ret = Lsensor_hw_client->mlsensor_hw->light_hw_set_lo_threshold(0);
			if(ret < 0){
				err("light_hw_set_lo_threshold ERROR. \n");
				return -ENOENT;
			}
			ret = Lsensor_hw_client->mlsensor_hw->light_hw_turn_onoff(true);
			if(ret < 0){
				err("light_hw_turn_onoff(true) ERROR. \n");
				return -ENOENT;
			}
		}
		/*enable IRQ only when proximity and light sensor is off*/
		if (g_als_data->Device_switch_on == false) {
			dbg("[IRQ] Enable irq !! \n");
			enable_irq(ASUS_IR_SENSOR_IRQ);
		}
		g_als_data->Device_switch_on = true;		
	} else	{	/* power off */	
		/*set turn off register*/
		if(g_als_data->Device_switch_on == true){
			/*disable IRQ before switch off*/		
			dbg("[IRQ] Disable irq !! \n");
			disable_irq_nosync(ASUS_IR_SENSOR_IRQ);
			
			ret = Lsensor_hw_client->mlsensor_hw->light_hw_turn_onoff(false);
			if(ret < 0){
				err("light_hw_turn_onoff(false) ERROR. \n");
			}else{
				/*change the Device Status*/
				g_als_data->Device_switch_on = false;	
			}
		}
	}
	
	return ret;
}

static int light_get_lux(int adc)
{
	int lux = 0;
#ifdef SW_OFFSET
	int slope = 0;
#endif

	if(adc < 0) {
		err("Light Sensor get Lux ERROR. (adc < 0)\n");
		return 0;
	}	

#ifdef SW_OFFSET
	slope = (800 * 10000 / (g_als_data->g_als_calvalue_1000lux - g_als_data->g_als_calvalue_200lux));
	if(slope < 3052 && adc > g_als_data->g_als_calvalue_1000lux) {
		log("SW work around for 20000 lux\n");
		lux = (adc * MAX_DIFF /(MAX_ADC - g_als_data->g_als_calvalue_1000lux)
				+ g_als_data->g_als_maxlux_shift);
	}else 
#endif
	if(adc < g_als_data->g_als_calvalue_200lux) {
		lux = (adc * 200/(g_als_data->g_als_calvalue_200lux));	
	} else {
		lux = (adc * 800/(g_als_data->g_als_calvalue_1000lux-g_als_data->g_als_calvalue_200lux)
				+ g_als_data->g_als_calvalue_shift);	
	}

	if(lux > LIGHT_MAX_LUX)
		lux = LIGHT_MAX_LUX;
	
	return lux;
}

static int 	light_get_shift(void)
{
	int ret = 0;
	
	/* Light Sensor Read Calibration*/
	ret = lsensor_sysfs_read_200lux();
	if(ret > 0 ) {
		g_als_data->g_als_calvalue_200lux = ret;
		log("Light Sensor read 200lux Calibration: Cal: %d\n", g_als_data->g_als_calvalue_200lux);
	}else{
		err("Light Sensor read DEFAULT 200lux Calibration: Cal: %d\n", g_als_data->g_als_calvalue_200lux);
	}
	
	ret = lsensor_sysfs_read_1000lux();
	if(ret > 0 ) {
		g_als_data->g_als_calvalue_1000lux = ret;
		log("Light Sensor read 1000lux Calibration: Cal: %d\n", g_als_data->g_als_calvalue_1000lux);
	}else{
		err("Light Sensor read DEFAULT 1000lux Calibration: Cal: %d\n", g_als_data->g_als_calvalue_1000lux);
	}

	g_als_data->g_als_maxlux_shift = (1000 - g_als_data->g_als_calvalue_1000lux*MAX_DIFF/
				(MAX_ADC - g_als_data->g_als_calvalue_1000lux));
	g_als_data->g_als_calvalue_shift = (1000 - g_als_data->g_als_calvalue_1000lux*800/
				(g_als_data->g_als_calvalue_1000lux-g_als_data->g_als_calvalue_200lux));

	return g_als_data->g_als_calvalue_shift;
}

static int 	light_get_accuracy_gain(void)
{
	int ret = 0;
	int gainvalue = 0;

	/* Light Sensor Read Calibration*/
	ret = lsensor_sysfs_read_200lux();
	if(ret > 0 )
		g_als_data->g_als_calvalue_200lux = ret;
	ret = lsensor_sysfs_read_1000lux();
	if(ret > 0 )
		g_als_data->g_als_calvalue_1000lux = ret;
		
	gainvalue = (800*LIGHT_GAIN_ACCURACY_CALVALUE)/
				(g_als_data->g_als_calvalue_1000lux-g_als_data->g_als_calvalue_200lux);
	
	return gainvalue;
}

static void light_polling_lux(struct work_struct *work)
{
	int adc = 0;
	int lux = 0;

	/* Check Hardware Support First */
	if(Lsensor_hw_client->mlsensor_hw->light_hw_get_g == NULL) {
		err("proximity_hw_get_adc NOT SUPPORT. \n");		
	}

mutex_lock(&g_ir_lock);
	if(g_als_data->HAL_switch_on == true) {
		/* Light Sensor Report the first real event*/			
		adc = Lsensor_hw_client->mlsensor_hw->light_hw_get_g();
		lux = light_get_lux(adc);
		log("[Polling] Light Sensor Report lux : %d (adc = %d)\n", lux, adc);
		lsensor_report_lux(lux);
	}
mutex_unlock(&g_ir_lock);
}
/**********************/
/*IR sensor Info Type*/
/*********************/

static IRsensor_info_type mLsensor_info_type = {{0}};
	
int mlight_show_calibration_200lux(void)
{
	int calvalue;
	calvalue = lsensor_sysfs_read_200lux();	
	dbg("Light Sensor show 200 lux Calibration: %d\n", calvalue);
	return calvalue;
}

int mlight_store_calibration_200lux(int calvalue)
{	
	if(calvalue <= 0) {
		err("Light Sensor store 200 lux Calibration with NON-POSITIVE value. (%d) \n", calvalue);
		return -EINVAL;
	}
	log("Light Sensor store 200 lux Calibration: %d\n", calvalue);
	lsensor_sysfs_write_200lux(calvalue);
	g_als_data->g_als_calvalue_200lux = calvalue;
	light_get_shift();

	return 0;
}

int mlight_show_calibration_1000lux(void)
{
	int calvalue;
	calvalue = lsensor_sysfs_read_1000lux();	
	dbg("Light Sensor show 1000 lux Calibration: %d\n", calvalue);
	return calvalue;
}	

int mlight_store_calibration_1000lux(int calvalue)
{
	if(calvalue <= 0) {
		err("Light Sensor store 1000 lux Calibration with NON-POSITIVE value. (%d) \n", calvalue);
		return -EINVAL;
	}
	log("Light Sensor store 1000 lux Calibration: %d\n", calvalue);
	lsensor_sysfs_write_1000lux(calvalue);
	g_als_data->g_als_calvalue_1000lux = calvalue;
	light_get_shift();
			
	return 0;
}

int mlight_show_shift(void)
{
	int shiftvalue;
	shiftvalue = light_get_shift();
	dbg("Light Sensor show Shift Calibration: %d\n", shiftvalue);
	return shiftvalue;
}

int mlight_show_gain(void)
{
	int gainvalue;
	gainvalue = light_get_accuracy_gain();
	dbg("Light Sensor show Gain Calibration: %d.%d\n",
		gainvalue/LIGHT_GAIN_ACCURACY_CALVALUE, gainvalue%LIGHT_GAIN_ACCURACY_CALVALUE);

	return gainvalue;
}

int mlight_show_r(void)
{
	int r = 0;
	if(Lsensor_hw_client->mlsensor_hw->light_hw_get_r == NULL) {
		err("light_hw_get_r NOT SUPPORT. \n");
		return -EINVAL;
	}

	mutex_lock(&g_ir_lock);

	if (!g_als_data->Device_switch_on) {
		light_turn_onoff(true);
	}
	
	msleep(LIGHT_TURNON_DELAY_TIME);
		
	r = Lsensor_hw_client->mlsensor_hw->light_hw_get_r();
	dbg("mlight_show_r : %d \n", r);

	if (!g_als_data->HAL_switch_on) {
		light_turn_onoff(false);
	}
	
	mutex_unlock(&g_ir_lock);
	return r;
}

int mlight_show_g(void)
{
	int g = 0;
	if(Lsensor_hw_client->mlsensor_hw->light_hw_get_g == NULL) {
		err("light_hw_get_g NOT SUPPORT. \n");
		return -EINVAL;
	}

	mutex_lock(&g_ir_lock);

	if (!g_als_data->Device_switch_on) {
		light_turn_onoff(true);
	}
	
	msleep(LIGHT_TURNON_DELAY_TIME);
		
	g = Lsensor_hw_client->mlsensor_hw->light_hw_get_g();
	dbg("mlight_show_g : %d \n", g);

	if (!g_als_data->HAL_switch_on) {
		light_turn_onoff(false);
	}
	
	mutex_unlock(&g_ir_lock);
	return g;
}

int mlight_show_b(void)
{
	int b = 0;
	if(Lsensor_hw_client->mlsensor_hw->light_hw_get_b == NULL) {
		err("light_hw_get_b NOT SUPPORT. \n");
		return -EINVAL;
	}

	mutex_lock(&g_ir_lock);

	if (!g_als_data->Device_switch_on) {
		light_turn_onoff(true);
	}
	
	msleep(LIGHT_TURNON_DELAY_TIME);
		
	b = Lsensor_hw_client->mlsensor_hw->light_hw_get_b();
	dbg("mlight_show_b : %d \n", b);

	if (!g_als_data->HAL_switch_on) {
		light_turn_onoff(false);
	}
	
	mutex_unlock(&g_ir_lock);
	return b;
}


int mlight_show_ir(void)
{
	int adc = 0;
	if(Lsensor_hw_client->mlsensor_hw->light_hw_get_ir == NULL) {
		err("light_hw_get_adc NOT SUPPORT. \n");
		return -EINVAL;
	}

	mutex_lock(&g_ir_lock);

	if (!g_als_data->Device_switch_on) {
		light_turn_onoff(true);
	}
	
	msleep(LIGHT_TURNON_DELAY_TIME);
		
	adc = Lsensor_hw_client->mlsensor_hw->light_hw_get_ir();
	dbg("mlight_show_ir : %d \n", adc);

	if (!g_als_data->HAL_switch_on) {
		light_turn_onoff(false);
	}
	
	mutex_unlock(&g_ir_lock);
	return adc;
}

static IRsensor_ATTR_Calibration mIRsensor_ATTR_Calibration = {
	.light_show_calibration_200lux = mlight_show_calibration_200lux,
	.light_store_calibration_200lux = mlight_store_calibration_200lux,
	.light_show_calibration_1000lux = mlight_show_calibration_1000lux,
	.light_store_calibration_1000lux = mlight_store_calibration_1000lux,
	.light_show_shift = mlight_show_shift,
	.light_show_gain = mlight_show_gain,
	.light_show_r = mlight_show_r,
	.light_show_g = mlight_show_g,
	.light_show_b = mlight_show_b,
	.light_show_ir = mlight_show_ir,
};

/******************/
/*BMMI Function*/
/****************/
bool mlight_show_atd_test(void)
{
	int ret=0;
	int round=0;

	ret = Lsensor_hw_client->IRsensor_hw_check_ID();
	if(ret < 0){
		err("Light Sensor ATD test check ID ERROR\n");
		goto light_atd_test_fail;
	}
	
	ret = Lsensor_hw_client->mlsensor_hw->light_hw_turn_onoff(true);
	if(ret < 0){
		err("Light Sensor ATD test turn on ERROR\n");
		goto light_atd_test_fail;
	}	
	
	for(; round<5; round++){
		ret = Lsensor_hw_client->mlsensor_hw->light_hw_get_g();
		if(ret < 0){
			err("Light Sensor ATD test get adc ERROR\n");
			goto light_atd_test_fail;
		}
		msleep(100);
	}	
	
	ret = Lsensor_hw_client->mlsensor_hw->light_hw_turn_onoff(false);
	if(ret < 0){
		err("Light Sensor ATD test turn off ERROR\n");
		goto light_atd_test_fail;
	}

	return true;
light_atd_test_fail:
	return false;

}

static IRsensor_ATTR_BMMI mLsensor_ATTR_BMMI = {
	.light_show_atd_test = mlight_show_atd_test,
};

/*********************/
/*Hardware Function*/
/********************/
int mLsensor_show_reg(uint8_t addr)
{
	int value;
	if(Lsensor_hw_client->IRsensor_hw_get_register == NULL) {
		err("IRsensor_hw_get_register NOT SUPPORT. \n");
		return -EINVAL;
	}

	value = Lsensor_hw_client->IRsensor_hw_get_register(addr);
	log("mLsensor_show_reg, addr=%02X, value=%02X.\n", addr, value);
	return value;
}

int mLsensor_store_reg(uint8_t addr, int value)
{	
	if(Lsensor_hw_client->IRsensor_hw_set_register == NULL) {
		err("IRsensor_hw_set_register NOT SUPPORT. \n");
		return -EINVAL;
	}

	Lsensor_hw_client->IRsensor_hw_set_register(addr, value);
	log("mLsensor_store_reg, addr=%02X, value=%02X.\n", addr, value);
	return 0;
}

static IRsensor_ATTR_Hardware mIRsensor_ATTR_Hardware = {
	.IRsensor_show_reg = mLsensor_show_reg,
	.IRsensor_store_reg = mLsensor_store_reg,
};

/****************/
/*HAL Function*/
/***************/
bool mlight_show_switch_onoff(void)
{
	return g_als_data->Device_switch_on;
}

int mlight_store_switch_onoff(bool bOn)
{
	mutex_lock(&g_ir_lock);
	dbg("Light Sensor switch = %d.\n", bOn);		
	if ((g_als_data->Device_switch_on != bOn)) {					
		if (bOn == true)	{
			/* Turn on Light Sensor */
			g_als_data->HAL_switch_on = true;
			light_turn_onoff(true);
			
			/*light sensor polling the first real event after delayed time. */
			queue_delayed_work(Lsensor_delay_workqueue, &light_polling_lux_work, msecs_to_jiffies(LIGHT_TURNON_DELAY_TIME));
		} else	{
			/* Turn off Light Sensor */
			g_als_data->HAL_switch_on = false;				
			light_turn_onoff(false);
			/* Report lux=-1 when turn off */
			lsensor_report_lux(-1);
		}			
	}else{
		log("Light Sensor is already %s", bOn?"ON":"OFF");
	}
	mutex_unlock(&g_ir_lock);

	return 0;
}

int mlight_show_lux(void)
{
	int adc = 0;
	int lux = 0;

	mutex_lock(&g_ir_lock);

	if (!g_als_data->Device_switch_on) {
		light_turn_onoff(true);
	}
	
	msleep(LIGHT_TURNON_DELAY_TIME);
	
	adc = Lsensor_hw_client->mlsensor_hw->light_hw_get_g();
	lux = light_get_lux(adc);
	dbg("mlight_show_lux : %d \n", lux);

	if (!g_als_data->HAL_switch_on) {
		light_turn_onoff(false);
	}
	
	mutex_unlock(&g_ir_lock);
	
	return lux;	
}

static IRsensor_ATTR_HAL mIRsensor_ATTR_HAL = {
	.light_show_switch_onoff = mlight_show_switch_onoff,
	.light_store_switch_onoff = mlight_store_switch_onoff,
	.light_show_lux = mlight_show_lux,		
};

/*********************/
/*Extension Function*/
/********************/
bool mLsensor_show_allreg(void)
{
	if(Lsensor_hw_client->IRsensor_hw_show_allreg == NULL) {
		err("IRsensor_hw_show_allreg NOT SUPPORT. \n");
		return false;
	}
	Lsensor_hw_client->IRsensor_hw_show_allreg();
	return true;
}

int mlight_show_sensitivity(void)
{
	return g_als_data->g_als_change_sensitivity;
}

int mlight_store_sensitivity(int sensitivity)
{
	g_als_data->g_als_change_sensitivity = sensitivity;
	log("Light Sensor store Sensitivity: %d\n", sensitivity);	
	
	return 0;
}

int mlight_show_log_threshold(void)
{
	return g_als_data->g_als_log_threshold;
}

int mlight_store_log_threshold(int log_threshold)
{
	g_als_data->g_als_log_threshold = log_threshold;
	log("Light Sensor store Log Threshold: %d\n", log_threshold);	
	
	return 0;
}

int mlight_show_int_count(void)
{
	return g_als_data->int_counter;
}

int mlight_show_event_count(void)
{
	return g_als_data->event_counter;
}

int mLsensor_show_error_mesg(char *error_mesg)
{
	memcpy(error_mesg, g_error_mesg, strlen(g_error_mesg)+1);
	return 0;
}

static IRsensor_ATTR_Extension mATTR_Extension = {
	.IRsensor_show_allreg = mLsensor_show_allreg,
	.light_show_sensitivity = mlight_show_sensitivity,
	.light_store_sensitivity = mlight_store_sensitivity,
	.light_show_log_threshold = mlight_show_log_threshold,
	.light_store_log_threshold = mlight_store_log_threshold,
	.light_show_int_count = mlight_show_int_count,
	.light_show_event_count = mlight_show_event_count,
	.IRsensor_show_error_mesg = mLsensor_show_error_mesg,
};

static IRsensor_ATTR mLsensor_ATTR = {
	.info_type = &mLsensor_info_type,
	.ATTR_Calibration = &mIRsensor_ATTR_Calibration,
	.ATTR_BMMI = &mLsensor_ATTR_BMMI,
	.ATTR_Hardware = &mIRsensor_ATTR_Hardware,
	.ATTR_HAL = &mIRsensor_ATTR_HAL,
	.ATTR_Extension = &mATTR_Extension,
};

/*================================
 *|| Interrupt Service Routine Part ||
 *================================
 */
static void light_work(void)
{	
	int low_threshold = 0;
	int high_threshold = 0;	
	int adc = 0;
	int lux = 0;
	int ret = 0;
	int light_change_sensitivity = 0;	
	int light_log_threshold = 0;

	/* Ignore the interrupt when Switch off */
	if(g_als_data->HAL_switch_on == true)
	{
		adc = Lsensor_hw_client->mlsensor_hw->light_hw_get_g();
		dbg("[ISR] Light Sensor Get adc : %d\n", adc);
		if(adc < 0){
			err("light_hw_get_adc ERROR\n");
			return;
		}

		/* Set the default sensitivity (3rd priority)*/
		if(adc >= g_als_data->g_als_calvalue_1000lux) {
			light_change_sensitivity = LIGHT_CHANGE_LOW_SENSITIVITY;
		} else if (adc <= g_als_data->g_als_calvalue_200lux) {
			light_change_sensitivity = LIGHT_CHANGE_HI_SENSITIVITY;
		} else {
			light_change_sensitivity = LIGHT_CHANGE_MID_SENSITIVITY;
		}

		/* Set the factory sensitivity (2nd priority) */
#ifdef ASUS_FACTORY_BUILD
		light_change_sensitivity = LIGHT_CHANGE_FACTORY_SENSITIVITY;
#endif

		/* Set the interface sensitivity (1st priority) */
		if(g_als_data->g_als_change_sensitivity >= 0)
			light_change_sensitivity = g_als_data->g_als_change_sensitivity;
		
		dbg("[ISR] Light Sensor Set Sensitivity. (light_change_sensitivity:%d)\n", light_change_sensitivity);	
		
		/* Light Sensor Low Threshold */	
		low_threshold = adc * (100 - light_change_sensitivity) / 100;

		/* Light Sensor High Threshold */
		high_threshold = (adc * (100 + light_change_sensitivity) / 100) + 1;	
		if (high_threshold > Lsensor_hw_client->mlsensor_hw->light_max_threshold)	
			high_threshold = Lsensor_hw_client->mlsensor_hw->light_max_threshold;	
		
		ret = Lsensor_hw_client->mlsensor_hw->light_hw_set_hi_threshold(high_threshold);
		if(ret < 0) {
			err("[ISR] Light Sensor Set High Threshold ERROR. (High:%d)\n", high_threshold);
		}
		dbg("[ISR] Light Sensor Set High Threshold. (High:%d)\n", high_threshold);	

		ret = Lsensor_hw_client->mlsensor_hw->light_hw_set_lo_threshold(low_threshold);
		if(ret < 0) {
			err("[ISR] Light Sensor Set Low Threshold ERROR. (Low:%d)\n", low_threshold);
		}
		dbg("[ISR] Light Sensor Set Low Threshold. (Low:%d)\n", low_threshold);	
		
		/* Light Sensor Report input event*/
		lux = light_get_lux(adc);

		light_log_threshold = LIGHT_LOG_THRESHOLD;
		
		/* Set the interface log threshold (1st priority) */
		if(g_als_data->g_als_log_threshold >= 0)
			light_log_threshold = g_als_data->g_als_log_threshold;
		
		if(abs(g_als_last_lux - lux) > light_log_threshold)
			log("[ISR] Light Sensor Report lux : %d (adc = %d)\n", lux, adc);
			
		lsensor_report_lux(lux);
		g_als_data->event_counter++;	/* --- For stress test debug --- */
		g_als_last_lux = lux;
	}
}

static void Lsensor_ist(struct work_struct *work)
{
	int irsensor_int_als;
	
mutex_lock(&g_ir_lock);
	if(g_als_data->HAL_switch_on == false) {
		log("ALSPS are disabled and ignore IST.\n");
		goto ist_err;
	}
	dbg("IRsensor ist +++ \n");
	if(Lsensor_hw_client == NULL)	{
		dbg("Lsensor_hw_client is NULL \n");
		goto ist_err;
	}

	/* Read INT_FLAG will clean the interrupt */
	ASUS_IR_SENSOR_INT = Lsensor_hw_client->IRsensor_hw_get_interrupt();
	if(ASUS_IR_SENSOR_INT <0){
		err("IRsensor_hw_get_interrupt ERROR\n");
		goto ist_err;
	}

	/* Check Light Sensor Interrupt */
	irsensor_int_als = ASUS_IR_SENSOR_INT&IRSENSOR_INT_ALS_MASK;
	if (irsensor_int_als == IRSENSOR_INT_ALS) {
		dbg("Light Sensor ist \n");
		if(g_als_data->HAL_switch_on == true)
			g_als_data->int_counter++;	/* --- For stress test debug --- */
		
		light_work();
	}
	dbg("IRsensor ist --- \n");
ist_err:	
	wake_unlock(&g_ir_wake_lock);
	dbg("[IRQ] Enable irq !! \n");
	enable_irq(ASUS_IR_SENSOR_IRQ);	
mutex_unlock(&g_ir_lock);

}

void Lsensor_irq_handler(void)
{
	dbg("[IRQ] Disable irq !! \n");
	disable_irq_nosync(ASUS_IR_SENSOR_IRQ);
	
	if(Lsensor_hw_client->IRsensor_hw_get_interrupt == NULL) {
		err("IRsensor_hw_get_interrupt NOT SUPPORT. \n");
		goto irq_err;
	}

	/*Queue work will enbale IRQ and unlock wake_lock*/
	queue_work(Lsensor_workqueue, &Lsensor_ist_work);
	wake_lock(&g_ir_wake_lock);
	return;
irq_err:
	dbg("[IRQ] Enable irq !! \n");
	enable_irq(ASUS_IR_SENSOR_IRQ);
}

static IRsensor_GPIO mIRsensor_GPIO = {
	.IRsensor_isr = Lsensor_irq_handler,
};

/*====================
 *|| Initialization Part ||
 *====================
 */
static int init_data(void)
{
	int ret = 0;
	/* Reset ASUS_light_sensor_data */
	g_als_data = kmalloc(sizeof(struct ASUS_light_sensor_data), GFP_KERNEL);
	if (!g_als_data)	{
		err("g_als_data kmalloc ERROR\n");
		ret = -ENOMEM;
		goto init_data_err;
	}
	memset(g_als_data, 0, sizeof(struct ASUS_light_sensor_data));
	g_als_data->Device_switch_on = false;
	g_als_data->HAL_switch_on = 	false;	
	
	g_als_data->g_als_calvalue_200lux = 	Lsensor_hw_client->mlsensor_hw->light_200lux_default;
	g_als_data->g_als_calvalue_1000lux = 	Lsensor_hw_client->mlsensor_hw->light_1000lux_default;	
	g_als_data->g_als_calvalue_shift = 		IRSENSOR_DEFAULT_VALUE;
	g_als_data->g_als_change_sensitivity = IRSENSOR_DEFAULT_VALUE;
	g_als_data->g_als_log_threshold = 		IRSENSOR_DEFAULT_VALUE;

	g_als_data->int_counter = 0;
	g_als_data->event_counter = 0;
	
	/*Record the error message*/
	g_error_mesg = kzalloc(sizeof(char [ERROR_MESG_SIZE]), GFP_KERNEL);
	
	return 0;
init_data_err:
	err("Init Data ERROR\n");
	return ret;
}
 
void mLsensor_algo_probe(struct i2c_client *client)
{	
	log("Driver PROBE +++\n");

	/*check i2c client*/
	if (client == NULL) {
		err("i2c Client is NUll\n");
		goto probe_err;
	}	

	/*link driver data to i2c client*/
	strlcpy(client->name, SENSOR_TYPE_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, g_als_data);

	/* GPIO */
	//	ASUS_IR_SENSOR_IRQ = IRsensor_gpio_register(client, &mIRsensor_GPIO);
	g_i2c_client = client;
	if (ASUS_IR_SENSOR_IRQ < 0)		
		goto probe_err;	

	/* I2c stress test */
#ifdef CONFIG_I2C_STRESS_TEST	
	i2c_add_test_case(client, "IRSensorTest", ARRAY_AND_SIZE(IRSensorTestCaseInfo));	
#endif

	log("Driver PROBE ---\n");
	return ;
probe_err:
	err("Driver PROBE ERROR ---\n");
	return;

}

void Lsensor_algo_remove(void)
{
	log("Driver REMOVE +++\n");

	Lsensor_gpio_unregister(ASUS_IR_SENSOR_IRQ);

	log("Driver REMOVE ---\n");
	
	return;
}

void mLsensor_algo_shutdown(void)
{
	log("Driver SHUTDOWN +++\n");

	/* Disable sensor */
	if (g_als_data->Device_switch_on)
		light_turn_onoff(false);
	
	log("Driver SHUTDOWN ---\n");
	
	return;
}

void mLsensor_algo_suspend(void)
{
	log("Driver SUSPEND +++\n");

	/* For make sure Light sensor mush be switch off when system suspend */
	if (g_als_data->Device_switch_on)				
		light_turn_onoff(false);
	
	log("Driver SUSPEND ---\n");
	
	return;
}

void mLsensor_algo_resume(void)
{
	log("Driver RESUME +++\n");

	if (g_als_data->Device_switch_on == 0 && g_als_data->HAL_switch_on == 1){
		resume_flag=1;
		light_turn_onoff(1);
	}
	
	log("Driver RESUME ---\n");
	
	return;
}

static IRsensor_I2C mLsensor_I2C = {
	.IRsensor_probe = mLsensor_algo_probe,
	.IRsensor_remove = Lsensor_algo_remove,
	.IRsensor_shutdown = mLsensor_algo_shutdown,
	.IRsensor_suspend = mLsensor_algo_suspend,
	.IRsensor_resume = mLsensor_algo_resume,
};

static int __init Lsensor_init(void)
{
	int ret = 0;
	log("Driver INIT +++\n");

	/*Record the error message*/
	g_error_mesg = kzalloc(sizeof(char [ERROR_MESG_SIZE]), GFP_KERNEL);
	
	/* Work Queue */
	Lsensor_workqueue = create_singlethread_workqueue(SENSOR_TYPE_NAME"_wq");	
	Lsensor_delay_workqueue = create_singlethread_workqueue(SENSOR_TYPE_NAME"_delay_wq");	

	/* Initialize the Mutex */
	mutex_init(&g_ir_lock);
	mutex_init(&g_i2c_lock);

	/* Initialize the wake lock */
	wake_lock_init(&g_ir_wake_lock, WAKE_LOCK_SUSPEND, "IRsensor_wake_lock");

	/* i2c Registration for probe/suspend/resume */				
	ret = IRsensor_i2c_register(&mLsensor_I2C);
	if (ret < 0)
		goto init_err;
	
	/* Hardware Register Initialization */
	Lsensor_hw_client = Lsensor_hw_getHardware();
	if(Lsensor_hw_client == NULL)
		goto init_err;

	/* driver data structure initialize */
	ret = init_data();
	if (ret < 0)
		goto init_err;

	/* string copy the character of vendor and module number */
	strcpy(mLsensor_ATTR.info_type->vendor, Lsensor_hw_client->vendor);
	strcpy(mLsensor_ATTR.info_type->module_number, Lsensor_hw_client->module_number);
	
	/* Attribute */
	Lsensor_ATTR_register(&mLsensor_ATTR);	
	if (ret < 0)
		goto init_err;
	
	/* Input Device */
	ret = IRsensor_report_register();
	if (ret < 0)
		goto init_err;	

	ASUS_IR_SENSOR_IRQ = Lsensor_gpio_register(g_i2c_client, &mIRsensor_GPIO);
	if (ASUS_IR_SENSOR_IRQ < 0)
		goto init_err;	
	log("Driver INIT ---\n");
	return 0;

init_err:
	err("Driver INIT ERROR ---\n");
	return ret;
}

static void __exit Lsensor_exit(void)
{
	log("Driver EXIT +++\n");

	/* i2c Unregistration */	
	IRsensor_i2c_unregister();

	IRsensor_report_unregister();
	Lsensor_ATTR_unregister();	
	
	wake_lock_destroy(&g_ir_wake_lock);
	mutex_destroy(&g_ir_lock);
	mutex_destroy(&g_i2c_lock);
	kfree(g_als_data);

	destroy_workqueue(Lsensor_workqueue);
	destroy_workqueue(Lsensor_delay_workqueue);
	
	log("Driver EXIT ---\n");
}

module_init(Lsensor_init);
module_exit(Lsensor_exit);

MODULE_AUTHOR("sr_Huang <sr_Huang@asus.com>");
MODULE_DESCRIPTION("Proximity and Ambient Light Sensor");
MODULE_LICENSE("GPL");

