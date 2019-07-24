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
#define SENSOR_TYPE_NAME		"Psensor"
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
static int ASUS_P_SENSOR_IRQ;
static int ASUS_P_SENSOR_INT;
static struct ASUS_proximity_sensor_data		*g_ps_data;
static struct IRsensor_hw					*Psensor_hw_client;
static struct workqueue_struct 				*Psensor_workqueue;
static struct workqueue_struct 				*Psensor_delay_workqueue;
static struct mutex 						g_p_lock;
static struct mutex 						g_i2c_lock;
static struct wake_lock 						g_ir_wake_lock;
static struct hrtimer 						g_ir_timer;
static struct i2c_client						*g_i2c_client;
static char *g_error_mesg;

/***********************/
/* IR Sensor Functions*/
/**********************/
/*Device Layer Part*/
static int 	proximity_turn_onoff(bool bOn);
static int 	proximity_set_threshold(void);
static void proximity_polling_adc(struct work_struct *work);

/*Interrupt Service Routine Part*/
static void IRsensor_ist(struct work_struct *work);
static void proximity_autok(struct work_struct *work);

/* Export Functions */
bool proximity_check_status(void);

/*Initialization Part*/
static int init_data(void);

/*Proximity auto calibration*/
static void proximity_autok(struct work_struct *work);

/*Work Queue*/
static 		DECLARE_WORK(IRsensor_ist_work, IRsensor_ist);
static 		DECLARE_WORK(proximity_autok_work, proximity_autok);
static 		DECLARE_DELAYED_WORK(proximity_polling_adc_work, proximity_polling_adc);

/*Disable touch for detecting near when phone call*/
//extern void ftxxxx_disable_touch(bool flag);
//extern int get_audiomode(void);

/*Proximity auto calibration*/
static int proximity_check_minCT(void);

/********************/
/* PS data structure */
/********************/

struct ASUS_proximity_sensor_data 
{	
	int g_ps_calvalue_lo;						/* Proximitysensor setting low calibration value(adc) */
	int g_ps_calvalue_hi;						/* Proximitysensor setting high calibration value(adc) */
	int g_ps_calvalue_inf;						/* Proximitysensor setting inf calibration value(adc) */

	int g_ps_autok_min;
	int g_ps_autok_max;
	
	bool HAL_switch_on;						/* this var. means if HAL is turning on ps or not */
	bool Device_switch_on;					/* this var. means is turning on ps or not */	
	bool polling_mode;							/* Polling for adc of proximity */
	bool autok;							/*auto calibration status*/

	int int_counter;
	int event_counter;
	int crosstalk_diff;
};

/*=======================
 *|| I2c Stress Test Part ||
 *=======================
 */

#ifdef CONFIG_I2C_STRESS_TEST

#include <linux/i2c_testcase.h>

#define I2C_TEST_Lsensor_FAIL (-1)
#define I2C_TEST_Psensor_FAIL (-1)

static int IRsensor_I2C_stress_test(struct i2c_client *client)
{
	int lnResult = I2C_TEST_PASS;	
	int ret = 0;

	i2c_log_in_test_case("TestIRSensorI2C ++\n");

	/* Check Hardware Support First */
	if(Psensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff== NULL) {
		err("proximity_hw_turn_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(Psensor_hw_client->mpsensor_hw->proximity_hw_get_adc == NULL) {
		err("proximity_hw_get_adc NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(Psensor_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold == NULL) {
		err("proximity_hw_set_hi_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(Psensor_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold == NULL) {
		err("proximity_hw_set_lo_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	
	/* Turn on Proximity Sensor */
	if(!g_ps_data->Device_switch_on) {
		ret = Psensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);
	}

	/* Proximity i2c read test */
	ret = Psensor_hw_client->mpsensor_hw->proximity_hw_get_adc();
	if(ret < 0){
		i2c_log_in_test_case("IRsensor Proximity Fail to get adc\n");
		lnResult = I2C_TEST_Psensor_FAIL;
		return lnResult;	
	}

	/* Proximity i2c write test */
	ret = Psensor_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold(g_ps_data->g_ps_calvalue_hi);
	if(ret < 0){
		i2c_log_in_test_case("IRsensor Proximity Fail to set high threshold.\n");
		lnResult = I2C_TEST_Psensor_FAIL;
		return lnResult;	
	}
	ret = Psensor_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold(g_ps_data->g_ps_calvalue_lo);
	if(ret < 0){
		i2c_log_in_test_case("IRsensor Proximity Fail to set low threshold. \n");
		lnResult = I2C_TEST_Psensor_FAIL;
		return lnResult;	
	}

	if(!g_ps_data->HAL_switch_on) {
		ret = Psensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);
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
static int proximity_turn_onoff(bool bOn)
{
	int ret = 0;	
	ktime_t autok_delay;

	/* Check Hardware Support First */
	if(Psensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff == NULL) {
		err("proximity_hw_turn_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	
	if (bOn == 1)	{	/* power on */
		/*Set Proximity Threshold*/
		ret = proximity_set_threshold();
		if (ret < 0) {	
			err("proximity_set_threshold ERROR\n");
			return ret;
		}

		/*check the min for auto calibration*/
		if(true == g_ps_data->autok){
			g_ps_data->crosstalk_diff = 0;
			/*Stage 1 : check first 6 adc which spend about 50ms~100ms*/
			ret = proximity_check_minCT();
			if (ret < 0) {	
				log("proximity_check_minCT ERROR\n");	
				g_ps_data->autok = false;
			}
		}
		
		/*set turn on register*/
		if(g_ps_data->Device_switch_on == false){
			ret = Psensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);	
			if(ret < 0){
				err("proximity_hw_turn_onoff(true) ERROR\n");
				return ret;
			}
				
		}
				
		/*enable IRQ only when proximity is off*/
		if (g_ps_data->Device_switch_on == false) {
			dbg("[IRQ] Enable irq !! \n");
			enable_irq(ASUS_P_SENSOR_IRQ);
		}
		/*change the Device Status*/
		g_ps_data->Device_switch_on = true;
		/*check the polling mode*/
		if(g_ps_data->polling_mode == true) {
			queue_delayed_work(Psensor_delay_workqueue, &proximity_polling_adc_work, msecs_to_jiffies(1000));
			log("[Polling] Proximity polling adc START. \n");
		}

		/*Stage 2 : start polling proximity adc(500ms) to check min value*/
		if(true == g_ps_data->autok && g_ps_data->crosstalk_diff != 0){
			autok_delay = ns_to_ktime( PROXIMITY_AUTOK_POLLING * NSEC_PER_MSEC);
			hrtimer_start(&g_ir_timer, autok_delay, HRTIMER_MODE_REL);
		}
		
	} else	{	/* power off */
		/*set turn off register*/
		if(g_ps_data->Device_switch_on == true){
			/*disable IRQ before switch off*/
			dbg("[IRQ] Disable irq !! \n");
			disable_irq_nosync(ASUS_P_SENSOR_IRQ);

			ret = Psensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);
			if(ret < 0){
				err("proximity_hw_turn_onoff(false) ERROR\n");
			}
			/*change the Device Status*/
			g_ps_data->Device_switch_on = false;			

			/*diable the timer*/
			if(g_ps_data->autok == true){
				hrtimer_cancel(&g_ir_timer);
			}
		}		
	}	
	
	return ret;
}

static int proximity_set_threshold(void)
{
	int ret = 0;	

	/* Check Hardware Support First */
	if(Psensor_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold == NULL) {
		err("proximity_hw_set_hi_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(Psensor_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold == NULL) {
		err("proximity_hw_set_lo_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	
	/*Set Proximity High Threshold*/
	ret = psensor_sysfs_read_high();
	if(ret > 0) {
	    	g_ps_data->g_ps_calvalue_hi = ret;
		log("Proximity read High Calibration : %d\n", g_ps_data->g_ps_calvalue_hi);
	}else{
		err("Proximity read DEFAULT High Calibration : %d\n", g_ps_data->g_ps_calvalue_hi);
	}
	ret = Psensor_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold(g_ps_data->g_ps_calvalue_hi);
	if(ret < 0){
		err("proximity_hw_set_hi_threshold ERROR. \n");
		return -ENOENT;
	}
	
	/*Set Proximity Low Threshold*/
	ret = psensor_sysfs_read_low();	
	if(ret > 0) {
	    	g_ps_data->g_ps_calvalue_lo = ret;
		log("Proximity read Low Calibration : %d\n", g_ps_data->g_ps_calvalue_lo);
	}else{
		err("Proximity read DEFAULT Low Calibration : %d\n", g_ps_data->g_ps_calvalue_lo);
	}
	ret = Psensor_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold(g_ps_data->g_ps_calvalue_lo);
	if(ret < 0){
		err("proximity_hw_set_hi_threshold ERROR. \n");
		return -ENOENT;
	}
	
	return 0;
}

static void proximity_polling_adc(struct work_struct *work)
{
	int adc = 0;

	/* Check Hardware Support First */
	if(Psensor_hw_client->mpsensor_hw->proximity_hw_get_adc == NULL) {
		err("proximity_hw_get_adc NOT SUPPORT. \n");		
	}
	
	adc= Psensor_hw_client->mpsensor_hw->proximity_hw_get_adc();
	log("[Polling] Proximity get adc = %d\n", adc);
	
	if(g_ps_data->Device_switch_on == true)
		queue_delayed_work(Psensor_delay_workqueue, &proximity_polling_adc_work, msecs_to_jiffies(1000));
	else
		log("[Polling] Proximity polling adc STOP. \n");
}

/**********************/
/*IR sensor Info Type*/
/*********************/

static IRsensor_info_type mPsensor_info_type = {{0}};
	
/**********************/
/*Calibration Function*/
/*********************/
int mproximity_show_calibration_hi(void)
{
	int calvalue;
	calvalue = psensor_sysfs_read_high();	
	dbg("Proximity show High Calibration: %d\n", calvalue);
	return calvalue;
}

int mproximity_store_calibration_hi(int calvalue)
{
	if(calvalue <= 0) {
		err("Proximity store High Calibration with NON-POSITIVE value. (%d) \n", calvalue);
		return -EINVAL;	
	}
	log("Proximity store High Calibration: %d\n", calvalue);
	psensor_sysfs_write_high(calvalue);
	proximity_set_threshold();
	
	return 0;
}

int mproximity_show_calibration_lo(void)
{
	int calvalue;
	calvalue = psensor_sysfs_read_low();
	dbg("Proximity show Low Calibration: %d\n", calvalue);
	return calvalue;
}

int mproximity_store_calibration_lo(int calvalue)
{
	if(calvalue <= 0) {
		err("Proximity store Low Calibration with NON-POSITIVE value. (%d) \n", calvalue);
		return -EINVAL;
	}
	log("Proximity store Low Calibration: %d\n", calvalue);
	psensor_sysfs_write_low(calvalue);
	proximity_set_threshold();	

	return 0;
}

int mproximity_show_calibration_inf(void)
{
	int calvalue;
	calvalue = psensor_sysfs_read_inf();
	dbg("Proximity show Inf Calibration: %d\n", calvalue);
	return calvalue;
}

int mproximity_store_calibration_inf(int calvalue)
{
	if(calvalue <= 0) {
		err("Proximity store Inf Calibration with NON-POSITIVE value. (%d) \n", calvalue);
		return -EINVAL;
	}
	log("Proximity store Inf Calibration: %d\n", calvalue);
	psensor_sysfs_write_inf(calvalue);
	
	return 0;
}

int mproximity_show_adc(void)
{
	int adc = 0;
	int ret;
	
	if(Psensor_hw_client->mpsensor_hw->proximity_hw_get_adc == NULL) {
		err("proximity_hw_get_adc NOT SUPPORT. \n");
		return -EINVAL;
	}

	mutex_lock(&g_p_lock);
	
	if(g_ps_data->Device_switch_on == false){
		ret = Psensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);	
		if(ret < 0){
			err("proximity_hw_turn_onoff(true) ERROR\n");
			return ret;
		}			
	}

	msleep(PROXIMITY_TURNON_DELAY_TIME);
	
	adc = Psensor_hw_client->mpsensor_hw->proximity_hw_get_adc();
	dbg("mproximity_show_adc : %d \n", adc);
	
	if(g_ps_data->HAL_switch_on == false){
		ret = Psensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);	
		if(ret < 0){
			err("proximity_hw_turn_onoff(false) ERROR\n");
			return ret;
		}			
	}
	
	mutex_unlock(&g_p_lock);
	
	return adc;
}

static IRsensor_ATTR_Calibration mIRsensor_ATTR_Calibration = {
	.proximity_show_calibration_hi = mproximity_show_calibration_hi,
	.proximity_store_calibration_hi = mproximity_store_calibration_hi,
	.proximity_show_calibration_lo = mproximity_show_calibration_lo,
	.proximity_store_calibration_lo = mproximity_store_calibration_lo,
	.proximity_show_calibration_inf = mproximity_show_calibration_inf ,
	.proximity_store_calibration_inf  = mproximity_store_calibration_inf ,
	.proximity_show_adc = mproximity_show_adc,
};

/******************/
/*BMMI Function*/
/****************/
bool mproximity_show_atd_test(void)
{
	int ret=0;
	int round=0;

	ret = Psensor_hw_client->IRsensor_hw_check_ID();
	if(ret < 0){
		err("Proximity ATD test check ID ERROR\n");
		goto proximity_atd_test_fail;
	}
	
	ret = Psensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);		
	if(ret < 0){
		err("Proximity ATD test turn on ERROR\n");
		goto proximity_atd_test_fail;
	}	
	
	for(;round<5; round++){
		ret = Psensor_hw_client->mpsensor_hw->proximity_hw_get_adc();
		if(ret < 0){
			err("Proximity ATD test get adc ERROR\n");
			goto proximity_atd_test_fail;
		}
		msleep(100);
	}	

	ret = Psensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);	
	if(ret < 0){
		err("Proximity ATD test turn off ERROR\n");
		goto proximity_atd_test_fail;
	}

	return true;
proximity_atd_test_fail:
	return false;
}

static IRsensor_ATTR_BMMI mPsensor_ATTR_BMMI = {
	.proximity_show_atd_test = mproximity_show_atd_test,
};

/*********************/
/*Hardware Function*/
/********************/
int mPsensor_show_reg(uint8_t addr)
{
	int value;
	if(Psensor_hw_client->IRsensor_hw_get_register == NULL) {
		err("IRsensor_hw_get_register NOT SUPPORT. \n");
		return -EINVAL;
	}

	value = Psensor_hw_client->IRsensor_hw_get_register(addr);
	log("mPsensor_show_reg, addr=%02X, value=%02X.\n", addr, value);
	return value;
}

int mPsensor_store_reg(uint8_t addr, int value)
{	
	if(Psensor_hw_client->IRsensor_hw_set_register == NULL) {
		err("IRsensor_hw_set_register NOT SUPPORT. \n");
		return -EINVAL;
	}

	Psensor_hw_client->IRsensor_hw_set_register(addr, value);
	log("mPsensor_store_reg, addr=%02X, value=%02X.\n", addr, value);
	return 0;
}

static IRsensor_ATTR_Hardware mIRsensor_ATTR_Hardware = {
	.IRsensor_show_reg = mPsensor_show_reg,
	.IRsensor_store_reg = mPsensor_store_reg,
};

/****************/
/*HAL Function*/
/***************/
bool mproximity_show_switch_onoff(void)
{	
	return g_ps_data->Device_switch_on;
}

int mproximity_store_switch_onoff(bool bOn)
{
	mutex_lock(&g_p_lock);
	dbg("Proximity switch = %d.\n", bOn);		
	if ((g_ps_data->Device_switch_on != bOn))	{						
		if (bOn == true)	{
			/* Turn on Proxomity */
			g_ps_data->HAL_switch_on = true;
			proximity_turn_onoff(true);
			/* send the init value */
			psensor_report_abs(IRSENSOR_REPORT_PS_AWAY);
			log("Proximity Report First Away abs.\n");
		} else	{
			/* Turn off Proxomity */
			g_ps_data->HAL_switch_on = false;				
			proximity_turn_onoff(false);
//			ftxxxx_disable_touch(false);
		}			
	}else{
		log("Proximity is already %s", bOn?"ON":"OFF");
	}
	mutex_unlock(&g_p_lock);
	
	return 0;
}

static IRsensor_ATTR_HAL mIRsensor_ATTR_HAL = {
	.proximity_show_switch_onoff = mproximity_show_switch_onoff,
	.proximity_store_switch_onoff = mproximity_store_switch_onoff,
	.proximity_show_status = proximity_check_status,
};

/*********************/
/*Extension Function*/
/********************/
bool mPsensor_show_allreg(void)
{
	if(Psensor_hw_client->IRsensor_hw_show_allreg == NULL) {
		err("IRsensor_hw_show_allreg NOT SUPPORT. \n");
		return false;
	}
	Psensor_hw_client->IRsensor_hw_show_allreg();
	return true;
}

bool mproximity_show_polling_mode(void)
{
	return g_ps_data->polling_mode;
}

int mproximity_store_polling_mode(bool bOn)
{
	g_ps_data->polling_mode = bOn;	
	return 0;
}

bool mproximity_show_autok(void)
{
	return g_ps_data->autok;
}

int mproximity_store_autok(bool bOn)
{
	g_ps_data->autok = bOn;	
	return 0;
}

int mproximity_show_int_count(void)
{
	return g_ps_data->int_counter;
}

int mproximity_show_event_count(void)
{
	return g_ps_data->event_counter;
}

int mproximity_show_autokmin(void)
{
	return g_ps_data->g_ps_autok_min;
}

int mproximity_store_autokmin(int autokmin)
{
	g_ps_data->g_ps_autok_min = autokmin;
	log("Proximity store autokmin: %d\n", autokmin);	
	
	return 0;
}

int mproximity_show_autokmax(void)
{
	return g_ps_data->g_ps_autok_max;
}

int mproximity_store_autokmax(int autokmax)
{
	g_ps_data->g_ps_autok_max = autokmax;
	log("Proximity store autokmax: %d\n", autokmax);	
	
	return 0;
}

int mPsensor_show_error_mesg(char *error_mesg)
{
	memcpy(error_mesg, g_error_mesg, strlen(g_error_mesg)+1);
	return 0;
}

static IRsensor_ATTR_Extension mATTR_Extension = {
	.IRsensor_show_allreg = mPsensor_show_allreg,
	.proximity_show_polling_mode = mproximity_show_polling_mode,
	.proximity_store_polling_mode = mproximity_store_polling_mode,
	.proximity_show_autok = mproximity_show_autok,
	.proximity_store_autok = mproximity_store_autok,
	.proximity_show_int_count = mproximity_show_int_count,
	.proximity_show_event_count = mproximity_show_event_count,
	.proximity_show_autokmin = mproximity_show_autokmin,
	.proximity_store_autokmin = mproximity_store_autokmin,
	.proximity_show_autokmax = mproximity_show_autokmax,
	.proximity_store_autokmax = mproximity_store_autokmax,
	.IRsensor_show_error_mesg = mPsensor_show_error_mesg,
};

static IRsensor_ATTR mPsensor_ATTR = {
	.info_type = &mPsensor_info_type,
	.ATTR_Calibration = &mIRsensor_ATTR_Calibration,
	.ATTR_BMMI = &mPsensor_ATTR_BMMI,
	.ATTR_Hardware = &mIRsensor_ATTR_Hardware,
	.ATTR_HAL = &mIRsensor_ATTR_HAL,
	.ATTR_Extension = &mATTR_Extension,
};

/*================================
 *|| Interrupt Service Routine Part ||
 *================================
 */
static void proximity_work(int state)
{
	int adc = 0;

	/* Get Proximity adc value */
	adc= Psensor_hw_client->mpsensor_hw->proximity_hw_get_adc();
	if(adc < 0){
		err("[ISR] Proximity get adc ERROR\n");	
		return;
	}

	/* Ignore the interrupt when Switch off */
	if(g_ps_data->HAL_switch_on == true)
	{
		/* Check proximity close or away. */
		if(IRSENSOR_INT_PS_AWAY == state) {
			log("[ISR] Proximity Detect Object Away. (adc = %d)\n", adc);
			psensor_report_abs(IRSENSOR_REPORT_PS_AWAY);
			g_ps_data->event_counter++;	/* --- For stress test debug --- */
//			if (2 == get_audiomode()) {
//				ftxxxx_disable_touch(false);
//			}
		} else if (IRSENSOR_INT_PS_CLOSE == state) {
			log("[ISR] Proximity Detect Object Close. (adc = %d)\n", adc);		
			psensor_report_abs(IRSENSOR_REPORT_PS_CLOSE);
			g_ps_data->event_counter++;	/* --- For stress test debug --- */
//			if (2 == get_audiomode()) {
//				ftxxxx_disable_touch(true);
//			}
		} else {
			err("[ISR] Proximity Detect Object ERROR. (adc = %d)\n", adc);
		}
	}
	
}

static void IRsensor_ist(struct work_struct *work)
{
	int irsensor_int_ps;
	
mutex_lock(&g_p_lock);
	if(g_ps_data->HAL_switch_on == false) {
		log("PS are disabled and ignore IST.\n");
		goto ist_err;
	}
	dbg("IRsensor ist +++ \n");
	if(Psensor_hw_client == NULL)	{
		dbg("Psensor_hw_client is NULL \n");
		goto ist_err;
	}

	/* Read INT_FLAG will clean the interrupt */
	ASUS_P_SENSOR_INT = Psensor_hw_client->IRsensor_hw_get_interrupt();
	if(ASUS_P_SENSOR_INT <0){
		err("IRsensor_hw_get_interrupt ERROR\n");
		goto ist_err;
	}

	/* Check Proximity Interrupt */
	irsensor_int_ps = ASUS_P_SENSOR_INT&IRSENSOR_INT_PS_MASK;
	if(irsensor_int_ps == IRSENSOR_INT_PS_CLOSE || irsensor_int_ps == IRSENSOR_INT_PS_AWAY) 
	{
		dbg("Proximity ist \n");
		if(g_ps_data->HAL_switch_on == true)
			g_ps_data->int_counter++;	/* --- For stress test debug --- */
		
		if (irsensor_int_ps == IRSENSOR_INT_PS_AWAY) {
			proximity_work(IRSENSOR_INT_PS_AWAY);
		}
		if (irsensor_int_ps == IRSENSOR_INT_PS_CLOSE) {
			proximity_work(IRSENSOR_INT_PS_CLOSE);
		}
	}

	dbg("IRsensor ist --- \n");
ist_err:	
	wake_unlock(&g_ir_wake_lock);
	dbg("[IRQ] Enable irq !! \n");
	enable_irq(ASUS_P_SENSOR_IRQ);	
mutex_unlock(&g_p_lock);

}

void Psensor_irq_handler(void)
{
	dbg("[IRQ] Disable irq !! \n");
	disable_irq_nosync(ASUS_P_SENSOR_IRQ);
	
	if(Psensor_hw_client->IRsensor_hw_get_interrupt == NULL) {
		err("IRsensor_hw_get_interrupt NOT SUPPORT. \n");
		goto irq_err;
	}

	/*Queue work will enbale IRQ and unlock wake_lock*/
	queue_work(Psensor_workqueue, &IRsensor_ist_work);
	wake_lock(&g_ir_wake_lock);
	return;
irq_err:
	dbg("[IRQ] Enable irq !! \n");
	enable_irq(ASUS_P_SENSOR_IRQ);
}

static IRsensor_GPIO mIRsensor_GPIO = {
	.IRsensor_isr = Psensor_irq_handler,
};


/*============================
 *|| For Proximity check status ||
 *============================
 */
bool proximity_check_status(void)
{	
	int adc_value = 0;
	bool status = false;
	int ret=0;
	int threshold_high = 0;
	
	/* check probe status */
	if(Psensor_hw_client == NULL)
		return status;

	mutex_lock(&g_p_lock);

	/*Set Proximity Threshold(reset to factory)*/
	if(g_ps_data->Device_switch_on == false){
		ret = proximity_set_threshold();
		if (ret < 0) {	
			err("proximity_set_threshold ERROR\n");
			return status;
		}

		ret = Psensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);	
		if(ret < 0){
			err("proximity_hw_turn_onoff(true) ERROR\n");
			return status;
		}			
	}
	
	msleep(PROXIMITY_TURNON_DELAY_TIME);

	adc_value = Psensor_hw_client->mpsensor_hw->proximity_hw_get_adc();
	threshold_high =  (g_ps_data->g_ps_calvalue_hi + g_ps_data->g_ps_autok_max);

	if (adc_value >= threshold_high) {
		status = true;
	}else{ 
		status = false;
	}
	log("proximity_check_status : %s , (adc, hi_cal + AutoK)MAX)=(%d, %d)\n", 
		status?"Close":"Away", adc_value, threshold_high);
	
	if(g_ps_data->Device_switch_on == false){
		ret = Psensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);	
		if(ret < 0){
			err("proximity_hw_turn_onoff(false) ERROR\n");
			return status;
		}			
	}
	
	mutex_unlock(&g_p_lock);

	return status;
}

EXPORT_SYMBOL(proximity_check_status);

/*===========================
 *|| Proximity Auto Calibration Part ||
 *============================
 */
 static int proximity_set_nowork_threshold(void)
{
	int ret = 0;	

	/* Check Hardware Support First */
	if(Psensor_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold == NULL) {
		err("proximity_hw_set_hi_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(Psensor_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold == NULL) {
		err("proximity_hw_set_lo_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	
	/*Set Proximity High Threshold*/
	ret = Psensor_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold(9999);
	if(ret < 0){
		err("proximity_hw_set_hi_threshold ERROR. \n");
		return -ENOENT;
	}
	
	/*Set Proximity Low Threshold*/
	ret = Psensor_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold(0);
	if(ret < 0){
		err("proximity_hw_set_hi_threshold ERROR. \n");
		return -ENOENT;
	}
	
	return 0;
}

static int proximity_check_minCT(void)
{
	int adc_value = 0;
	int crosstalk_diff;
	int crosstalk_min = 9999;
	int ret;
	int round;
	
	/*check the crosstalk calibration value*/	
	ret = psensor_sysfs_read_inf();	
	if(ret > 0) {
	    	g_ps_data->g_ps_calvalue_inf= ret;
		log("Proximity read INF Calibration : %d\n", g_ps_data->g_ps_calvalue_inf);
	}else{
		err("Proximity read DEFAULT INF Calibration : %d\n", g_ps_data->g_ps_calvalue_inf);
	}	

	/*make sure de-asserted INT when cat adc*/
	ret = proximity_set_nowork_threshold();
	if (ret < 0) {	
		err("proximity_set_nowork_threshold ERROR\n");
		return ret;
	}

	ret = Psensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);	
	if(ret < 0){
		err("proximity_hw_turn_onoff(true) ERROR\n");
		return ret;
	}
	/*update the min crosstalk value*/
	for(round=0; round<PROXIMITY_AUTOK_COUNT; round++){	
		mdelay(PROXIMITY_AUTOK_DELAY);
		adc_value = Psensor_hw_client->mpsensor_hw->proximity_hw_get_adc();
		log("proximity auto calibration adc : %d\n", adc_value);
		if(adc_value < crosstalk_min ){
			crosstalk_min = adc_value;
			log("Update the min for crosstalk : %d\n", crosstalk_min);
		}
	}
	ret = Psensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);	
	if(ret < 0){
		err("proximity_hw_turn_onoff(true) ERROR\n");
		return ret;
	}

	/*Set Proximity Threshold*/
	ret = proximity_set_threshold();
	if (ret < 0) {	
		err("proximity_set_threshold ERROR\n");
		return ret;
	}

	/*update the diff crosstalk value*/
	crosstalk_diff = crosstalk_min -g_ps_data->g_ps_calvalue_inf;
	if(crosstalk_diff>g_ps_data->g_ps_autok_min && crosstalk_diff<g_ps_data->g_ps_autok_max){
		log("Update the diff for crosstalk : %d\n", crosstalk_diff);
		g_ps_data->crosstalk_diff = crosstalk_diff;

		if(Psensor_hw_client->mpsensor_hw->proximity_hw_set_autoK == NULL) {
			err("proximity_hw_set_autoK NOT SUPPORT. \n");
			return -1;
		}
		Psensor_hw_client->mpsensor_hw->proximity_hw_set_autoK(crosstalk_diff);
		g_ps_data->g_ps_calvalue_hi += crosstalk_diff;
		g_ps_data->g_ps_calvalue_lo += crosstalk_diff;
	}else if(crosstalk_diff>=g_ps_data->g_ps_autok_max){
		log("crosstalk diff(%d) >= proximity autok max(%d)\n", crosstalk_diff, g_ps_data->g_ps_autok_max);
		g_ps_data->crosstalk_diff = crosstalk_diff;
	}else{
		log("crosstalk diff(%d) <= proximity autok min(%d)\n", crosstalk_diff, g_ps_data->g_ps_autok_min);
		g_ps_data->crosstalk_diff = 0;
	}
	
	return 0;
}

static void proximity_autok(struct work_struct *work)
{
	int adc_value;
	int crosstalk_diff;

	if(Psensor_hw_client->mpsensor_hw->proximity_hw_set_autoK == NULL) {
		err("proximity_hw_set_autoK NOT SUPPORT. \n");
		return;
	}
	
	adc_value = Psensor_hw_client->mpsensor_hw->proximity_hw_get_adc();
	dbg("auto calibration polling : %d\n", adc_value);

	crosstalk_diff = adc_value -g_ps_data->g_ps_calvalue_inf;

	if((crosstalk_diff<g_ps_data->crosstalk_diff) &&( g_ps_data->crosstalk_diff!=0)){
		/*last diff of crosstalk does not set to HW, should reset the value to 0.*/
		if(g_ps_data->crosstalk_diff >= g_ps_data->g_ps_autok_max ){
			g_ps_data->crosstalk_diff=0;
		}
		
		if(crosstalk_diff<=g_ps_data->g_ps_autok_min ){			
			Psensor_hw_client->mpsensor_hw->proximity_hw_set_autoK(0-g_ps_data->crosstalk_diff);
			g_ps_data->crosstalk_diff = 0;
			log("Update the diff for crosstalk : %d\n", g_ps_data->crosstalk_diff);
		}else if((crosstalk_diff>g_ps_data->g_ps_autok_min) && (crosstalk_diff<g_ps_data->g_ps_autok_max) ){			
			Psensor_hw_client->mpsensor_hw->proximity_hw_set_autoK(crosstalk_diff-g_ps_data->crosstalk_diff);
			g_ps_data->crosstalk_diff = crosstalk_diff;
			log("Update the diff for crosstalk : %d\n", crosstalk_diff);
		}else{
			log("over the autok_max : (adc, inf) = %d(%d, %d) > %d\n", 
				crosstalk_diff, adc_value, g_ps_data->g_ps_calvalue_inf, g_ps_data->g_ps_autok_max);
			g_ps_data->crosstalk_diff = crosstalk_diff;
		}
	}	
}

static enum hrtimer_restart proximity_timer_function(struct hrtimer *timer)
{
	ktime_t autok_delay;
	
	dbg("proximity_timer_function\n");
	queue_work(Psensor_workqueue, &proximity_autok_work);	

	if(0 == g_ps_data->crosstalk_diff){
		return HRTIMER_NORESTART;
	}else{
		/*needs to be reset in the callback function*/
		autok_delay = ns_to_ktime( PROXIMITY_AUTOK_POLLING * NSEC_PER_MSEC);
		hrtimer_forward_now(&g_ir_timer, autok_delay);
	}
	return HRTIMER_RESTART;
}

/*====================
 *|| I2C mutex lock ||
 *====================
 */
void lock_i2c_bus6(void) {
	mutex_lock(&g_i2c_lock);
}
EXPORT_SYMBOL(lock_i2c_bus6);

void unlock_i2c_bus6(void) {
	mutex_unlock(&g_i2c_lock);
}
EXPORT_SYMBOL(unlock_i2c_bus6);

/*====================
 *|| Initialization Part ||
 *====================
 */
static int init_data(void)
{
	int ret = 0;
	/* Reset ASUS_proximity_sensor_data */
	g_ps_data = kmalloc(sizeof(struct ASUS_proximity_sensor_data), GFP_KERNEL);
	if (!g_ps_data) {
		err("g_ps_data kmalloc ERROR\n");
		ret = -ENOMEM;
		goto init_data_err;
	}
	memset(g_ps_data, 0, sizeof(struct ASUS_proximity_sensor_data));
	g_ps_data->Device_switch_on = 	false;
	g_ps_data->HAL_switch_on = 	false;	
	g_ps_data->polling_mode = 		false;
	g_ps_data->autok = 			true;
	
	g_ps_data->g_ps_calvalue_hi = Psensor_hw_client->mpsensor_hw->proximity_hi_threshold_default;
	g_ps_data->g_ps_calvalue_lo = Psensor_hw_client->mpsensor_hw->proximity_low_threshold_default;	
	g_ps_data->g_ps_calvalue_inf = Psensor_hw_client->mpsensor_hw->proximity_crosstalk_default;	
	g_ps_data->g_ps_autok_min= Psensor_hw_client->mpsensor_hw->proximity_autok_min;	
	g_ps_data->g_ps_autok_max = Psensor_hw_client->mpsensor_hw->proximity_autok_max;	

	g_ps_data->int_counter = 0;
	g_ps_data->event_counter = 0;
	g_ps_data->crosstalk_diff = 0;

	/*Record the error message*/
	g_error_mesg = kzalloc(sizeof(char [ERROR_MESG_SIZE]), GFP_KERNEL);
	
	return 0;
init_data_err:
	err("Init Data ERROR\n");
	return ret;
}
 
void mPsensor_algo_probe(struct i2c_client *client)
{	
	log("Driver PROBE +++\n");

	/*check i2c client*/
	if (client == NULL) {
		err("i2c Client is NUll\n");
		goto probe_err;
	}	

	/*link driver data to i2c client*/
	strlcpy(client->name, SENSOR_TYPE_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, g_ps_data);	

	/* GPIO */
	//	ASUS_IR_SENSOR_IRQ = IRsensor_gpio_register(client, &mIRsensor_GPIO);
	g_i2c_client = client;
	if (ASUS_P_SENSOR_IRQ < 0)		
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

void Psensor_algo_remove(void)
{
	log("Driver REMOVE +++\n");

	Psensor_gpio_unregister(ASUS_P_SENSOR_IRQ);

	log("Driver REMOVE ---\n");
	
	return;
}

void mPsensor_algo_shutdown(void)
{
	log("Driver SHUTDOWN +++\n");

	/* Disable sensor */
	if (g_ps_data->Device_switch_on)
		proximity_turn_onoff(false);	
	
	log("Driver SHUTDOWN ---\n");
	
	return;
}

void mPsensor_algo_suspend(void)
{
	log("Driver SUSPEND +++\n");

	/* For keep Proximity can wake_up system */
	if (g_ps_data->Device_switch_on)
		enable_irq_wake(ASUS_P_SENSOR_IRQ);

	log("Driver SUSPEND ---\n");
	
	return;
}

void mPsensor_algo_resume(void)
{
	log("Driver RESUME +++\n");
	log("Driver RESUME ---\n");
	
	return;
}

static IRsensor_I2C mPsensor_I2C = {
	.IRsensor_probe = mPsensor_algo_probe,
	.IRsensor_remove = Psensor_algo_remove,
	.IRsensor_shutdown = mPsensor_algo_shutdown,
	.IRsensor_suspend = mPsensor_algo_suspend,
	.IRsensor_resume = mPsensor_algo_resume,
};

static int __init IRsensor_init(void)
{
	int ret = 0;
	log("Driver INIT +++\n");

	/*Record the error message*/
	g_error_mesg = kzalloc(sizeof(char [ERROR_MESG_SIZE]), GFP_KERNEL);
	
	/* Work Queue */
	Psensor_workqueue = create_singlethread_workqueue(SENSOR_TYPE_NAME"_wq");	
	Psensor_delay_workqueue = create_singlethread_workqueue(SENSOR_TYPE_NAME"_delay_wq");	

	/* Initialize the Mutex */
	mutex_init(&g_p_lock);
	mutex_init(&g_i2c_lock);

	/* Initialize the wake lock */
	wake_lock_init(&g_ir_wake_lock, WAKE_LOCK_SUSPEND, "IRsensor_wake_lock");

	/*Initialize high resolution timer*/
	hrtimer_init(&g_ir_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_ir_timer.function = proximity_timer_function;
	
	/* i2c Registration for probe/suspend/resume */				
	ret = IRsensor_i2c_register(&mPsensor_I2C);
	if (ret < 0)
		goto init_err;
	
	/* Hardware Register Initialization */
	Psensor_hw_client = Psensor_hw_getHardware();
	if(Psensor_hw_client == NULL)
		goto init_err;

	/* driver data structure initialize */
	ret = init_data();
	if (ret < 0)
		goto init_err;

	/* string copy the character of vendor and module number */
	strcpy(mPsensor_ATTR.info_type->vendor, Psensor_hw_client->vendor);
	strcpy(mPsensor_ATTR.info_type->module_number, Psensor_hw_client->module_number);
	
	/* Attribute */
	Psensor_ATTR_register(&mPsensor_ATTR);	
	if (ret < 0)
		goto init_err;
	
	/* Input Device */
	ret = IRsensor_report_register();
	if (ret < 0)
		goto init_err;	

	ASUS_P_SENSOR_IRQ = Psensor_gpio_register(g_i2c_client, &mIRsensor_GPIO);
	if (ASUS_P_SENSOR_IRQ < 0)
		goto init_err;	
	log("Driver INIT ---\n");
	return 0;

init_err:
	err("Driver INIT ERROR ---\n");
	return ret;
}

static void __exit IRsensor_exit(void)
{
	log("Driver EXIT +++\n");

	/* i2c Unregistration */	
	IRsensor_i2c_unregister();

	IRsensor_report_unregister();
	Psensor_ATTR_unregister();	
	
	wake_lock_destroy(&g_ir_wake_lock);
	mutex_destroy(&g_p_lock);
	mutex_destroy(&g_i2c_lock);
	kfree(g_ps_data);

	destroy_workqueue(Psensor_workqueue);
	destroy_workqueue(Psensor_delay_workqueue);
	
	log("Driver EXIT ---\n");
}

module_init(IRsensor_init);
module_exit(IRsensor_exit);

MODULE_AUTHOR("sr_Huang <sr_Huang@asus.com>");
MODULE_DESCRIPTION("Proximity Sensor");
MODULE_LICENSE("GPL");

