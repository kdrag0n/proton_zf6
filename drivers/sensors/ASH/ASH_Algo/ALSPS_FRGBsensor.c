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
//#include <linux/wakelock.h>
#include "ASH_Wakelock.h"
#include <linux/input/ASH.h>
#include "ALSPS_FRGBsensor.h"

#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>

/******************************/
/* Debug and Log System */
/*****************************/
#define MODULE_NAME			"ASH_ALGO"
#define SENSOR_TYPE_NAME	"ALSPS_FRGB"
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

/*************************************************/
/* ALSPS_FRGB Sensor Global Variables */
/************************************************/
static int ALSPS_FRGB_SENSOR_IRQ;
static int ALSPS_FRGB_SENSOR_INT;
static struct lsensor_data			*g_als_data;
static struct psensor_data		*g_ps_data;
static struct frgb_data			*g_frgb_data;
static struct ALSPS_FRGB_hw	*ALSPS_FRGB_hw_client;
static struct workqueue_struct 		*ALSPS_FRGB_workqueue;
static struct workqueue_struct 		*ALSPS_FRGB_delay_workqueue;
static struct mutex 				g_alsps_frgb_lock;
static struct wake_lock 			g_alsps_frgb_wake_lock;
static struct hrtimer 				g_alsps_frgb_timer;
static struct i2c_client 			*g_i2c_client;
static int g_als_last_lux = 0;
static int g_red_last_raw = -1;
static int g_green_last_raw = -1;
static int g_blue_last_raw = -1;
static int g_ir_last_raw = -1;
static char *g_error_mesg;
static int resume_flag = 0;

#define ALSPS_SUSPEND 0
#define ALSPS_RESUME 1
static int g_alsps_frgb_power_status = ALSPS_RESUME;

#define WAIT_I2C_DELAY 5

static bool g_frgb_polling_cancel_flag = false;

/**********************************/
/* ALSPS_FRGB Sensor IO Control */
/**********************************/
#define CM36656_I2C_NAME 					"CM36656"
#define ASUS_RGB_SENSOR_DATA_SIZE			5
#define ASUS_RGB_SENSOR_NAME_SIZE			32
#define ASUS_RGB_SENSOR_IOC_MAGIC			('A')	///< RGB sensor ioctl magic number 
#define ASUS_RGB_SENSOR_IOCTL_DATA_READ	_IOR(ASUS_RGB_SENSOR_IOC_MAGIC, 1, int[ASUS_RGB_SENSOR_DATA_SIZE])		///< RGB sensor ioctl command - Read data RGBW
#define ASUS_RGB_SENSOR_IOCTL_MODULE_NAME	_IOR(ASUS_RGB_SENSOR_IOC_MAGIC, 2, char[ASUS_RGB_SENSOR_NAME_SIZE])	///< RGB sensor ioctl command - Get module name
static int rgb_first_data_ready = 0;
bool camera_HAL_switch_on;
#define ENABLE_PROXIMITY_IOCTL_LIB 1
#define ENABLE_LIGHT_IOCTL_LIB 1
static int prox_open_count = 0;
static int light_open_count = 0;

/*********************************************/
/* ALSPS Front RGB Sensor Functions*/
/********************************************/
/*Device Layer Part*/
static int 	proximity_turn_onoff(bool bOn);
static int 	proximity_set_threshold(void);
static void proximity_polling_adc(struct work_struct *work);
static int 	light_turn_onoff(bool bOn);
static int 	light_get_lux(int adc);
static int 	light_get_accuracy_gain(void);
static void light_polling_lux(struct work_struct *work);
static int 	FRGB_turn_onoff(bool bOn);
static void FRGB_polling_raw(struct work_struct *work);

/*Interrupt Service Routine Part*/
static void ALSPS_FRGB_ist(struct work_struct *work);

/* Export Functions */
bool proximity_check_status(void);

/*Initialization Part*/
static int init_data(void);

/*Proximity auto calibration*/
static void proximity_autok(struct work_struct *work);
static int proximity_check_minCT(void);

/*Work Queue*/
static 		DECLARE_WORK(ALSPS_FRGB_ist_work,ALSPS_FRGB_ist);
static 		DECLARE_WORK(proximity_autok_work, proximity_autok);
static 		DECLARE_DELAYED_WORK(proximity_polling_adc_work, proximity_polling_adc);
static 		DECLARE_DELAYED_WORK(light_polling_lux_work, light_polling_lux);
static 		DECLARE_DELAYED_WORK(FRGB_polling_raw_work, FRGB_polling_raw);

/*Disable touch for detecting near when phone call*/
extern void ftxxxx_disable_touch(bool flag);
extern int get_audiomode(void);

static int pocket_mode_threshold = 0;
static int touch_enable = 1;
static int anti_oil_enable = 1;

#define LIGHT_SENSOR_POLLING_REPORT_THRESHOLD 5
static int light_sensor_log_count = 0;

/*****************************************/
/* ALS / PS / FRGB data structure */
/****************************************/
struct psensor_data 
{	
	int g_ps_calvalue_lo;						/* Proximitysensor setting low calibration value(adc) */
	int g_ps_calvalue_hi;						/* Proximitysensor setting high calibration value(adc) */
	int g_ps_calvalue_inf;						/* Proximitysensor setting inf calibration value(adc) */

	int g_ps_factory_cal_lo;				/* Proximitysensor factory low calibration value(adc) */
	int g_ps_factory_cal_hi;				/* Proximitysensor factory high calibration value(adc) */

	int g_ps_autok_min;
	int g_ps_autok_max;
	
	bool HAL_switch_on;						/* this var. means if HAL is turning on ps or not */
	bool Device_switch_on;					/* this var. means is turning on ps or not */	
	bool polling_mode;							/* Polling for adc of proximity */
	bool autok;							/*auto calibration status*/

	int int_counter;
	int event_counter;
	int crosstalk_diff;

	int selection;
};

struct lsensor_data 
{	
	int g_als_calvalue;						/* Lightsensor calibration value(adc) */
	int g_als_accuracy_gain;					/* Lightsensor Gain calibration value X LIGHT_GAIN_ACCURACY_CALVALUE*/
	int g_als_change_sensitivity;			/* Lightsensor Change sensitivity */
	int g_als_log_threshold;				/* Lightsensor Log Print Threshold */
	bool g_als_log_first_event;				/* Lightsensor Log Print Threshold */

	bool HAL_switch_on;						/* this var. means if HAL is turning on als or not */
	bool Device_switch_on;					/* this var. means if als hw is turn on or not */

	int int_counter;
	int event_counter;

	int selection;
};

struct frgb_data 
{	
	int g_frgb_log_threshold;				/* Lightsensor Log Print Threshold */

	bool HAL_switch_on;					/* this var. means if HAL is turning on als or not */
	bool Device_switch_on;					/* this var. means if als hw is turn on or not */

	int event_counter;
};

/*==================
 *|| I2c Stress Test Part ||
 *===================
 */

#ifdef CONFIG_I2C_STRESS_TEST

#include <linux/i2c_testcase.h>

#define I2C_TEST_Lsensor_FAIL (-1)
#define I2C_TEST_Psensor_FAIL (-1)

static int ALSPS_FRGB_I2C_stress_test(struct i2c_client *client)
{
	int lnResult = I2C_TEST_PASS;	
	int ret = 0;
	int adc = 0;
	int low_threshold = 0;
	int high_threshold = 0;

	i2c_log_in_test_case("TestIRSensorI2C ++\n");

	/* Check Hardware Support First */
	if(ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_turn_onoff== NULL) {
		err("proximity_hw_turn_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_get_adc == NULL) {
		err("proximity_hw_get_adc NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold == NULL) {
		err("proximity_hw_set_hi_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold == NULL) {
		err("proximity_hw_set_lo_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_turn_onoff== NULL) {
		err("light_hw_turn_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_get_adc == NULL) {
		err("light_hw_get_adc NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_set_hi_threshold == NULL) {
		err("light_hw_set_hi_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_set_lo_threshold == NULL) {
		err("light_hw_set_lo_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	
	/* Turn on Proximity and Light Sensor */
	if(!g_ps_data->Device_switch_on) {
		ret = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);
	}
	if(!g_als_data->Device_switch_on) {
		ret = ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_turn_onoff(true);
	}

	/* Proximity i2c read test */
	ret = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_get_adc();
	if(ret < 0){
		i2c_log_in_test_case("IRsensor Proximity Fail to get adc\n");
		lnResult = I2C_TEST_Psensor_FAIL;
		return lnResult;	
	}

	/* Proximity i2c write test */
	ret = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold(g_ps_data->g_ps_calvalue_hi);
	if(ret < 0){
		i2c_log_in_test_case("IRsensor Proximity Fail to set high threshold.\n");
		lnResult = I2C_TEST_Psensor_FAIL;
		return lnResult;	
	}
	ret = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold(g_ps_data->g_ps_calvalue_lo);
	if(ret < 0){
		i2c_log_in_test_case("IRsensor Proximity Fail to set low threshold. \n");
		lnResult = I2C_TEST_Psensor_FAIL;
		return lnResult;	
	}

	/* Light Sensor i2c read test */
	adc = ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_get_adc();
	if(adc < 0){
		i2c_log_in_test_case("IRsensor Light Sensor Fail to get adc\n");
		lnResult = I2C_TEST_Lsensor_FAIL;
		return lnResult;	
	}
	/* Light Sensor Low Threshold */	
	low_threshold = adc * (100 - LIGHT_CHANGE_MID_SENSITIVITY) / 100;

	/* Light Sensor High Threshold */
	high_threshold = adc * (100 + LIGHT_CHANGE_MID_SENSITIVITY) / 100;	
	if (high_threshold > ALSPS_FRGB_hw_client->mlsensor_hw->light_max_threshold)	
		high_threshold = ALSPS_FRGB_hw_client->mlsensor_hw->light_max_threshold;	
	
	ret = ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_set_hi_threshold(high_threshold);
	if(ret < 0) {
		i2c_log_in_test_case("IRsensor Light Sensor Fail to set high threshold. \n");
		lnResult = I2C_TEST_Lsensor_FAIL;
		return lnResult;	
	}
	
	ret = ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_set_lo_threshold(low_threshold);
	if(ret < 0) {
		i2c_log_in_test_case("IRsensor Light Sensor Fail to set low threshold. \n");
		lnResult = I2C_TEST_Lsensor_FAIL;
		return lnResult;	
	}

	if(!g_ps_data->HAL_switch_on) {
		ret = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);
	}
	if(!g_als_data->HAL_switch_on) {
		ret = ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_turn_onoff(false);
	}
	
	i2c_log_in_test_case("TestLSensorI2C --\n");
	return lnResult;
}

static struct i2c_test_case_info ALSPS_FRGB_TestCaseInfo[] =	{
	__I2C_STRESS_TEST_CASE_ATTR(ALSPS_FRGB_I2C_stress_test),
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
	if(ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_turn_onoff == NULL) {
		err("proximity_hw_turn_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_interrupt_onoff == NULL) {
		err("proximity_hw_interrupt_onoff NOT SUPPORT. \n");
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
			/*Enable INT*/
			ret = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_interrupt_onoff(true);	
			if(ret < 0){
				err("proximity_hw_interrupt_onoff(true) ERROR\n");
				return ret;
			}
			/*Power ON*/
			ret = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);	
			if(ret < 0){
				err("proximity_hw_turn_onoff(true) ERROR\n");
				return ret;
			}
		}
				
		/*enable IRQ only when proximity and light sensor is off*/
		if (g_ps_data->Device_switch_on == false && g_als_data->Device_switch_on == false) {
			dbg("[IRQ] Enable irq !! \n");
			enable_irq(ALSPS_FRGB_SENSOR_IRQ);
		}
		/*change the Device Status*/
		g_ps_data->Device_switch_on = true;
		/*check the polling mode*/
		if(g_ps_data->polling_mode == true) {
			queue_delayed_work(ALSPS_FRGB_delay_workqueue, &proximity_polling_adc_work, msecs_to_jiffies(1000));
			log("[Polling] Proximity polling adc START. \n");
		}

		/*Stage 2 : start polling proximity adc(500ms) to check min value*/
		if(true == g_ps_data->autok && g_ps_data->crosstalk_diff != 0){
			autok_delay = ns_to_ktime(PROXIMITY_AUTOK_POLLING * NSEC_PER_MSEC);
			hrtimer_start(&g_alsps_frgb_timer, autok_delay, HRTIMER_MODE_REL);
		}
		
	} else	{	/* power off */
		/*set turn off register*/
		if(g_ps_data->Device_switch_on == true){
			/*disable IRQ before switch off*/
			dbg("[IRQ] Disable irq !! \n");
			disable_irq_nosync(ALSPS_FRGB_SENSOR_IRQ);

			/*Power OFF*/
			ret = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);
			if(ret < 0){
				err("proximity_hw_turn_onoff(false) ERROR\n");
			}
			
			/*Disable INT*/
			ret = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_interrupt_onoff(false);
			if(ret < 0){
				err("proximity_hw_interrupt_onoff(false) ERROR\n");
			}

			/*reset the threshold data*/
			g_ps_data->g_ps_calvalue_hi = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hi_threshold_default;
			g_ps_data->g_ps_calvalue_lo = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_low_threshold_default;	
			g_ps_data->g_ps_calvalue_inf = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_crosstalk_default;	
	
			/*change the Device Status*/
			g_ps_data->Device_switch_on = false;			

			/*enable IRQ when light sensor is ON*/
			if (g_als_data->Device_switch_on == true) {
				dbg("[IRQ] Enable irq !! \n");
				enable_irq(ALSPS_FRGB_SENSOR_IRQ);
			}

			/*diable the timer*/
			if(g_ps_data->autok == true){
				hrtimer_cancel(&g_alsps_frgb_timer);
			}
		}		
	}	
	
	return ret;
}

static int proximity_set_threshold(void)
{
	int ret = 0;
	int temp = 0;

	/* Check Hardware Support First */
	if(ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold == NULL) {
		err("proximity_hw_set_hi_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold == NULL) {
		err("proximity_hw_set_lo_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	
	/*Set Proximity High Threshold*/
	ret = psensor_factory_read_high();

	/*For transition period from 3/5 to 2/4 +++*/
	if(0 == g_ps_data->selection)
		ret = psensor_factory_read_2cm();
	else if(1 == g_ps_data->selection)
		ret = psensor_factory_read_3cm();
	else
		err("INVALID selection : %d\n", g_ps_data->selection);
	/*For transition period from 3/5 to 2/4 ---*/
	
	if(ret > 0) {
	    g_ps_data->g_ps_calvalue_hi = ret;
		log("Proximity read High Calibration : %d\n", g_ps_data->g_ps_calvalue_hi);
	}else{
		err("Proximity read DEFAULT High Calibration : %d\n", g_ps_data->g_ps_calvalue_hi);
	}
	ret = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold(g_ps_data->g_ps_calvalue_hi);
	if(ret < 0){
		err("proximity_hw_set_hi_threshold ERROR. \n");
		return -ENOENT;
	}
	
	/*Set Proximity Low Threshold*/
	ret = psensor_factory_read_low();	

	/*For transition period from 3/5 to 2/4 +++*/
	if(0 == g_ps_data->selection)
		ret = psensor_factory_read_4cm();
	else if(1 == g_ps_data->selection)
		ret = psensor_factory_read_5cm();
	else
		err("INVALID selection : %d\n", g_ps_data->selection);
	/*For transition period from 3/5 to 2/4 ---*/
	
	if(ret > 0) {
	    g_ps_data->g_ps_calvalue_lo = ret;
		log("Proximity read Low Calibration : %d\n", g_ps_data->g_ps_calvalue_lo);
	}else{
		err("Proximity read DEFAULT Low Calibration : %d\n", g_ps_data->g_ps_calvalue_lo);
	}
	
	if(anti_oil_enable == 1){
		temp = (g_ps_data->g_ps_calvalue_hi - g_ps_data->g_ps_calvalue_lo) / 3;
		g_ps_data->g_ps_calvalue_lo = temp + g_ps_data->g_ps_calvalue_lo;
		log("Proximity apply anti-oil, update low threshold to %d\n", g_ps_data->g_ps_calvalue_lo);
	}
	ret = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold(g_ps_data->g_ps_calvalue_lo);
	if(ret < 0){
		err("proximity_hw_set_hi_threshold ERROR. \n");
		return -ENOENT;
	}
	
	ret = psensor_factory_read_1cm();
	if(ret > 0){
		pocket_mode_threshold = ret;
		log("Proximity read Pocket Mode Calibration : %d\n", pocket_mode_threshold);
	}else{
		pocket_mode_threshold = 470;
		err("Proximity read DEFAULT Pocket Mode Calibration : %d\n", pocket_mode_threshold);
	}
	
	return 0;
}

static void proximity_polling_adc(struct work_struct *work)
{
	int adc = 0;

	/* Check Hardware Support First */
	if(ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_get_adc == NULL) {
		err("proximity_hw_get_adc NOT SUPPORT. \n");		
	}
	
	adc= ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_get_adc();
	log("[Polling] Proximity get adc = %d\n", adc);
	
	if(g_ps_data->Device_switch_on == true)
		queue_delayed_work(ALSPS_FRGB_delay_workqueue, &proximity_polling_adc_work, msecs_to_jiffies(1000));
	else
		log("[Polling] Proximity polling adc STOP. \n");
}

static int light_turn_onoff(bool bOn)
{
	int ret=0;

	/* Check Hardware Support First */
	if(ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_turn_onoff == NULL) {
		err("light_hw_turn_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_interrupt_onoff == NULL) {
		err("light_hw_interrupt_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_set_hi_threshold == NULL) {
		err("light_hw_set_hi_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_set_lo_threshold == NULL) {
		err("light_hw_set_lo_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	
	if (bOn == 1)	{	/* power on */	
		if(1 == resume_flag){
			resume_flag=0;
		}else{
			light_get_accuracy_gain();
		}
		log("[Cal] Light Sensor Set Accuracy Gain : %d\n", g_als_data->g_als_accuracy_gain);

		if(g_als_data->Device_switch_on == false) {
			ret = ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_set_hi_threshold(0);
			if(ret < 0){
				err("light_hw_set_hi_threshold ERROR. \n");
				return -ENOENT;
			}
			ret = ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_set_lo_threshold(0);
			if(ret < 0){
				err("light_hw_set_lo_threshold ERROR. \n");
				return -ENOENT;
			}

			/*Enable INT*/
			ret = ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_interrupt_onoff(true);
			if(ret < 0){
				err("light_hw_interrupt_onoff(true) ERROR. \n");
				return -ENOENT;
			}
			/*Power ON*/
			ret = ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_turn_onoff(true);
			if(ret < 0){
				err("light_hw_turn_onoff(true) ERROR. \n");
				return -ENOENT;
			}
			
		}
		/*enable IRQ only when proximity and light sensor is off*/
		if (g_ps_data->Device_switch_on == false && g_als_data->Device_switch_on == false) {
			dbg("[IRQ] Enable irq !! \n");
			enable_irq(ALSPS_FRGB_SENSOR_IRQ);
		}
		g_als_data->Device_switch_on = true;
		g_als_data->g_als_log_first_event = true;
	} else	{	/* power off */	
		/*set turn off register*/
		if(g_als_data->Device_switch_on == true){
			/*disable IRQ before switch off*/		
			dbg("[IRQ] Disable irq !! \n");
			disable_irq_nosync(ALSPS_FRGB_SENSOR_IRQ);

			/*Power OFF*/
			if(!g_frgb_data->HAL_switch_on) {
				ret = ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_turn_onoff(false);
				if(ret < 0){
					err("light_hw_turn_onoff(false) ERROR. \n");
					return ret;
				}
			}
			/*Disbale INT*/
			ret = ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_interrupt_onoff(false);
			if(ret < 0){
				err("light_hw_interrupt_onoff(false) ERROR. \n");
				return ret;
			}

			g_als_data->Device_switch_on = false;

			/*enable IRQ when proximity sensor is ON*/
			if (g_ps_data->Device_switch_on == true) {
				dbg("[IRQ] Enable irq !! \n");
				enable_irq(ALSPS_FRGB_SENSOR_IRQ);
			}
		}
	}

	return ret;
}

static int light_suspend_turn_off(bool bOn)
{
	int ret=0;

	/* Check Hardware Support First */
	if(ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_turn_onoff == NULL) {
		err("light_hw_turn_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_interrupt_onoff == NULL) {
		err("light_hw_interrupt_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}

	/* power off */	
	/*set turn off register*/
	if(g_als_data->Device_switch_on == true){
		/*disable IRQ before switch off*/		
		dbg("[IRQ] Disable irq !! \n");
		disable_irq_nosync(ALSPS_FRGB_SENSOR_IRQ);

		/*Power OFF*/
		if(!g_frgb_data->HAL_switch_on) {
			ret = ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_turn_onoff(false);
			if(ret < 0){
				err("light_hw_turn_onoff(false) ERROR. \n");
				return ret;
			}
		}

		/*Disbale INT*/
		ret = ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_interrupt_onoff(false);
		if(ret < 0){
			err("light_hw_interrupt_onoff(false) ERROR. \n");
			return ret;
		}

		g_als_data->Device_switch_on = false;
		/*enable IRQ when proximity sensor is ON*/
		if (g_ps_data->Device_switch_on == true) {
			dbg("[IRQ] Enable irq !! \n");
			enable_irq(ALSPS_FRGB_SENSOR_IRQ);
		}
	}

	return ret;
}

static int light_get_lux(int adc)
{
	int lux = 0;

	if(adc < 0) {
		err("Light Sensor get Lux ERROR. (adc < 0)\n");
		return 0;
	}
	
	lux = (adc * g_als_data->g_als_accuracy_gain) / LIGHT_GAIN_ACCURACY_CALVALUE;	
	
	//if(lux > LIGHT_MAX_LUX)
	//	lux = LIGHT_MAX_LUX;
	
	return lux;
}

static int 	light_get_accuracy_gain(void)
{
	int cal = 0;
	int gainvalue = 0;

	/* Light Sensor Read Calibration*/
	cal = lsensor_factory_read();
	
	if(0 == g_als_data->selection)
		cal = lsensor_factory_read_50ms();
	else if(1 == g_als_data->selection)
		cal = lsensor_factory_read_100ms();
	else
		err("INVALID selection : %d\n", g_als_data->selection);
	
	if(cal > 0 )
		g_als_data->g_als_calvalue = cal;
	
	gainvalue = (1000*LIGHT_GAIN_ACCURACY_CALVALUE)/
				(g_als_data->g_als_calvalue);

	g_als_data->g_als_accuracy_gain = gainvalue;
	
	return gainvalue;
}

static void light_polling_lux(struct work_struct *work)
{
	int adc = 0;
	int lux = 0;
	static int count = 0;

	/* Check Hardware Support First */
	if(ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_get_adc == NULL) {
		err("light_hw_get_adc NOT SUPPORT. \n");		
	}

mutex_lock(&g_alsps_frgb_lock);

	if(g_als_data->HAL_switch_on == true) {
		adc = ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_get_adc();
		if ((0 == adc) && (count < 25)) {
			log("[Polling] Light Sensor retry for get adc\n");
			queue_delayed_work(ALSPS_FRGB_delay_workqueue, &light_polling_lux_work, msecs_to_jiffies(LIGHT_TURNON_DELAY_TIME));
			count++;
		} else {
			count = 0;
			lux = light_get_lux(adc);
			log("[Polling] Light Sensor Report lux : %d (adc = %d)\n", lux, adc);
			lsensor_report_lux(lux);
		}
	}
	
mutex_unlock(&g_alsps_frgb_lock);
}

static int FRGB_turn_onoff(bool bOn)
{
	int ret=0;

	/* Check Hardware Support First */
	if(ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_turn_onoff == NULL) {
		err("frgb_hw_turn_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	
	if (bOn == 1)	{	/* power on */
		if(g_frgb_data->Device_switch_on == false) {
			ret = ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_turn_onoff(true);
			if(ret < 0){
				err("frgb_hw_turn_onoff(true) ERROR. \n");
				return -ENOENT;
			}
		}
		g_frgb_data->Device_switch_on = true;		
	} else{	/* power off */	
		/*set turn off register*/
		if(g_frgb_data->Device_switch_on == true) {
			if(!g_als_data->HAL_switch_on) {
				ret = ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_turn_onoff(false);
				if(ret < 0){
					err("frgb_hw_turn_onoff(false) ERROR. \n");
				}
			}
			g_frgb_data->Device_switch_on = false;	
		}
	}
	
	return ret;
}

static int FRGB_suspend_turn_off(bool bOn)
{
	int ret=0;

	/* Check Hardware Support First */
	if(ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_turn_onoff == NULL) {
		err("frgb_hw_turn_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	
	/* power off */	
	/*set turn off register*/
	if(g_frgb_data->Device_switch_on == true) {
		if(!g_als_data->HAL_switch_on) {
			ret = ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_turn_onoff(false);
			if(ret < 0){
				err("frgb_hw_turn_onoff(false) ERROR. \n");
			}
		}
		g_frgb_data->Device_switch_on = false;	
	}
	
	return ret;
}

static void FRGB_polling_raw(struct work_struct *work)
{
	int red = 0, green = 0, blue = 0, ir = 0;
	int data[4]={0,0,0,0};
	int frgb_log_threshold = 0;
	static int count = 0;
	int lux = 0;

	/* Check Hardware Support First */
	if(ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_get_red == NULL) {
		err("frgb_hw_get_red NOT SUPPORT. \n");		
	}
	if(ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_get_green == NULL) {
		err("frgb_hw_get_green NOT SUPPORT. \n");		
	}
	if(ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_get_blue == NULL) {
		err("frgb_hw_get_blue NOT SUPPORT. \n");		
	}
	if(ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_get_ir == NULL) {
		err("frgb_hw_get_ir NOT SUPPORT. \n");		
	}

	/* frgb sensor go to suspend, cancel frgb get raw data polling */
	if(g_alsps_frgb_power_status == ALSPS_SUSPEND){
		g_frgb_polling_cancel_flag = true;
		log("frgb sensor has suspended, cancel frgb raw data polling!!");
		return;
	}

mutex_lock(&g_alsps_frgb_lock);
	if(g_frgb_data->HAL_switch_on == true) {
		/* Light Sensor Report the first real event*/

		red = ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_get_red();
		green = ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_get_green();
		blue = ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_get_blue();
		ir = ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_get_ir();

		frgb_log_threshold = FRGB_LOG_THRESHOLD;
		
		/* Set the interface log threshold (1st priority) */
		if(g_frgb_data->g_frgb_log_threshold >= 0)
			frgb_log_threshold = g_frgb_data->g_frgb_log_threshold;
		
		if(abs(g_red_last_raw - red) > frgb_log_threshold ||
			abs(g_green_last_raw - green) > frgb_log_threshold||
			abs(g_blue_last_raw - blue) > frgb_log_threshold||
			abs(g_ir_last_raw - ir) > frgb_log_threshold){
			log("[Polling] Front RGB Sensor Report raw , red=%d, green=%d, blue=%d, ir=%d\n", red, green, blue, ir);
		}

		if ((0 == red || 0 == green || 0 == blue || 0 == ir) && (count < 7) && (!rgb_first_data_ready)) {
			queue_delayed_work(ALSPS_FRGB_delay_workqueue, &FRGB_polling_raw_work, msecs_to_jiffies(FRGB_POLLING_FIRST_RAW));
			count++;
		} else {
			data[0] = red;
			data[1] = green;
			data[2] = blue;
			data[3] = ir;
			rgb_first_data_ready = 1;
			count = 0;
			FRGBsensor_report_raw(data, sizeof(data));

			g_red_last_raw=red;
			g_green_last_raw=green;
			g_blue_last_raw=blue;
			g_ir_last_raw=ir;

			/* Light sensor polling in the low lux */
			if(green < LIGHT_SENSOR_POLLING_REPORT_THRESHOLD && green >= 0){
				lux = light_get_lux(green);
				lsensor_report_lux(lux);
				if(light_sensor_log_count == 10) {
					log("[Polling] Front RGB Sensor Report lux=%d (adc=%d)\n", lux, green);
					light_sensor_log_count = 0;
				}
				light_sensor_log_count++;
			}
		
			/*FRGB sensor polling the raw data. */
			queue_delayed_work(ALSPS_FRGB_delay_workqueue, &FRGB_polling_raw_work, msecs_to_jiffies(FRGB_POLLING_TIME));
		}
	} else {
		data[0] = -1;
		data[1] = -1;
		data[2] = -1;
		data[3] = -1;
		g_red_last_raw = -1;
		g_green_last_raw = -1;
		g_blue_last_raw = -1;
		g_ir_last_raw = -1;
		FRGBsensor_report_raw(data, sizeof(data));
	}
mutex_unlock(&g_alsps_frgb_lock);
}

/********************************/
/*Proximity sensor Info Type*/
/*******************************/
static psensor_info_type mpsensor_info_type = {{0}};

/***************************/
/*Light sensor Info Type*/
/***************************/
static lsensor_info_type mlsensor_info_type = {{0}};

/**********************************/
/*Front RGB sensor Info Type*/
/*********************************/
static FRGBsensor_info_type mFRGBsensor_info_type = {{0}};
	
/*************************/
/*Calibration Function*/
/************************/
int mproximity_show_calibration_hi(void)
{
	int calvalue;
	int ret = 0;	

	calvalue = psensor_factory_read_high();

	/*For transition period from 3/5 to 2/4 +++*/
	if(0 == g_ps_data->selection)
		calvalue = psensor_factory_read_2cm();
	else if(1 == g_ps_data->selection)
		calvalue = psensor_factory_read_3cm();
	else
		err("INVALID selection : %d\n", g_ps_data->selection);
	/*For transition period from 3/5 to 2/4 ---*/
	
	if(calvalue > 0) {
	    	g_ps_data->g_ps_calvalue_hi = calvalue;
		log("Proximity read High Calibration : %d\n", g_ps_data->g_ps_calvalue_hi);
	}else{
		err("Proximity read DEFAULT High Calibration : %d\n", g_ps_data->g_ps_calvalue_hi);
	}

	ret = g_ps_data->g_ps_calvalue_hi;
	dbg("Proximity show High Calibration: %d\n", ret);
	return ret;
}

int mproximity_store_calibration_hi(int calvalue)
{
	if(calvalue <= 0) {
		err("Proximity store High Calibration with NON-POSITIVE value. (%d) \n", calvalue);
		return -EINVAL;	
	}
	log("Proximity store High Calibration: %d\n", calvalue);
	psensor_factory_write_high(calvalue);
	
	/*For transition period from 3/5 to 2/4 +++*/
	if(0 == g_ps_data->selection)
		psensor_factory_write_2cm(calvalue);
	else if(1 == g_ps_data->selection)
		psensor_factory_write_3cm(calvalue);
	else
		err("INVALID selection : %d\n", g_ps_data->selection);
	/*For transition period from 3/5 to 2/4 ---*/
	
	proximity_set_threshold();
	
	return 0;
}

int mproximity_show_calibration_lo(void)
{
	int calvalue;
	int ret = 0;

	calvalue = psensor_factory_read_low();	

	/*For transition period from 3/5 to 2/4 +++*/
	if(0 == g_ps_data->selection)
		calvalue = psensor_factory_read_4cm();
	else if(1 == g_ps_data->selection)
		calvalue = psensor_factory_read_5cm();
	else
		err("INVALID selection : %d\n", g_ps_data->selection);
	/*For transition period from 3/5 to 2/4 ---*/
	
	if(calvalue > 0) {
	    	g_ps_data->g_ps_calvalue_lo = calvalue;
		log("Proximity read Low Calibration : %d\n", g_ps_data->g_ps_calvalue_lo);
	}else{
		err("Proximity read DEFAULT Low Calibration : %d\n", g_ps_data->g_ps_calvalue_lo);
	}
	
	ret = g_ps_data->g_ps_calvalue_lo;
	dbg("Proximity show Low Calibration: %d\n", ret);
	return ret;
}

int mproximity_store_calibration_lo(int calvalue)
{
	if(calvalue <= 0) {
		err("Proximity store Low Calibration with NON-POSITIVE value. (%d) \n", calvalue);
		return -EINVAL;
	}
	log("Proximity store Low Calibration: %d\n", calvalue);
	psensor_factory_write_low(calvalue);

	/*For transition period from 3/5 to 2/4 +++*/
	if(0 == g_ps_data->selection)
		psensor_factory_write_4cm(calvalue);
	else if(1 == g_ps_data->selection)
		psensor_factory_write_5cm(calvalue);
	else
		err("INVALID selection : %d\n", g_ps_data->selection);
	/*For transition period from 3/5 to 2/4 ---*/
	
	proximity_set_threshold();	

	return 0;
}

int mproximity_show_calibration_inf(void)
{
	int calvalue;
	calvalue = psensor_factory_read_inf();
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
	psensor_factory_write_inf(calvalue);
	
	return 0;
}

int mproximity_show_adc(void)
{
	int adc = 0;
	int ret;
	
	if(ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_get_adc == NULL) {
		err("proximity_hw_get_adc NOT SUPPORT. \n");
		return -EINVAL;
	}

	mutex_lock(&g_alsps_frgb_lock);
	
	if(g_ps_data->Device_switch_on == false){
		ret = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);	
		if(ret < 0){
			err("proximity_hw_turn_onoff(true) ERROR\n");
			return ret;
		}
		msleep(PROXIMITY_TURNON_DELAY_TIME);
	}
	
	adc = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_get_adc();
	dbg("mproximity_show_adc : %d \n", adc);
	
	if(g_ps_data->HAL_switch_on == false){
		ret = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);	
		if(ret < 0){
			err("proximity_hw_turn_onoff(false) ERROR\n");
			return ret;
		}			
	}
	
	mutex_unlock(&g_alsps_frgb_lock);
	
	return adc;
}

int mlight_show_calibration(void)
{
	int calvalue;
	int ret = 0;
	
	calvalue = lsensor_factory_read();

	if(0 == g_als_data->selection)
		calvalue = lsensor_factory_read_50ms();
	else if(1 == g_als_data->selection)
		calvalue = lsensor_factory_read_100ms();
	else
		err("INVALID selection : %d\n", g_als_data->selection);
	
	if(calvalue > 0 )
		g_als_data->g_als_calvalue = calvalue;
	
	ret = g_als_data->g_als_calvalue;	
	dbg("Light Sensor show Calibration: %d\n", ret);
	return ret;
}	

int mlight_store_calibration(int calvalue)
{
	if(calvalue <= 0) {
		err("Light Sensor store Calibration with NON-POSITIVE value. (%d) \n", calvalue);
		return -EINVAL;
	}
	log("Light Sensor store Calibration: %d\n", calvalue);
	lsensor_factory_write(calvalue);

	if(0 == g_als_data->selection)
		lsensor_factory_write_50ms(calvalue);
	else if(1 == g_als_data->selection)
		lsensor_factory_write_100ms(calvalue);
	else
		err("INVALID selection : %d\n", g_als_data->selection);
	
	if(calvalue > 0 )
		g_als_data->g_als_calvalue = calvalue;
	light_get_accuracy_gain();
			
	return 0;
}

int mlight_show_adc(void)
{
	int adc = 0;
	if(ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_get_adc == NULL) {
		err("light_hw_get_adc NOT SUPPORT. \n");
		return -EINVAL;
	}

	mutex_lock(&g_alsps_frgb_lock);

	if (!g_als_data->Device_switch_on) {
		light_turn_onoff(true);
		msleep(LIGHT_TURNON_DELAY_TIME);
	}

	adc = ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_get_adc();

	dbg("mlight_show_adc : %d \n", adc);

	if (!g_als_data->HAL_switch_on) {
		light_turn_onoff(false);
	}
	
	mutex_unlock(&g_alsps_frgb_lock);
	return adc;
}

int mlight_show_gain(void)
{
	int gainvalue;
	gainvalue = light_get_accuracy_gain();
	dbg("Light Sensor show Gain Calibration: %d.%d\n",
		gainvalue/LIGHT_GAIN_ACCURACY_CALVALUE, gainvalue%LIGHT_GAIN_ACCURACY_CALVALUE);

	return gainvalue;
}

int mFRGB_show_red(void)
{
	int adc = 0;
	int iter = 0;
	
	if(ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_get_red == NULL) {
		err("frgb_hw_get_red NOT SUPPORT. \n");
		return -EINVAL;
	}

	mutex_lock(&g_alsps_frgb_lock);
	if (!g_frgb_data->Device_switch_on) {
		FRGB_turn_onoff(true);
	}

	for(iter=0;iter<15;iter++) {
		adc = ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_get_red();
		if(adc) {
			break;
		}
		msleep(10);
	}

	dbg("mFRGB_show_red : %d \n", adc);

	if (!g_frgb_data->HAL_switch_on) {
		FRGB_turn_onoff(false);
	}
	
	mutex_unlock(&g_alsps_frgb_lock);
	return adc;
}

int mFRGB_show_green(void)
{
	int adc = 0;
	int iter = 0;

	if(ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_get_green == NULL) {
		err("frgb_hw_get_green NOT SUPPORT. \n");
		return -EINVAL;
	}

	mutex_lock(&g_alsps_frgb_lock);

	if (!g_frgb_data->Device_switch_on) {
		FRGB_turn_onoff(true);
	}

	for(iter=0;iter<15;iter++) {
		adc = ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_get_green();
		if(adc) {
			break;
		}
		msleep(10);
	}

	dbg("mFRGB_show_green : %d \n", adc);

	if (!g_frgb_data->HAL_switch_on) {
		FRGB_turn_onoff(false);
	}
	
	mutex_unlock(&g_alsps_frgb_lock);
	return adc;
}

int mFRGB_show_blue(void)
{
	int adc = 0;
	int iter = 0;
	
	if(ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_get_blue == NULL) {
		err("frgb_hw_get_blue NOT SUPPORT. \n");
		return -EINVAL;
	}

	mutex_lock(&g_alsps_frgb_lock);
	if (!g_frgb_data->Device_switch_on) {
		FRGB_turn_onoff(true);
	}

	for(iter=0;iter<15;iter++) {
		adc = ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_get_blue();
		if(adc) {
			break;
		}
		msleep(10);
	}

	dbg("mFRGB_show_blue : %d \n", adc);

	if (!g_frgb_data->HAL_switch_on) {
		FRGB_turn_onoff(false);
	}
	
	mutex_unlock(&g_alsps_frgb_lock);
	return adc;
}

int mFRGB_show_ir(void)
{
	int adc = 0;
	int iter = 0;
	
	if(ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_get_ir == NULL) {
		err("frgb_hw_get_ir NOT SUPPORT. \n");
		return -EINVAL;
	}

	mutex_lock(&g_alsps_frgb_lock);
	if (!g_frgb_data->Device_switch_on) {
		FRGB_turn_onoff(true);
	}

	for(iter=0;iter<15;iter++) {
		adc = ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_get_ir();
		if(adc) {
			break;
		}
		msleep(10);
	}

	dbg("mFRGB_show_ir : %d \n", adc);

	if (!g_frgb_data->HAL_switch_on) {
		FRGB_turn_onoff(false);
	}
	
	mutex_unlock(&g_alsps_frgb_lock);
	return adc;
}

static psensor_ATTR_Calibration mpsensor_ATTR_Calibration = {
	.proximity_show_calibration_hi = mproximity_show_calibration_hi,
	.proximity_store_calibration_hi = mproximity_store_calibration_hi,
	.proximity_show_calibration_lo = mproximity_show_calibration_lo,
	.proximity_store_calibration_lo = mproximity_store_calibration_lo,
	.proximity_show_calibration_inf = mproximity_show_calibration_inf ,
	.proximity_store_calibration_inf  = mproximity_store_calibration_inf ,
	.proximity_show_adc = mproximity_show_adc,
};

static lsensor_ATTR_Calibration mlsensor_ATTR_Calibration = {
	.light_show_calibration = mlight_show_calibration,
	.light_store_calibration = mlight_store_calibration,
	.light_show_gain = mlight_show_gain,
	.light_show_adc = mFRGB_show_green,
};

static FRGBsensor_ATTR_Calibration mFRGBsensor_ATTR_Calibration = {
	.FRGB_show_red = mFRGB_show_red,
	.FRGB_show_green = mFRGB_show_green,
	.FRGB_show_blue = mFRGB_show_blue,
	.FRGB_show_ir = mFRGB_show_ir,
};

/********************/
/*BMMI Function*/
/*******************/
bool mproximity_show_atd_test(void)
{
	int ret=0;
	int round=0;

	ret = ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_check_ID();
	if(ret < 0){
		err("Proximity ATD test check ID ERROR\n");
		goto proximity_atd_test_fail;
	}
	
	if(g_ps_data->Device_switch_on == false){
		ret = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);		
		if(ret < 0){
			err("Proximity ATD test turn on ERROR\n");
			goto proximity_atd_test_fail;
		}
	}	
	
	for(;round<5; round++){
		ret = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_get_adc();
		if(ret < 0){
			err("Proximity ATD test get adc ERROR\n");
			goto proximity_atd_test_fail;
		}
		msleep(100);
	}	

	if(g_ps_data->HAL_switch_on == false){
		ret = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);	
		if(ret < 0){
			err("Proximity ATD test turn off ERROR\n");
			goto proximity_atd_test_fail;
		}
	}

	return true;
proximity_atd_test_fail:
	return false;
}

bool mlight_show_atd_test(void)
{
	int ret=0;
	int round=0;

	ret = ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_check_ID();
	if(ret < 0){
		err("Light Sensor ATD test check ID ERROR\n");
		goto light_atd_test_fail;
	}
	
	if (!g_als_data->Device_switch_on) {
		//ret = ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_turn_onoff(true);
		ret = light_turn_onoff(true);
		if(ret < 0){
			err("Light Sensor ATD test turn on ERROR\n");
			goto light_atd_test_fail;
		}
		msleep(LIGHT_TURNON_DELAY_TIME);
	}	
	
	for(; round<5; round++){
		ret = ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_get_adc();
		if(ret < 0){
			err("Light Sensor ATD test get G adc ERROR\n");
			goto light_atd_test_fail;
		}
		msleep(100);
	}	
	
	if (!g_als_data->HAL_switch_on) {
		//ret = ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_turn_onoff(false);
		ret = light_turn_onoff(false);
		if(ret < 0){
			err("Light Sensor ATD test turn off ERROR\n");
			goto light_atd_test_fail;
		}
	}

	return true;
light_atd_test_fail:
	return false;

}

bool mFRGB_show_atd_test(void)
{
	int ret=0;
	int round=0;

	ret = ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_check_ID();
	if(ret < 0){
		err("FRGB Sensor ATD test check ID ERROR\n");
		goto frgb_atd_test_fail;
	}
	
	if (!g_frgb_data->Device_switch_on) {
		//ret = ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_turn_onoff(true);
		ret = FRGB_turn_onoff(true);
		if(ret < 0){
			err("FRGB Sensor ATD test turn on ERROR\n");
			goto frgb_atd_test_fail;
		}
	}
	
	for(; round<5; round++){
		ret = ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_get_red();
		if(ret < 0){
			err("FRGB Sensor ATD test get RED Channel ERROR\n");
			goto frgb_atd_test_fail;
		}
		ret = ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_get_green();
		if(ret < 0){
			err("FRGB Sensor ATD test get GREEN Channel ERROR\n");
			goto frgb_atd_test_fail;
		}
		ret = ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_get_blue();
		if(ret < 0){
			err("FRGB Sensor ATD test get BLUE Channel ERROR\n");
			goto frgb_atd_test_fail;
		}
		ret = ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_get_ir();
		if(ret < 0){
			err("FRGB Sensor ATD test get IR Channel ERROR\n");
			goto frgb_atd_test_fail;
		}
		msleep(100);
	}	
	
	if (!g_frgb_data->HAL_switch_on) {
		//ret = ALSPS_FRGB_hw_client->mFRGB_hw->frgb_hw_turn_onoff(false);
		ret = FRGB_turn_onoff(false);
		if(ret < 0){
			err("FRGB Sensor ATD test turn off ERROR\n");
			goto frgb_atd_test_fail;
		}
	}

	return true;
frgb_atd_test_fail:
	return false;

}

static psensor_ATTR_BMMI mpsensor_ATTR_BMMI = {
	.proximity_show_atd_test = mproximity_show_atd_test,
};

static lsensor_ATTR_BMMI mlsensor_ATTR_BMMI = {
	.light_show_atd_test = mlight_show_atd_test,
};

static FRGBsensor_ATTR_BMMI mFRGBsensor_ATTR_BMMI = {
	.FRGB_show_atd_test = mFRGB_show_atd_test,
};

/************************/
/*Hardware Function*/
/***********************/
int mproximity_show_reg(uint8_t addr)
{
	int value;
	if(ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_get_register == NULL) {
		err("proximity_hw_get_register NOT SUPPORT. \n");
		return -EINVAL;
	}

	value = ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_get_register(addr);
	log("mproximity_show_reg, addr=%02X, value=%02X.\n", addr, value);
	return value;
}

int mproximity_store_reg(uint8_t addr, int value)
{	
	if(ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_set_register == NULL) {
		err("proximity_hw_set_register NOT SUPPORT. \n");
		return -EINVAL;
	}

	ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_set_register(addr, value);
	log("mproximity_store_reg, addr=%02X, value=%02X.\n", addr, value);
	return 0;
}

int mlight_show_reg(uint8_t addr)
{
	int value;
	if(ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_get_register == NULL) {
		err("light_hw_get_register NOT SUPPORT. \n");
		return -EINVAL;
	}

	value = ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_get_register(addr);
	log("mlight_show_reg, addr=%02X, value=%02X.\n", addr, value);
	return value;
}

int mlight_store_reg(uint8_t addr, int value)
{	
	if(ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_set_register == NULL) {
		err("light_hw_set_register NOT SUPPORT. \n");
		return -EINVAL;
	}

	ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_set_register(addr, value);
	log("mlight_store_reg, addr=%02X, value=%02X.\n", addr, value);
	return 0;
}

int mFRGB_show_reg(uint8_t addr)
{
	int value;
	if(ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_get_register == NULL) {
		err("frgb_hw_get_register NOT SUPPORT. \n");
		return -EINVAL;
	}

	value = ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_get_register(addr);
	log("mFRGB_show_reg, addr=%02X, value=%02X.\n", addr, value);
	return value;
}

int mFRGB_store_reg(uint8_t addr, int value)
{	
	if(ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_set_register == NULL) {
		err("frgb_hw_set_register NOT SUPPORT. \n");
		return -EINVAL;
	}

	ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_set_register(addr, value);
	log("mFRGB_store_reg, addr=%02X, value=%02X.\n", addr, value);
	return 0;
}

static psensor_ATTR_Hardware mpsensor_ATTR_Hardware = {
	.proximity_show_reg = mproximity_show_reg,
	.proximity_store_reg = mproximity_store_reg,
};

static lsensor_ATTR_Hardware mlsensor_ATTR_Hardware = {
	.light_show_reg = mlight_show_reg,
	.light_store_reg = mlight_store_reg,
};

static FRGBsensor_ATTR_Hardware mFRGBsensor_ATTR_Hardware = {
	.FRGB_show_reg = mFRGB_show_reg,
	.FRGB_store_reg = mFRGB_store_reg,
};

/******************/
/*HAL Function*/
/*****************/
bool mproximity_show_switch_onoff(void)
{	
	return g_ps_data->Device_switch_on;
}

int mproximity_store_switch_onoff(bool bOn)
{
	int adc_value, threshold_high;
	mutex_lock(&g_alsps_frgb_lock);
	dbg("Proximity switch = %d.\n", bOn);		
	if ((g_ps_data->Device_switch_on != bOn))	{						
		if (bOn == true)	{
			/* Turn on Proxomity */
			g_ps_data->HAL_switch_on = true;
			proximity_turn_onoff(true);

			msleep(PROXIMITY_TURNON_DELAY_TIME);
			
			/* check if send first AWAY */
			adc_value = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_get_adc();
			threshold_high =  (g_ps_data->g_ps_calvalue_hi);
			log("Proximity adc_value=%d, threshold_high=%d\n", adc_value, threshold_high);
			if (adc_value < threshold_high) {
				psensor_report_abs(PSENSOR_REPORT_PS_AWAY);
				log("Proximity Report First Away abs.\n");
			}			
		} else	{
			/* Turn off Proxomity */
			g_ps_data->HAL_switch_on = false;				
			proximity_turn_onoff(false);
			psensor_report_abs(-1);
//#ifdef ASUS_DISABLE_TOUCH_PORTING_COMPLETED
			//ftxxxx_disable_touch(false);
			//touch_enable = 1;
//#endif
		}			
	}else{
		log("Proximity is already %s", bOn?"ON":"OFF");
	}
	mutex_unlock(&g_alsps_frgb_lock);
	
	return 0;
}

bool mlight_show_switch_onoff(void)
{
	return g_als_data->Device_switch_on;
}

int mlight_store_switch_onoff(bool bOn)
{
	mutex_lock(&g_alsps_frgb_lock);
	dbg("Light Sensor switch = %d.\n", bOn);		
	if ((g_als_data->Device_switch_on != bOn)) {					
		if (bOn == true)	{
			/* Turn on Light Sensor */
			g_als_data->HAL_switch_on = true;
			light_turn_onoff(true);
			/*light sensor polling the first real event after delayed time. */
			queue_delayed_work(ALSPS_FRGB_delay_workqueue, &light_polling_lux_work, msecs_to_jiffies(LIGHT_TURNON_DELAY_TIME));
		} else	{
			/* Turn off Light Sensor */
			g_als_data->HAL_switch_on = false;
			light_turn_onoff(false);
			/* Report lux=-1 when turn off */
			lsensor_report_lux(-1);
		}			
	}else{
		log("Light Sensor is already %s", bOn?"ON":"OFF");
		queue_delayed_work(ALSPS_FRGB_delay_workqueue, &light_polling_lux_work, msecs_to_jiffies(LIGHT_TURNON_DELAY_TIME));
	}
	mutex_unlock(&g_alsps_frgb_lock);

	return 0;
}

int mlight_show_lux(void)
{
	int adc = 0;
	int lux = 0;

	mutex_lock(&g_alsps_frgb_lock);

	if (!g_als_data->Device_switch_on) {
		light_turn_onoff(true);
	}
	
	msleep(LIGHT_TURNON_DELAY_TIME);

	adc = ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_get_adc();

	lux = light_get_lux(adc);
	dbg("mlight_show_lux : %d \n", lux);

	if (!g_als_data->HAL_switch_on) {
		light_turn_onoff(false);
	}
	
	mutex_unlock(&g_alsps_frgb_lock);
	
	return lux;	
}

bool mFRGB_show_switch_onoff(void)
{
	return g_frgb_data->Device_switch_on;
}

int mFRGB_store_switch_onoff(bool bOn)
{
	int data[4] = {-1,-1,-1,-1};
	static int double_open = 0;

	mutex_lock(&g_alsps_frgb_lock);
	log("Front RGB Sensor switch = %d.\n", bOn);
	log("g_frgb_data->HAL_switch_on = %d, camera_HAL_switch_on = %d\n", g_frgb_data->HAL_switch_on, camera_HAL_switch_on);
	if(bOn) {
		if(g_frgb_data->HAL_switch_on && camera_HAL_switch_on) {
			double_open = 1;
			log("FRGB has opened twice\n");
		}
	}

	if((g_frgb_data->Device_switch_on != bOn)) {
		if (bOn == true) {
			/* Turn on FRGB Sensor */
			g_frgb_data->HAL_switch_on = true;
			FRGB_turn_onoff(true);	
			
			/*FRGB sensor polling the raw data. */
			queue_delayed_work(ALSPS_FRGB_delay_workqueue, &FRGB_polling_raw_work, msecs_to_jiffies(FRGB_POLLING_FIRST_RAW));
		} else {
			if(!double_open) {
				/* Turn off FRGB Sensor */
				g_frgb_data->HAL_switch_on = false;
				log("FRGB & Light turn off\n");
				FRGB_turn_onoff(false);
				FRGBsensor_report_raw(data, sizeof(data));
				rgb_first_data_ready = 0;
			} else {
				double_open = 0;
				log("double_open clean\n");
			}
		}			
	} else {
		log("Front RGB Sensor is already %s", bOn?"ON":"OFF");
	}
	mutex_unlock(&g_alsps_frgb_lock);

	return 0;
}

static psensor_ATTR_HAL mpsensor_ATTR_HAL = {
	.proximity_show_switch_onoff = mproximity_show_switch_onoff,
	.proximity_store_switch_onoff = mproximity_store_switch_onoff,
	.proximity_show_status = proximity_check_status,
};

static lsensor_ATTR_HAL mlsensor_ATTR_HAL = {
	.light_show_switch_onoff = mlight_show_switch_onoff,
	.light_store_switch_onoff = mlight_store_switch_onoff,
	.light_show_lux = mlight_show_lux,		
};

static FRGBsensor_ATTR_HAL mFRGBsensor_ATTR_HAL = {
	.FRGB_show_switch_onoff = mFRGB_show_switch_onoff,
	.FRGB_store_switch_onoff = mFRGB_store_switch_onoff,		
};

/************************/
/*Extension Function*/
/***********************/
bool mproximity_show_allreg(void)
{
	if(ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_show_allreg == NULL) {
		err("IRsensor_hw_show_allreg NOT SUPPORT. \n");
		return false;
	}
	ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_show_allreg();
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

int mproximity_show_error_mesg(char *error_mesg)
{
	memcpy(error_mesg, g_error_mesg, strlen(g_error_mesg)+1);
	return 0;
}

int mproximity_show_selection(void)
{
	return g_ps_data->selection;
}

int mproximity_store_selection(int selection)
{
	int ret=0;
	g_ps_data->selection= selection;
	log("Proximity store selection: %d\n", selection);	

	ret = proximity_set_threshold();
	if (ret < 0) {	
		err("proximity_set_threshold ERROR\n");
		return ret;
	}
	
	return 0;
}

/*For power key turn on screen and enable touch*/
int mproximity_show_touch_enable(void)
{
	return touch_enable;
}

int mproximity_store_touch_enable(bool enable)
{
	int ret=0;
	
	if(enable && touch_enable == 0){
//#ifdef ASUS_DISABLE_TOUCH_PORTING_COMPLETED
		//ftxxxx_disable_touch(false);
		//touch_enable = 1;
//#endif
		log("Proximity store enable touch: %d\n", touch_enable);
	} else if(!enable && touch_enable == 1){
//#ifdef ASUS_DISABLE_TOUCH_PORTING_COMPLETED
		//ftxxxx_disable_touch(true);
		//touch_enable = 0;
//#endif
		log("Proximity store disable touch: %d\n", touch_enable);
	}
	
	return ret;
}

/*For load calibration data*/
int mproximity_store_load_calibration_data()
{
	int ret=0;
	
	/*For transition period from 3/5 to 2/4 +++*/
	if(0 == g_ps_data->selection)
		ret = psensor_factory_read_2cm();
	else if(1 == g_ps_data->selection)
		ret = psensor_factory_read_3cm();
	else
		err("INVALID selection : %d\n", g_ps_data->selection);
	/*For transition period from 3/5 to 2/4 ---*/
	
	if(ret > 0) {
		g_ps_data->g_ps_factory_cal_hi = ret;
	    log("Proximity read High Calibration : %d\n", g_ps_data->g_ps_factory_cal_hi);
	}else{
		err("Proximity read DEFAULT High Calibration : %d\n", g_ps_data->g_ps_factory_cal_hi);
	}
	
	/*For transition period from 3/5 to 2/4 +++*/
	if(0 == g_ps_data->selection)
		ret = psensor_factory_read_4cm();
	else if(1 == g_ps_data->selection)
		ret = psensor_factory_read_5cm();
	else
		err("INVALID selection : %d\n", g_ps_data->selection);
	/*For transition period from 3/5 to 2/4 ---*/
	
	if(ret > 0) {
		g_ps_data->g_ps_factory_cal_lo = ret;
		log("Proximity read Low Calibration : %d\n", g_ps_data->g_ps_factory_cal_lo);
	}else{
		err("Proximity read DEFAULT Low Calibration : %d\n", g_ps_data->g_ps_factory_cal_lo);
	}
	
	log("Proximity load factory Calibration done!\n");
	
	return ret;
}

/*For enable anti-oil workaround*/
int mproximity_show_anti_oil_enable(void)
{
	return anti_oil_enable;
}

int mproximity_store_anti_oil_enable(bool enable)
{
	int ret=0;
	
	if(enable){
		anti_oil_enable = 1;
		log("Proximity store enable anti-oil: %d\n", anti_oil_enable);
	} else {
		anti_oil_enable = 0;
		log("Proximity store disable anti-oil: %d\n", anti_oil_enable);
	}
	
	return ret;
}

bool mlight_show_allreg(void)
{
	if(ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_show_allreg == NULL) {
		err("IRsensor_hw_show_allreg NOT SUPPORT. \n");
		return false;
	}
	ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_show_allreg();
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

int mlight_show_error_mesg(char *error_mesg)
{
	memcpy(error_mesg, g_error_mesg, strlen(g_error_mesg)+1);
	return 0;
}

int mlight_show_selection(void)
{
	return g_als_data->selection;
}

int mlight_store_selection(int selection)
{
	bool turnoff_flag=false;
	mutex_lock(&g_alsps_frgb_lock);
	g_als_data->selection= selection;

	if(g_als_data->Device_switch_on){
		light_turn_onoff(false);
		turnoff_flag = true;		
	}	
	if(g_frgb_data->Device_switch_on){
		FRGB_turn_onoff(false);
		turnoff_flag = true;		
	}	
	if(true == turnoff_flag){
		mdelay(LIGHT_TURNON_DELAY_TIME);
	}
	
	
	/*For transition period from 100ms to 50ms*/
	if(0 == selection){		
		ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_set_integration(0);
	}else if(1 == selection){
		ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_set_integration(1);
	}else{
		err("INVALID selection : %d\n", selection);
	}

	if(g_als_data->HAL_switch_on ){
		light_turn_onoff(true);
	}
	if(g_frgb_data->HAL_switch_on ){
		FRGB_turn_onoff(true);
	}
	
	//log("Light Sensor store selection: %d\n ", selection);	
	light_get_accuracy_gain();

	mutex_unlock(&g_alsps_frgb_lock);
	return 0;
}

bool mFRGB_show_allreg(void)
{
	if(ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_show_allreg == NULL) {
		err("IRsensor_hw_show_allreg NOT SUPPORT. \n");
		return false;
	}
	ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_show_allreg();
	return true;
}

int mFRGB_show_log_threshold(void)
{
	return g_frgb_data->g_frgb_log_threshold;
}

int mFRGB_store_log_threshold(int log_threshold)
{
	g_frgb_data->g_frgb_log_threshold = log_threshold;
	log("FRGB Sensor store Log Threshold: %d\n", log_threshold);	
	
	return 0;
}

int mFRGB_show_error_mesg(char *error_mesg)
{
	memcpy(error_mesg, g_error_mesg, strlen(g_error_mesg)+1);
	return 0;
}

static psensor_ATTR_Extension mpsensor_ATTR_Extension = {
	.proximity_show_allreg = mproximity_show_allreg,
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
	.proximity_show_error_mesg = mproximity_show_error_mesg,
	.proximity_show_selection = mproximity_show_selection,
	.proximity_store_selection = mproximity_store_selection,
	/*For power key turn on screen and enable touch*/
	.proximity_show_touch_enable = mproximity_show_touch_enable,
	.proximity_store_touch_enable = mproximity_store_touch_enable,
	/*For load calibration data*/
	.proximity_store_load_calibration_data = mproximity_store_load_calibration_data,
	/*For enable anti-oil workaround*/
	.proximity_show_anti_oil_enable = mproximity_show_anti_oil_enable,
	.proximity_store_anti_oil_enable = mproximity_store_anti_oil_enable,
};

static lsensor_ATTR_Extension mlsensor_ATTR_Extension = {
	.light_show_allreg = mlight_show_allreg,
	.light_show_sensitivity = mlight_show_sensitivity,
	.light_store_sensitivity = mlight_store_sensitivity,
	.light_show_log_threshold = mlight_show_log_threshold,
	.light_store_log_threshold = mlight_store_log_threshold,
	.light_show_int_count = mlight_show_int_count,
	.light_show_event_count = mlight_show_event_count,
	.light_show_error_mesg = mlight_show_error_mesg,
	.light_show_selection = mlight_show_selection,
	.light_store_selection = mlight_store_selection
};

static FRGBsensor_ATTR_Extension mFRGBsensor_ATTR_Extension = {
	.FRGB_show_allreg = mFRGB_show_allreg,
	.FRGB_show_log_threshold = mFRGB_show_log_threshold,
	.FRGB_store_log_threshold = mFRGB_store_log_threshold,
	.FRGB_show_error_mesg = mFRGB_show_error_mesg,
};

static psensor_ATTR mpsensor_ATTR = {
	.info_type = &mpsensor_info_type,
	.ATTR_Calibration = &mpsensor_ATTR_Calibration,
	.ATTR_BMMI = &mpsensor_ATTR_BMMI,
	.ATTR_Hardware = &mpsensor_ATTR_Hardware,
	.ATTR_HAL = &mpsensor_ATTR_HAL,
	.ATTR_Extension = &mpsensor_ATTR_Extension,
};

static lsensor_ATTR mlsensor_ATTR = {
	.info_type = &mlsensor_info_type,
	.ATTR_Calibration = &mlsensor_ATTR_Calibration,
	.ATTR_BMMI = &mlsensor_ATTR_BMMI,
	.ATTR_Hardware = &mlsensor_ATTR_Hardware,
	.ATTR_HAL = &mlsensor_ATTR_HAL,
	.ATTR_Extension = &mlsensor_ATTR_Extension,
};

static FRGBsensor_ATTR mFRGBsensor_ATTR = {
	.info_type = &mFRGBsensor_info_type,
	.ATTR_Calibration = &mFRGBsensor_ATTR_Calibration,
	.ATTR_BMMI = &mFRGBsensor_ATTR_BMMI,
	.ATTR_Hardware = &mFRGBsensor_ATTR_Hardware,
	.ATTR_HAL = &mFRGBsensor_ATTR_HAL,
	.ATTR_Extension = &mFRGBsensor_ATTR_Extension,
};

extern void asus_lcd_early_on_for_phone_call(void);
/*==========================
 *|| Interrupt Service Routine Part ||
 *===========================
 */
static void proximity_work(int state)
{
	int adc = 0;
	//int audio_mode = 0;

	/* Get Proximity adc value */
	adc= ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_get_adc();
	if(adc < 0){
		err("[ISR] Proximity get adc ERROR\n");	
		return;
	}

	/* Ignore the interrupt when Switch off */
	if(g_ps_data->HAL_switch_on == true)
	{
		/* Check proximity close or away. */
		if(ALSPS_INT_PS_AWAY == state) {
			log("[ISR] Proximity Detect Object Away. (adc = %d)\n", adc);
			asus_lcd_early_on_for_phone_call();
			psensor_report_abs(PSENSOR_REPORT_PS_AWAY);
			g_ps_data->event_counter++;	/* --- For stress test debug --- */
			//audio_mode = get_audiomode();
			//if (0 == audio_mode || 2 == audio_mode || 3 == audio_mode) {
			//	ftxxxx_disable_touch(false);
			//}
//#ifdef ASUS_DISABLE_TOUCH_PORTING_COMPLETED
			//ftxxxx_disable_touch(false);
			//touch_enable = 1;
//#endif
		} else if (ALSPS_INT_PS_CLOSE == state) {
			if(pocket_mode_threshold > 0 && adc > pocket_mode_threshold){
				log("[ISR] Proximity Detect Object Close. (adc = %d, distance < 1cm)\n", adc);		
				psensor_report_abs(PSENSOR_REPORT_PS_POCKET);
			}else{
				log("[ISR] Proximity Detect Object Close. (adc = %d)\n", adc);		
				psensor_report_abs(PSENSOR_REPORT_PS_CLOSE);
			}
			g_ps_data->event_counter++;	/* --- For stress test debug --- */
//#ifdef ASUS_AUDIO_MODE_PORTING_COMPLETED
			//audio_mode = get_audiomode();
//#endif
			//if (2 == audio_mode || 3 == audio_mode) {
//#ifdef ASUS_DISABLE_TOUCH_PORTING_COMPLETED
				//ftxxxx_disable_touch(true);
				//touch_enable = 0;
//#endif
			//}
		} else {
			err("[ISR] Proximity Detect Object ERROR. (adc = %d)\n", adc);
		}
	}
	
}

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
		adc = ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_get_adc();

		dbg("[ISR] Light Sensor Get adc : %d\n", adc);
		if(adc < 0){
			err("light_hw_get_adc ERROR\n");
			return;
		}

		/* Set the default sensitivity (3rd priority)*/
		if(adc >= g_als_data->g_als_calvalue) {
			light_change_sensitivity = LIGHT_CHANGE_LOW_SENSITIVITY;
		}else {
			light_change_sensitivity = LIGHT_CHANGE_HI_SENSITIVITY;
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
		if (high_threshold > ALSPS_FRGB_hw_client->mlsensor_hw->light_max_threshold)	
			high_threshold = ALSPS_FRGB_hw_client->mlsensor_hw->light_max_threshold;	
		
		ret = ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_set_hi_threshold(high_threshold);
		if(ret < 0) {
			err("[ISR] Light Sensor Set High Threshold ERROR. (High:%d)\n", high_threshold);
		}
		dbg("[ISR] Light Sensor Set High Threshold. (High:%d)\n", high_threshold);	

		ret = ALSPS_FRGB_hw_client->mlsensor_hw->light_hw_set_lo_threshold(low_threshold);
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
		
		if(g_als_data->g_als_log_first_event){
			log("[ISR] Light Sensor First Report lux : %d (adc = %d)\n", lux, adc);
			g_als_data->g_als_log_first_event = false;
		}
		if(abs(g_als_last_lux - lux) > light_log_threshold)
			log("[ISR] Light Sensor Report lux : %d (adc = %d)\n", lux, adc);

		lsensor_report_lux(lux);

		g_als_data->event_counter++;	/* --- For stress test debug --- */
		g_als_last_lux = lux;
	}
}

static void ALSPS_FRGB_ist(struct work_struct *work)
{
	int alsps_int_ps, alsps_int_als;
	
mutex_lock(&g_alsps_frgb_lock);
	if(g_als_data->Device_switch_on == false && g_ps_data->Device_switch_on == false) {
		err("ALSPS are disabled and ignore IST.\n");
		goto ist_err;
	}
	dbg("ALSPS FRGB ist +++ \n");
	if(ALSPS_FRGB_hw_client == NULL)	{
		dbg("ALSPS_FRGB_hw_client is NULL \n");
		goto ist_err;
	}

	/* Wait the i2c driver resume finish */
	if(g_alsps_frgb_power_status == ALSPS_SUSPEND){
		mdelay(WAIT_I2C_DELAY);
		dbg("Wait i2c driver ready(%d ms)", WAIT_I2C_DELAY);
	}

	/* Read INT_FLAG will clean the interrupt */
	ALSPS_FRGB_SENSOR_INT = ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_get_interrupt();
	if(ALSPS_FRGB_SENSOR_INT <0){
//		err("ALSPS_FRGB_hw_get_interrupt ERROR\n");
		goto ist_err;
	}

	/* Check Proximity Interrupt */
	alsps_int_ps = ALSPS_FRGB_SENSOR_INT&ALSPS_INT_PS_MASK;
	if(alsps_int_ps == ALSPS_INT_PS_CLOSE || alsps_int_ps == ALSPS_INT_PS_AWAY) 
	{
		dbg("Proximity ist \n");
		if(g_ps_data->HAL_switch_on == true)
			g_ps_data->int_counter++;	/* --- For stress test debug --- */
		
		if (alsps_int_ps == ALSPS_INT_PS_AWAY) {
			proximity_work(ALSPS_INT_PS_AWAY);
		}
		if (alsps_int_ps == ALSPS_INT_PS_CLOSE) {
			proximity_work(ALSPS_INT_PS_CLOSE);
		}
	}

	/* Check Light Sensor Interrupt */
	alsps_int_als = ALSPS_FRGB_SENSOR_INT&ALSPS_INT_ALS_MASK;
	if (alsps_int_als == ALSPS_INT_ALS) {
		dbg("Light Sensor ist \n");
		if(g_als_data->HAL_switch_on == true)
			g_als_data->int_counter++;	/* --- For stress test debug --- */
		
		light_work();
	}
	dbg("ALSPS FRGB ist --- \n");
ist_err:	
	wake_unlock(&g_alsps_frgb_wake_lock);
	dbg("[IRQ] Enable irq !! \n");
	enable_irq(ALSPS_FRGB_SENSOR_IRQ);	
mutex_unlock(&g_alsps_frgb_lock);

}

void ALSPSsensor_irq_handler(void)
{
	dbg("[IRQ] Disable irq !! \n");
	disable_irq_nosync(ALSPS_FRGB_SENSOR_IRQ);
	
	if(ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_get_interrupt == NULL) {
		err("ALSPS_FRGB_hw_get_interrupt NOT SUPPORT. \n");
		goto irq_err;
	}

	/*Queue work will enbale IRQ and unlock wake_lock*/
	queue_work(ALSPS_FRGB_workqueue, &ALSPS_FRGB_ist_work);
	wake_lock(&g_alsps_frgb_wake_lock);
	return;
irq_err:
	dbg("[IRQ] Enable irq !! \n");
	enable_irq(ALSPS_FRGB_SENSOR_IRQ);
}

static ALSPSsensor_GPIO mALSPSsensor_GPIO = {
	.ALSPSsensor_isr = ALSPSsensor_irq_handler,
};


/*=======================
 *|| For Proximity check status ||
 *========================
 */
bool proximity_check_status(void)
{	
	int adc_value = 0;
	bool status = false;
	int ret=0;
	int threshold_high = 0;

wake_lock(&g_alsps_frgb_wake_lock);	
	/* check probe status */
	if(ALSPS_FRGB_hw_client == NULL)
		goto ERROR_HANDLE;

	mutex_lock(&g_alsps_frgb_lock);

	if(g_ps_data->Device_switch_on == false){
#if 0
		/*Set Proximity Threshold(reset to factory)*/
		ret = proximity_set_threshold();
		if (ret < 0) {	
			err("proximity_set_threshold ERROR\n");
			goto ERROR_HANDLE;
		}
#endif
		ret = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);	
		if(ret < 0){
			err("proximity_hw_turn_onoff(true) ERROR\n");
			goto ERROR_HANDLE;
		}			
	}
	
	msleep(PROXIMITY_TURNON_DELAY_TIME);

	adc_value = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_get_adc();
	threshold_high =  (g_ps_data->g_ps_factory_cal_hi + g_ps_data->g_ps_autok_max);

	if (adc_value >= threshold_high) {
		status = true;
	}else{ 
		status = false;
	}
	log("proximity_check_status : %s , (adc, hi_cal + AutoK_MAX)=(%d, %d)\n", 
		status?"Close":"Away", adc_value, threshold_high);
	
	if(g_ps_data->Device_switch_on == false){
		ret = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);	
		if(ret < 0){
			err("proximity_hw_turn_onoff(false) ERROR\n");
		}			
	}
	
ERROR_HANDLE:
	
	mutex_unlock(&g_alsps_frgb_lock);
wake_unlock(&g_alsps_frgb_wake_lock);
	return status;
}

EXPORT_SYMBOL(proximity_check_status);

/*===========================
 *|| Proximity Auto Calibration Part ||
 *============================
 */
static int proximity_check_minCT(void)
{
	int adc_value = 0;
	int crosstalk_diff;
	int crosstalk_min = 9999;
	int ret;
	int round;
	
	/*check the crosstalk calibration value*/	
	ret = psensor_factory_read_inf();	
	if(ret > 0) {
	    	g_ps_data->g_ps_calvalue_inf= ret;
		log("Proximity read INF Calibration : %d\n", g_ps_data->g_ps_calvalue_inf);
	}else{
		err("Proximity read DEFAULT INF Calibration : %d\n", g_ps_data->g_ps_calvalue_inf);
	}

	ret = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);	
	if(ret < 0){
		err("proximity_hw_turn_onoff(true) ERROR\n");
		return ret;
	}
	/*update the min crosstalk value*/
	for(round=0; round<PROXIMITY_AUTOK_COUNT; round++){	
		mdelay(PROXIMITY_AUTOK_DELAY);
		adc_value = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_get_adc();
		log("proximity auto calibration adc : %d\n", adc_value);
		if(adc_value < crosstalk_min ){
			crosstalk_min = adc_value;
			log("Update the min for crosstalk : %d\n", crosstalk_min);
		}
	}
	ret = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);	
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

		if(ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_set_autoK == NULL) {
			err("proximity_hw_set_autoK NOT SUPPORT. \n");
			return -1;
		}
		ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_set_autoK(crosstalk_diff);
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

	/* proximity sensor go to suspend, cancel autok timer */
	if(g_alsps_frgb_power_status == ALSPS_SUSPEND){
		hrtimer_cancel(&g_alsps_frgb_timer);
		log("proximity has suspended, cancel proximity autoK polling!!");
		return;
	}

	if(ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_set_autoK == NULL) {
		err("proximity_hw_set_autoK NOT SUPPORT. \n");
		return;
	}
	
	adc_value = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_get_adc();
	if(adc_value < 0){
		return;
	} else {
		dbg("auto calibration polling : %d\n", adc_value);
	}

	crosstalk_diff = adc_value - g_ps_data->g_ps_calvalue_inf;

	if((crosstalk_diff<g_ps_data->crosstalk_diff) &&( g_ps_data->crosstalk_diff!=0)){
		/*last diff of crosstalk does not set to HW, should reset the value to 0.*/
		if(g_ps_data->crosstalk_diff >= g_ps_data->g_ps_autok_max){
			g_ps_data->crosstalk_diff = 0;
		}
		
		if(crosstalk_diff<=g_ps_data->g_ps_autok_min){			
			ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_set_autoK(0-g_ps_data->crosstalk_diff);
			g_ps_data->crosstalk_diff = 0;
			log("Update the diff for crosstalk : %d\n", g_ps_data->crosstalk_diff);
		}else if((crosstalk_diff>g_ps_data->g_ps_autok_min) && (crosstalk_diff<g_ps_data->g_ps_autok_max)){			
			ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hw_set_autoK(crosstalk_diff-g_ps_data->crosstalk_diff);
			g_ps_data->crosstalk_diff = crosstalk_diff;
			log("Update the diff for crosstalk : %d\n", crosstalk_diff);
		}else{
			dbg("over the autok_max : (adc, inf) = %d(%d, %d) > %d\n", 
				crosstalk_diff, adc_value, g_ps_data->g_ps_calvalue_inf, g_ps_data->g_ps_autok_max);
			g_ps_data->crosstalk_diff = crosstalk_diff;
		}
	}	
}

static enum hrtimer_restart proximity_timer_function(struct hrtimer *timer)
{
	ktime_t autok_delay;
	
	dbg("proximity_timer_function\n");
	queue_work(ALSPS_FRGB_workqueue, &proximity_autok_work);	

	if(0 == g_ps_data->crosstalk_diff){
		return HRTIMER_NORESTART;
	}else{
		/*needs to be reset in the callback function*/
		autok_delay = ns_to_ktime(PROXIMITY_AUTOK_POLLING * NSEC_PER_MSEC);
		hrtimer_forward_now(&g_alsps_frgb_timer, autok_delay);
	}
	return HRTIMER_RESTART;
}

/*====================
 *|| IO Control Part ||
 *====================
 */
#if ENABLE_PROXIMITY_IOCTL_LIB
static int proxSensor_miscOpen(struct inode *inode, struct file *file)
{
	int ret = 0;
	if(prox_open_count == 0){
		ret = mproximity_store_switch_onoff(true);
		dbg("%s: %d\n", __func__, ret);
		if (ret < 0) {
			err("proximity_hw_turn_onoff(true) ERROR\n");
		} else {
			prox_open_count++;
		}
	} else if(prox_open_count > 0) {
		prox_open_count++;
		log("proximity sensor has been opened(count=%d)\n", prox_open_count);
	}
	return ret;
}

static int proxSensor_miscRelease(struct inode *inode, struct file *file)
{
	int ret = 0;
	prox_open_count--;
	if(prox_open_count == 0){
		ret = mproximity_store_switch_onoff(false);
		dbg("%s: %d\n", __func__, ret);
		if (ret < 0) {
			err("proximity_hw_turn_onoff(false) ERROR\n");
			prox_open_count++;
		}
	} else if(prox_open_count < 0){
		prox_open_count = 0;
		log("proximity sensor has been closed, do nothing(count=%d)\n", prox_open_count);
	}
	return ret;
}

static struct file_operations prox_fops = {
  .owner = THIS_MODULE,
  .open = proxSensor_miscOpen,
  .release = proxSensor_miscRelease
};

struct miscdevice prox_misc = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = "asusProxSensor",
  .fops = &prox_fops
};

int proxSensor_miscRegister(void)
{
	int rtn = 0;
	rtn = misc_register(&prox_misc);
	if (rtn < 0) {
		log("[%s] Unable to register prox misc deive\n", __func__);
		misc_deregister(&prox_misc);
	}
	return rtn;
}
#endif

#if ENABLE_LIGHT_IOCTL_LIB
static int lightSensor_miscOpen(struct inode *inode, struct file *file)
{
	int ret = 0;
	if(light_open_count == 0){
		ret = mlight_store_switch_onoff(true);
		dbg("%s: %d\n", __func__, ret);
		if (ret < 0) {
			err("light_hw_turn_onoff(true) ERROR\n");
		} else {
			light_open_count++;
		}
	} else if(light_open_count > 0) {
		light_open_count++;
		log("light sensor has been opened(count=%d)\n", light_open_count);
	}
	return ret;
}

static int lightSensor_miscRelease(struct inode *inode, struct file *file)
{
	int ret = 0;
	light_open_count--;
	if(light_open_count == 0){
		ret = mlight_store_switch_onoff(false);
		dbg("%s: %d\n", __func__, ret);
		if (ret < 0) {
			err("light_hw_turn_onoff(false) ERROR\n");
			light_open_count++;
		}
	} else if(light_open_count < 0){
		light_open_count = 0;
		log("light sensor has been closed, do nothing(count=%d)\n", light_open_count);
	}
	return ret;
}

static struct file_operations light_fops = {
  .owner = THIS_MODULE,
  .open = lightSensor_miscOpen,
  .release = lightSensor_miscRelease
};

struct miscdevice light_misc = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = "asusLightSensor",
  .fops = &light_fops
};

int lightSensor_miscRegister(void)
{
	int rtn = 0;
	rtn = misc_register(&light_misc);
	if (rtn < 0) {
		log("[%s] Unable to register light misc deive\n", __func__);
		misc_deregister(&light_misc);
	}
	return rtn;
}
#endif

static int frgbSensor_miscOpen(struct inode *inode, struct file *file)
{
	int ret = 0;
	camera_HAL_switch_on = true;
	ret = mFRGB_store_switch_onoff(true);
	dbg("%s: %d\n", __func__, ret);
	if (ret < 0) {
		camera_HAL_switch_on = false;
		mFRGB_store_switch_onoff(false);
	}		
	return ret;
}
static int frgbSensor_miscRelease(struct inode *inode, struct file *file)
{
	int ret = 0;
	ret = mFRGB_store_switch_onoff(false);
	camera_HAL_switch_on = false;
	dbg("%s: %d\n", __func__, ret);
	return ret;
}
static long frgbSensor_miscIoctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int dataI[ASUS_RGB_SENSOR_DATA_SIZE];
	char dataS[ASUS_RGB_SENSOR_NAME_SIZE];

	switch (cmd) {
		case ASUS_RGB_SENSOR_IOCTL_DATA_READ:
			dataI[0] = g_red_last_raw;
			dataI[1] = g_green_last_raw;
			dataI[2] = g_blue_last_raw;
			dataI[3] = g_ir_last_raw;
			dataI[4] = 1;

			dbg("%s: cmd = DATA_READ, data[0] = %d, data[1] = %d, data[2] = %d, data[3] = %d, data[4] = %d\n"
				, __func__, dataI[0], dataI[1], dataI[2], dataI[3], dataI[4]);
			ret = copy_to_user((int __user*)arg, &dataI, sizeof(dataI));
			break;
		case ASUS_RGB_SENSOR_IOCTL_MODULE_NAME:
			snprintf(dataS, sizeof(dataS), "%s", CM36656_I2C_NAME);
			log("%s: cmd = MODULE_NAME, name = %s\n", __func__, dataS);
			ret = copy_to_user((int __user*)arg, &dataS, sizeof(dataS));
			break;
		default:
			ret = -1;
			log("%s: default\n", __func__);
	}

	return ret;
}

static struct file_operations frgb_fops = {
  .owner = THIS_MODULE,
  .open = frgbSensor_miscOpen,
  .release = frgbSensor_miscRelease,
  .unlocked_ioctl = frgbSensor_miscIoctl,
  .compat_ioctl = frgbSensor_miscIoctl
};

struct miscdevice frgb_misc = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = "asusFRGBSensor",
  .fops = &frgb_fops
};

int frgbSensor_miscRegister(void)
{
	int rtn = 0;
	rtn = misc_register(&frgb_misc);
	if (rtn < 0) {
		log("[%s] Unable to register misc deive\n", __func__);
		misc_deregister(&frgb_misc);
	}
	return rtn;
}

/*====================
 *|| Initialization Part ||
 *====================
 */
static int init_data(void)
{
	int ret = 0;	
	/* Reset ASUS_proximity_sensor_data */
	g_ps_data = kmalloc(sizeof(struct psensor_data), GFP_KERNEL);
	if (!g_ps_data) {
		err("g_ps_data kmalloc ERROR\n");
		ret = -ENOMEM;
		goto init_data_err;
	}
	memset(g_ps_data, 0, sizeof(struct psensor_data));
	g_ps_data->Device_switch_on = 	false;
	g_ps_data->HAL_switch_on = 	false;	
	g_ps_data->polling_mode = 		false;
	g_ps_data->autok = 			true;
	
	g_ps_data->g_ps_calvalue_hi = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hi_threshold_default;
	g_ps_data->g_ps_calvalue_lo = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_low_threshold_default;	
	g_ps_data->g_ps_calvalue_inf = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_crosstalk_default;	
	g_ps_data->g_ps_factory_cal_hi = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_hi_threshold_default;
	g_ps_data->g_ps_factory_cal_lo = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_low_threshold_default;
	g_ps_data->g_ps_autok_min= ALSPS_FRGB_hw_client->mpsensor_hw->proximity_autok_min;	
	g_ps_data->g_ps_autok_max = ALSPS_FRGB_hw_client->mpsensor_hw->proximity_autok_max;	

	g_ps_data->int_counter = 0;
	g_ps_data->event_counter = 0;
	g_ps_data->crosstalk_diff = 0;
	g_ps_data->selection = 0;

	/* Reset ASUS_light_sensor_data */
	g_als_data = kmalloc(sizeof(struct lsensor_data), GFP_KERNEL);
	if (!g_als_data)	{
		err("g_als_data kmalloc ERROR\n");
		ret = -ENOMEM;
		goto init_data_err;
	}
	memset(g_als_data, 0, sizeof(struct lsensor_data));
	g_als_data->Device_switch_on = false;
	g_als_data->HAL_switch_on = 	false;	
	g_als_data->g_als_calvalue = ALSPS_FRGB_hw_client->mlsensor_hw->light_calibration_default;
	g_als_data->g_als_accuracy_gain = ALSPS_DEFAULT_VALUE;
	g_als_data->g_als_change_sensitivity = 	ALSPS_DEFAULT_VALUE;
	g_als_data->g_als_log_threshold = 		ALSPS_DEFAULT_VALUE;
	g_als_data->g_als_log_first_event = true;

	g_als_data->int_counter = 0;
	g_als_data->event_counter = 0;
	g_als_data->selection = 0;

	/* Reset ASUS_FRGB_sensor_data */
	g_frgb_data = kmalloc(sizeof(struct frgb_data), GFP_KERNEL);
	if (!g_frgb_data)	{
		err("g_frgb_data kmalloc ERROR\n");
		ret = -ENOMEM;
		goto init_data_err;
	}
	memset(g_frgb_data, 0, sizeof(struct frgb_data));
	g_frgb_data->Device_switch_on = 	false;
	g_frgb_data->HAL_switch_on = 	false;	
	g_frgb_data->g_frgb_log_threshold = ALSPS_DEFAULT_VALUE;
	
	g_frgb_data->event_counter = 0;
	
	/* Reset ASUS P/L/FRGB sensor power status */
	g_alsps_frgb_power_status = ALSPS_RESUME;
	
	/* Reset ASUS FRGB sensor polling cancel flag */
	g_frgb_polling_cancel_flag = false;

	return 0;
init_data_err:
	err("Init Data ERROR\n");
	return ret;
}
 
void mALSPS_FRGB_algo_probe(struct i2c_client *client)
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
	i2c_set_clientdata(client, g_ps_data);	

	/* i2c client */
	g_i2c_client = client;
	if (ALSPS_FRGB_SENSOR_IRQ < 0)		
		goto probe_err;	

	/* I2c stress test */
#ifdef CONFIG_I2C_STRESS_TEST	
	i2c_add_test_case(client, "ALSPS FRGB Test", ARRAY_AND_SIZE(ALSPS_FRGB_TestCaseInfo));	
#endif

	log("Driver PROBE ---\n");
	return ;
probe_err:
	err("Driver PROBE ERROR ---\n");
	return;

}

void mALSPS_FRGB_algo_remove(void)
{
	log("Driver REMOVE +++\n");

	ALSPSsensor_gpio_unregister(ALSPS_FRGB_SENSOR_IRQ);

	log("Driver REMOVE ---\n");
	
	return;
}

void mALSPS_FRGB_algo_shutdown(void)
{
	log("Driver SHUTDOWN +++\n");

	/* Disable sensor */
	if (g_als_data->Device_switch_on)
		light_turn_onoff(false);
	if (g_ps_data->Device_switch_on)
		proximity_turn_onoff(false);	
	if (g_frgb_data->Device_switch_on)
		FRGB_turn_onoff(false);	
	
	log("Driver SHUTDOWN ---\n");
	
	return;
}

void mALSPS_FRGB_algo_suspend(void)
{
	log("Driver SUSPEND +++\n");

	g_alsps_frgb_power_status = ALSPS_SUSPEND;

	/* For keep Proximity can wake_up system */
	if (g_ps_data->Device_switch_on)
		enable_irq_wake(ALSPS_FRGB_SENSOR_IRQ);

	/* For make sure Light sensor mush be switch off when system suspend */
	if (g_als_data->Device_switch_on)				
		light_suspend_turn_off(false);
	
	/* For make sure FRGB sensor mush be switch off when system suspend */
	if (g_frgb_data->Device_switch_on)				
		FRGB_suspend_turn_off(false);
	
	log("Driver SUSPEND ---\n");
	
	return;
}

void mALSPS_FRGB_algo_resume(void)
{
	ktime_t autok_delay;

	//log("Driver RESUME +++\n");

	g_alsps_frgb_power_status = ALSPS_RESUME;

	if (g_als_data->Device_switch_on == 0 && g_als_data->HAL_switch_on == 1){
		resume_flag=1;
		light_turn_onoff(1);
	}

	if (g_frgb_data->Device_switch_on == 0 && g_frgb_data->HAL_switch_on >= 1){
		resume_flag=1;
		FRGB_turn_onoff(1);
		/* If the frgb sensor has been opened and frgb polling thread has been canceled, 
		 * restart it. */
		if(g_frgb_polling_cancel_flag){
			queue_delayed_work(ALSPS_FRGB_delay_workqueue, &FRGB_polling_raw_work, msecs_to_jiffies(FRGB_POLLING_TIME));
			g_frgb_polling_cancel_flag = false;
		}
	}
	
	/* If the proximity sensor has been opened and autok has been enabled, 
	 * start autoK work queue. */
	if (g_ps_data->Device_switch_on) {
		if(true == g_ps_data->autok && g_ps_data->crosstalk_diff != 0){
			autok_delay = ns_to_ktime(PROXIMITY_AUTOK_POLLING * NSEC_PER_MSEC);
			hrtimer_start(&g_alsps_frgb_timer, autok_delay, HRTIMER_MODE_REL);
		}
	}
	
	//log("Driver RESUME ---\n");
	
	return;
}

static ALSPS_FRGB_I2C mALSPS_FRGB_I2C = {
	.ALSPS_FRGB_probe = mALSPS_FRGB_algo_probe,
	.ALSPS_FRGB_remove = mALSPS_FRGB_algo_remove,
	.ALSPS_FRGB_shutdown = mALSPS_FRGB_algo_shutdown,
	.ALSPS_FRGB_suspend = mALSPS_FRGB_algo_suspend,
	.ALSPS_FRGB_resume = mALSPS_FRGB_algo_resume,
};

static int __init ALSPS_FRGB_init(void)
{
	int ret = 0;
	int data[4] = {-1,-1,-1,-1};
	
	log("Driver INIT +++\n");

	/*Record the error message*/
	g_error_mesg = kzalloc(sizeof(char [ERROR_MESG_SIZE]), GFP_KERNEL);
	
	/* Work Queue */
	ALSPS_FRGB_workqueue = create_singlethread_workqueue(SENSOR_TYPE_NAME"_wq");	
	ALSPS_FRGB_delay_workqueue = create_singlethread_workqueue(SENSOR_TYPE_NAME"_delay_wq");	

	/* Initialize the Mutex */
	mutex_init(&g_alsps_frgb_lock);
	
	/* Initialize the wake lock */
	wake_lock_init(&g_alsps_frgb_wake_lock, WAKE_LOCK_SUSPEND, "ALSPS_FRGB_wake_lock");

	/*Initialize high resolution timer*/
	hrtimer_init(&g_alsps_frgb_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_alsps_frgb_timer.function = proximity_timer_function;

	/* i2c Registration for probe/suspend/resume */				
	ret = ALSPS_FRGB_i2c_register(&mALSPS_FRGB_I2C);
	if (ret < 0)
		goto init_err;
		
	/* Hardware Register Initialization */
	ALSPS_FRGB_hw_client = ALSPS_FRGB_hw_getHardware();
	if(ALSPS_FRGB_hw_client == NULL)
		goto init_err;

	/* driver data structure initialize */
	ret = init_data();
	if (ret < 0)
		goto init_err;

	/* string copy the character of vendor and module number */
	strcpy(mpsensor_ATTR.info_type->vendor, ALSPS_FRGB_hw_client->vendor);
	strcpy(mpsensor_ATTR.info_type->module_number, ALSPS_FRGB_hw_client->module_number);
	strcpy(mlsensor_ATTR.info_type->vendor, ALSPS_FRGB_hw_client->vendor);
	strcpy(mlsensor_ATTR.info_type->module_number, ALSPS_FRGB_hw_client->module_number);
	strcpy(mFRGBsensor_ATTR.info_type->vendor, ALSPS_FRGB_hw_client->vendor);
	strcpy(mFRGBsensor_ATTR.info_type->module_number, ALSPS_FRGB_hw_client->module_number);
	
	/* Attribute */
	ret = psensor_ATTR_register(&mpsensor_ATTR);	
	if (ret < 0)
		goto init_err;
	ret = lsensor_ATTR_register(&mlsensor_ATTR);	
	if (ret < 0)
		goto init_err;
	ret = FRGBsensor_ATTR_register(&mFRGBsensor_ATTR);	
	if (ret < 0)
		goto init_err;
	
	/* Input Device */
#if ENABLE_PROXIMITY_IOCTL_LIB
	ret = proxSensor_miscRegister();
	if (ret < 0)
		goto init_err;
#endif
	ret = psensor_report_register();
	if (ret < 0)
		goto init_err;
#if ENABLE_LIGHT_IOCTL_LIB
	ret = lightSensor_miscRegister();
	if (ret < 0)
		goto init_err;
#endif
	ret = lsensor_report_register();
	if (ret < 0)
		goto init_err;	
	ret = frgbSensor_miscRegister();
	if (ret < 0)
		goto init_err;	
//	ret = FRGBsensor_report_register();
//	if (ret < 0)
//		goto init_err;	

	ALSPS_FRGB_SENSOR_IRQ = ALSPSsensor_gpio_register(g_i2c_client, &mALSPSsensor_GPIO);
	if (ALSPS_FRGB_SENSOR_IRQ < 0)
		goto init_err;	
		
	/* Enable LDO 3V */
	ret = ALSPSsensor_enabe_gpio_register(g_i2c_client);
	if (ret < 0)
		goto init_err;

	/*To avoid LUX can NOT report when reboot in LUX=0*/
	lsensor_report_lux(-1);
	psensor_report_abs(-1);
	g_red_last_raw = -1;
	g_green_last_raw = -1;
	g_blue_last_raw = -1;
	g_ir_last_raw = -1;
	FRGBsensor_report_raw(data, sizeof(data));
	
	log("Driver INIT ---\n");
	return 0;

init_err:
	err("Driver INIT ERROR ---\n");
	return ret;
}

static void __exit ALSPS_FRGB_exit(void)
{
	log("Driver EXIT +++\n");

	/* i2c Unregistration */	
	ALSPS_FRGB_i2c_unregister();

	/*Report Unregistration*/
	psensor_report_unregister();
	lsensor_report_unregister();
//	FRGBsensor_report_unregister();

	/*ATTR Unregistration*/
	psensor_ATTR_unregister();
	lsensor_ATTR_unregister();
	FRGBsensor_ATTR_unregister();
	
	wake_lock_destroy(&g_alsps_frgb_wake_lock);
	mutex_destroy(&g_alsps_frgb_lock);
	kfree(g_ps_data);
	kfree(g_als_data);
	kfree(g_frgb_data);

	destroy_workqueue(ALSPS_FRGB_workqueue);
	destroy_workqueue(ALSPS_FRGB_delay_workqueue);
	
	log("Driver EXIT ---\n");
}

module_init(ALSPS_FRGB_init);
module_exit(ALSPS_FRGB_exit);

MODULE_AUTHOR("sr_Huang <sr_Huang@asus.com>");
MODULE_DESCRIPTION("Proximity and Ambient Light Sensor with Front RGB");
MODULE_LICENSE("GPL");

