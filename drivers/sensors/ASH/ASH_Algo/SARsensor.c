/* 
 * Copyright (C) 2019 ASUSTek Inc.
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

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_ALGO"
#define SENSOR_TYPE_NAME	"SAR"
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
/* SAR Sensor Global Variables */
/*****************************/
static int ASUS_SAR_SENSOR_IRQ;
//static int ASUS_SAR_SENSOR_INT_STATUS;
static struct ASUS_sar_sensor_data			*g_sar_sensor_data;
static struct SAR_sensor_hw				*sar_sensor_hw_client;
static struct workqueue_struct				*SAR_sensor_workqueue;
static struct mutex 							g_sar_sensor_lock;
static struct wake_lock 						g_sar_sensor_wake_lock;
static struct i2c_client						*g_i2c_client;
static char *g_error_mesg;

/***********************/
/* SAR Sensor Functions*/
/**********************/
/*Device Layer Part*/
static int 	SAR_sensor_turn_onoff(bool bOn);

/*Interrupt Service Routine Part*/
static void SAR_sensor_ist(struct work_struct *work);

/*Initialization Part*/
static int init_data(void);

/*Work Queue*/
static DECLARE_WORK(SAR_sensor_ist_work, SAR_sensor_ist);

/********************/
/* SAR sensor data structure */
/********************/

struct ASUS_sar_sensor_data 
{
	bool Device_switch_on;					/* this var. means is turning on sensor or not */
	int Device_Interrupt_detect_status;		/* this var. means is for member detect status */
	bool Device_detected_status[4];			/* this var. means is for which channels have detected near */
	
	bool HAL_switch_on;						/* this var. means if HAL is turning on sar or not */
};

/**********************************/
/* SAR sensor Info Type*/
/*********************************/
static SAR_sensor_info_type mSAR_sensor_info_type = {{0}};

/*=======================
 *|| I2c Stress Test Part ||
 *=======================
 */

/*====================
 *|| Device Layer Part ||
 *====================
 */
static int SAR_sensor_turn_onoff(bool bOn)
{	
	int ret=0;

	/* Check Hardware Support First */
	if(sar_sensor_hw_client->SAR_sensor_hw_turn_onoff == NULL) {
		err("SAR_sensor_hw_turn_onoff NOT SUPPORT.\n");
		return -ENOENT;
	}
	
	if (bOn == 1) {	/* power on */
		if(g_sar_sensor_data->Device_switch_on == false){
			/* Power On */
			ret = sar_sensor_hw_client->SAR_sensor_hw_turn_onoff(true);
			if (ret < 0)	{
				err("SAR sensor turn on ERROR\n");
			} else {
				g_sar_sensor_data->Device_switch_on = bOn;
				/* Enable IRQ */
				log("[IRQ] Enable irq !!\n");
				enable_irq(ASUS_SAR_SENSOR_IRQ);
				/*change the Device Status*/
				g_sar_sensor_data->Device_switch_on = true;
			}
		}
	} else { /* power off */
		if(g_sar_sensor_data->Device_switch_on == true){
			/*disable IRQ before switch off*/
			log("[IRQ] Disable irq !!\n");
			disable_irq_nosync(ASUS_SAR_SENSOR_IRQ);
			
			/*Power OFF*/
			ret = sar_sensor_hw_client->SAR_sensor_hw_turn_onoff(false);
			if (ret < 0)	{
				err("SAR sensor turn off ERROR\n");
			}
			g_sar_sensor_data->Device_switch_on = false;
			
			/* reset global variable */
			g_sar_sensor_data->Device_Interrupt_detect_status = -EBUSY;
			g_sar_sensor_data->Device_detected_status[0] = false;
			g_sar_sensor_data->Device_detected_status[1] = false;
			g_sar_sensor_data->Device_detected_status[2] = false;
			g_sar_sensor_data->Device_detected_status[3] = false;
		}
	}
	return 0;
}

/*******************/
/*BMMI Function*/
/*******************/
bool mSAR_sensor_show_atd_test(void)
{
	int ret = 0;
	int round = 0;

	mutex_lock(&g_sar_sensor_lock);

	/* Check SAR sensor HW ID */
	ret = sar_sensor_hw_client->SAR_sensor_hw_check_ID();
	if (ret < 0) {
		err("SAR sensor ATD test check ID ERROR\n");
		goto SAR_sensor_atd_test_fail;
	}

	if(g_sar_sensor_data->Device_switch_on == false) {
		/* Trun on SAR sensor */
		ret = sar_sensor_hw_client->SAR_sensor_hw_turn_onoff(true);
		if (ret < 0) {
			err("SAR sensor ATD test turn on ERROR\n");
			goto SAR_sensor_atd_test_fail;
		}
	}
 
	/* Read SAR sensor raw data 4 time */
	for(round = 0; round < 4; round++) {
		ret = sar_sensor_hw_client->SAR_sensor_hw__get_proxuserful(round);
		if (ret < 0) {
			err("SAR sensor ATD test get raw data ERROR\n");
			goto SAR_sensor_atd_test_fail;
		}
		msleep(100);
	}

	if(g_sar_sensor_data->HAL_switch_on == false) {
		/* Trun off SAR sensor */
		ret = sar_sensor_hw_client->SAR_sensor_hw_turn_onoff(false);
		if(ret < 0) {
			err("SAR sensor ATD test turn off ERROR\n");
			goto SAR_sensor_atd_test_fail;
		}
	}

	mutex_unlock(&g_sar_sensor_lock);

	return true;
SAR_sensor_atd_test_fail:
	return false;
}

int mSAR_sensor_show_raw_data(void)
{
	int ret = 0;
	
	if(sar_sensor_hw_client->SAR_sensor_hw_read_rawData == NULL) {
		err("SAR_sensor_hw_read_rawData NOT SUPPORT.\n");
		return -EINVAL;
	}
	
	mutex_lock(&g_sar_sensor_lock);
	
	if(g_sar_sensor_data->Device_switch_on == false) {
		SAR_sensor_turn_onoff(true);
	}
	
	/* Read SAR sensor raw data */
	ret = sar_sensor_hw_client->SAR_sensor_hw_read_rawData();
	if (ret < 0)
		err("SAR sensor get raw data ERROR\n");
	
	if(g_sar_sensor_data->HAL_switch_on == false) {
		SAR_sensor_turn_onoff(false);
	}
	
	mutex_unlock(&g_sar_sensor_lock);
	
	return ret;
}

/*****************************/
/* AP Interface Function*/
/*****************************/
bool mSAR_sensor_show_switch_onoff(void)
{	
	return g_sar_sensor_data->Device_switch_on;
}

int mSAR_sensor_store_switch_onoff(bool bOn)
{
	mutex_lock(&g_sar_sensor_lock);
	
	log("SAR switch = %d.\n", bOn);
	if((g_sar_sensor_data->Device_switch_on != bOn))	{
		if (bOn) {
			/* Turn on SAR */
			g_sar_sensor_data->HAL_switch_on = true;
			SAR_sensor_turn_onoff(true);
		} else {
			/* Turn off SAR */
			g_sar_sensor_data->HAL_switch_on = false;
			SAR_sensor_turn_onoff(false);
		}
	} else	{
		log("SAR Sensor is already %s\n", bOn?"ON":"OFF");
	}

	mutex_unlock(&g_sar_sensor_lock);
	
	return 0;
}

int mSAR_sensor_show_Interrupt_detect_status(void)
{
	return g_sar_sensor_data->Device_Interrupt_detect_status;
}

/*************************/
/*Hardware Function*/
/*************************/
int mSAR_sensor_show_reg(uint8_t addr)
{
	int value;
	if (sar_sensor_hw_client->SAR_sensor_hw_get_register == NULL) {
		err("SAR_sensor_hw_get_register NOT SUPPORT.\n");
		return -EINVAL;
	}

	value = sar_sensor_hw_client->SAR_sensor_hw_get_register(addr);
	log("mSAR_sensor_show_reg, addr=0X%02x, value=0x%x.\n", addr, value);
	return value;
}

int mSAR_sensor_store_reg(uint8_t addr, int value)
{	
	if (sar_sensor_hw_client->SAR_sensor_hw_set_register == NULL) {
		err("SAR_sensor_hw_set_register NOT SUPPORT.\n");
		return -EINVAL;
	}

	sar_sensor_hw_client->SAR_sensor_hw_set_register(addr, value);
	log("mSAR_sensor_store_reg, addr=0x%02x, value=0x%x.\n", addr, value);
	return 0;
}

int mSAR_sensor_show_manual_offset_calibration(void)
{
	int value;
	if (sar_sensor_hw_client->SAR_sensor_hw_get_manual_offset_cal == NULL) {
		err("SAR_sensor_hw_get_manual_offset_calibration NOT SUPPORT. \n");
		return -EINVAL;
	}

	value = sar_sensor_hw_client->SAR_sensor_hw_get_manual_offset_cal();
	log("SAR_sensor_hw_get_manual_offset_cal, value=0x%x.\n", value);
	
	return value;
}

int mSAR_sensor_store_manual_offset_calibration(int value)
{	
	int ret = 0;
	
	if (sar_sensor_hw_client->SAR_sensor_hw_manual_offset_cal == NULL) {
		err("SAR_sensor_hw_manual_offset_calibration NOT SUPPORT. \n");
		return -EINVAL;
	}

	ret = sar_sensor_hw_client->SAR_sensor_hw_manual_offset_cal(value);
	log("SAR_sensor_hw_manual_offset_cal, value=%d.(ret=%d)\n", value, ret);
	
	return ret;
}

/*=======================
 *|| For Sar check status ||
 *========================
 */
bool mSAR_sensor_show_sar_status(void)
{
	int ret = 0;
	int i = 0;
	bool status = false;

	mutex_unlock(&g_sar_sensor_lock);

	if(g_sar_sensor_data->Device_switch_on == false) {
		/* Trun on SAR sensor */
		ret = SAR_sensor_turn_onoff(true);
		if (ret < 0) {
			err("SAR sensor status turn on ERROR\n");
			goto SAR_sensor_status_fail;
		}
	} else {
		log("Sar has been opened, Sar_check_status : %s", 
			g_sar_sensor_data->Device_Interrupt_detect_status?"Close":"Away");
		status = (g_sar_sensor_data->Device_Interrupt_detect_status?true:false);
		goto SAR_sensor_status_fail;
	}
 
	/* Check SAR sensor status */
	for(i = 0; i < 5; i++){
		if(g_sar_sensor_data->Device_Interrupt_detect_status == OBJECT_NEAR || 
				g_sar_sensor_data->Device_Interrupt_detect_status == OBJECT_FAR){
			log("Sar_check_status : %s", 
				g_sar_sensor_data->Device_Interrupt_detect_status?"Close":"Away");
			status = g_sar_sensor_data->Device_Interrupt_detect_status;
			break;
		}
		msleep(100);
	}

	if(g_sar_sensor_data->HAL_switch_on == false) {
		/* Trun off SAR sensor */
		ret = SAR_sensor_turn_onoff(false);
		if(ret < 0) {
			err("SAR sensor status turn off ERROR\n");
			goto SAR_sensor_status_fail;
		}
	}

SAR_sensor_status_fail:
	mutex_unlock(&g_sar_sensor_lock);
	return status;
}

/**************************/
/* Extension Function */
/**************************/
bool mSAR_sensor_show_allreg(void)
{
	if(sar_sensor_hw_client->SAR_sensor_hw_show_allreg == NULL) {
		err("SAR_sensor_hw_show_allreg NOT SUPPORT.\n");
		return false;
	}
	sar_sensor_hw_client->SAR_sensor_hw_show_allreg();
	return true;
}

/******************  ATTR Structure ****************/
static SAR_sensor_ATTR_BMMI mSAR_sensor_ATTR_BMMI = {
	.SAR_sensor_show_atd_test                = mSAR_sensor_show_atd_test,
	.SAR_sensor_show_raw_data                = mSAR_sensor_show_raw_data,
};

static SAR_sensor_ATTR_HAL mSAR_sensor_ATTR_HAL = {
	.SAR_sensor_show_switch_onoff            = mSAR_sensor_show_switch_onoff,
	.SAR_sensor_store_switch_onoff           = mSAR_sensor_store_switch_onoff,
	.SAR_sensor_show_Interrupt_detect_status = mSAR_sensor_show_Interrupt_detect_status,
	.SAR_sensor_show_sar_status              = mSAR_sensor_show_sar_status,
};

static SAR_sensor_ATTR_Hardware mSAR_sensor_ATTR_Hardware = {
	.SAR_show_reg                            = mSAR_sensor_show_reg,
	.SAR_store_reg                           = mSAR_sensor_store_reg,
	.SAR_sensor_show_manual_offset_cal       = mSAR_sensor_show_manual_offset_calibration,
	.SAR_sensor_store_manual_offset_cal      = mSAR_sensor_store_manual_offset_calibration,
};

static SAR_sensor_ATTR_Extension mSAR_sensor_ATTR_Extension = {
	.SAR_sensor_show_allreg                  = mSAR_sensor_show_allreg,
};

static SAR_sensor_ATTR mSAR_sensor_ATTR = {
	.info_type       = &mSAR_sensor_info_type,
	.ATTR_BMMI       = &mSAR_sensor_ATTR_BMMI,
	.ATTR_HAL        = &mSAR_sensor_ATTR_HAL,
	.ATTR_Hardware   = &mSAR_sensor_ATTR_Hardware,
	.ATTR_Extension  = &mSAR_sensor_ATTR_Extension,
};

/*==========================
 *|| Interrupt Service Routine Part ||
 *===========================
 */
static void SAR_get_detected_cs(int irq_status, int sensor_status)
{
	if(irq_status == OBJECT_NEAR){
		if((sensor_status & PROXSTAT0) == PROXSTAT0 &&
			g_sar_sensor_data->Device_detected_status[0] == false) {
			log("SAR sensor CS0 is detected.\n");
			g_sar_sensor_data->Device_detected_status[0] = true;
		}
		if((sensor_status & PROXSTAT1) == PROXSTAT1 &&
			g_sar_sensor_data->Device_detected_status[1] == false) {
			log("SAR sensor CS1 is detected.\n");
			g_sar_sensor_data->Device_detected_status[1] = true;
		}
		if((sensor_status & PROXSTAT2) == PROXSTAT2 &&
			g_sar_sensor_data->Device_detected_status[2] == false) {
			log("SAR sensor CS2 is detected.\n");
			g_sar_sensor_data->Device_detected_status[2] = true;
		}
		if((sensor_status & PROXSTAT3) == PROXSTAT3 &&
			g_sar_sensor_data->Device_detected_status[3] == false) {
			log("SAR sensor CS3 is detected.\n");
			g_sar_sensor_data->Device_detected_status[3] = true;
		}
	} else if(irq_status == OBJECT_FAR){
		if((sensor_status & PROXSTAT0) == 0x00 &&
			g_sar_sensor_data->Device_detected_status[0] == true) {
			log("SAR sensor CS0 is detected.\n");
			g_sar_sensor_data->Device_detected_status[0] = false;
		}
		if((sensor_status & PROXSTAT1) == 0x00 &&
			g_sar_sensor_data->Device_detected_status[1] == true) {
			log("SAR sensor CS1 is detected.\n");
			g_sar_sensor_data->Device_detected_status[1] = false;
		}
		if((sensor_status & PROXSTAT2) == 0x00 &&
			g_sar_sensor_data->Device_detected_status[2] == true) {
			log("SAR sensor CS2 is detected.\n");
			g_sar_sensor_data->Device_detected_status[2] = false;
		}
		if((sensor_status & PROXSTAT3) == 0x00 &&
			g_sar_sensor_data->Device_detected_status[3] == true) {
			log("SAR sensor CS3 is detected.\n");
			g_sar_sensor_data->Device_detected_status[3] = false;
		}
	}
} 
 
static int SAR_sensor_work(void)
{
	int sensor_status = 0, irq_status = 0;
	
	sensor_status = sar_sensor_hw_client->SAR_sensor_hw_get_interrupt();
	if (sensor_status < 0) {
		err("sensor_status ERROR(err:%d)\n", sensor_status);
		return sensor_status;
	}
	
	/* Clean interrupt */
	irq_status = sar_sensor_hw_client->SAR_sensor_hw_read_regStat();
	if (irq_status < 0)	{
		err("irq_status ERROR(err:%d)\n", irq_status);
		return irq_status;
	}

	dbg("sensor status:0x%x, irq status:0x%x\n", sensor_status, irq_status);

	if(irq_status == OBJECT_NEAR) {
		if(g_sar_sensor_data->Device_Interrupt_detect_status != OBJECT_NEAR){
			SAR_sensor_report_abs(SAR_SENSOR_REPORT_CLOSE);
			log("SAR sensor Detect object Near.\n");
			g_sar_sensor_data->Device_Interrupt_detect_status = OBJECT_NEAR;
		}	
	} else if(irq_status == OBJECT_FAR) {
		if(g_sar_sensor_data->Device_Interrupt_detect_status != OBJECT_FAR) {
			SAR_sensor_report_abs(SAR_SENSOR_REPORT_AWAY);
			log("SAR sensor Detect object Far.\n");
			g_sar_sensor_data->Device_Interrupt_detect_status = OBJECT_FAR;
		}
	} else {
		err("Unknown irq status(0x%02x)\n", irq_status);
	}
	
	SAR_get_detected_cs(irq_status, sensor_status);
	
	return 0;
}

static void SAR_sensor_ist(struct work_struct *work)
{
mutex_lock(&g_sar_sensor_lock);
	if (g_sar_sensor_data->Device_switch_on == false)	{
		err("SAR sensor are disabled and ignore IST.\n");
		goto ist_err;
	}
	dbg("SAR sensor ist +++\n");
	if (sar_sensor_hw_client == NULL)	{
		dbg("sar_sensor_hw_client is NULL\n");
		goto ist_err;
	}

	/**************************************************/
	/* Check IRQ Status and refresh status    */
	/* Read INT_FLAG will clean the interrupt */
	/**************************************************/
	SAR_sensor_work();
	dbg("SAR sensor ist ---\n");
ist_err:	
	wake_unlock(&g_sar_sensor_wake_lock);
	if (g_sar_sensor_data->Device_switch_on == true )	{
		dbg("[IRQ] Enable irq !!\n");
		enable_irq(ASUS_SAR_SENSOR_IRQ);	
	}
mutex_unlock(&g_sar_sensor_lock);
}

void SAR_sensor_irq_handler(void)
{
	dbg("[IRQ] Disable irq !!\n");
	disable_irq_nosync(ASUS_SAR_SENSOR_IRQ);
	
	if(sar_sensor_hw_client->SAR_sensor_hw_get_interrupt == NULL) {
		err("SAR_sensor_hw_get_interrupt NOT SUPPORT.\n");
		goto irq_err;
	}

	/*Queue work will enbale IRQ and unlock wake_lock*/
	queue_work(SAR_sensor_workqueue, &SAR_sensor_ist_work);
	wake_lock(&g_sar_sensor_wake_lock);
	
	return;
irq_err:
	dbg("[IRQ] Enable irq !!\n");
	enable_irq(ASUS_SAR_SENSOR_IRQ);
}

static SAR_sensor_GPIO mSAR_sensor_GPIO = {
	.SAR_sensor_isr = SAR_sensor_irq_handler,
};

/*====================
 *|| Initialization Part ||
 *====================
 */
static int init_data(void)
{
	int ret = 0;
	/* Reset ASUS_SAR_sensor_data */
	g_sar_sensor_data = kmalloc(sizeof(struct ASUS_sar_sensor_data), GFP_KERNEL);
	if (!g_sar_sensor_data) {
		err("g_sar_sensor_data kmalloc ERROR\n");
		ret = -ENOMEM;
		goto init_data_err;
	}
	memset(g_sar_sensor_data, 0, sizeof(struct ASUS_sar_sensor_data));
	g_sar_sensor_data->Device_switch_on = false;
	g_sar_sensor_data->HAL_switch_on = false;
	g_sar_sensor_data->Device_Interrupt_detect_status = -EBUSY;
	g_sar_sensor_data->Device_detected_status[0] = false;
	g_sar_sensor_data->Device_detected_status[1] = false;
	g_sar_sensor_data->Device_detected_status[2] = false;
	g_sar_sensor_data->Device_detected_status[3] = false;
	return 0;
init_data_err:
	err("Init Data ERROR\n");
	return ret;
}
 
void mSAR_sensor_algo_probe(struct i2c_client *client)
{	
	log("Driver PROBE +++\n");

	/*check i2c client*/
	if (client == NULL) {
		err("i2c Client is NUll\n");
		goto probe_err;
	}	

	/*link driver data to i2c client*/
	strlcpy(client->name, SENSOR_TYPE_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, g_sar_sensor_data);	

	/* i2c client */
	g_i2c_client = client;
	if (ASUS_SAR_SENSOR_IRQ < 0)		
		goto probe_err;	

	log("Driver PROBE ---\n");
	return ;
probe_err:
	err("Driver PROBE ERROR ---\n");
	return;

}

void SAR_sensor_algo_remove(void)
{
	log("Driver REMOVE +++\n");
	
	SAR_sensor_gpio_unregister(ASUS_SAR_SENSOR_IRQ);
	
	log("Driver REMOVE ---\n");
	
	return;
}

void mSAR_sensor_algo_shutdown(void)
{
	log("Driver SHUTDOWN +++\n");

	/* Disable sensor */
	if (g_sar_sensor_data->Device_switch_on)
		SAR_sensor_turn_onoff(false);	

	log("Driver SHUTDOWN ---\n");
	
	return;
}

void mSAR_sensor_algo_suspend(void)
{
	log("Driver SUSPEND +++\n");
	
	/* Disable SAR sensor IRQ */
	log("[IRQ] Disable irq !!\n");
	disable_irq_nosync(ASUS_SAR_SENSOR_IRQ);
	
	log("Driver SUSPEND ---\n");
	
	return;
}

void mSAR_sensor_algo_resume(void)
{
	log("Driver RESUME +++\n");
	
	/* Enable SAR sensor IRQ */
	log("[IRQ] Enable irq !!\n");
	enable_irq(ASUS_SAR_SENSOR_IRQ);
	
	log("Driver RESUME ---\n");
	
	return;
}

static SAR_sensor_I2C mSAR_sensor_I2C = {
	.SAR_sensor_probe = mSAR_sensor_algo_probe,
	.SAR_sensor_remove = SAR_sensor_algo_remove,
	.SAR_sensor_shutdown = mSAR_sensor_algo_shutdown,
	.SAR_sensor_suspend = mSAR_sensor_algo_suspend,
	.SAR_sensor_resume = mSAR_sensor_algo_resume,
};

static int __init SAR_sensor_init(void)
{
	int ret = 0;
	log("Driver INIT +++\n");

	/*Record the error message*/
	g_error_mesg = kzalloc(sizeof(char [ERROR_MESG_SIZE]), GFP_KERNEL);
	
	/* Work Queue */
	SAR_sensor_workqueue = create_singlethread_workqueue(SENSOR_TYPE_NAME"_wq");

	/* Initialize the Mutex */
	mutex_init(&g_sar_sensor_lock);

	/* Initialize the wake lock */
	wake_lock_init(&g_sar_sensor_wake_lock, WAKE_LOCK_SUSPEND, "SAR_sensor_wake_lock");

	/* i2c Registration for probe/suspend/resume */				
	ret = SAR_sensor_i2c_register(&mSAR_sensor_I2C);
	if (ret < 0)
		goto init_err;
	
	/* Hardware Register Initialization */
	sar_sensor_hw_client = SAR_sensor_hw_getHardware();
	if(sar_sensor_hw_client == NULL)
		goto init_err;

	/* driver data structure initialize */
	ret = init_data();
	if (ret < 0)
		goto init_err;

	/* string copy the character of vendor and module number */
	strcpy(mSAR_sensor_ATTR.info_type->vendor, sar_sensor_hw_client->vendor);
	strcpy(mSAR_sensor_ATTR.info_type->module_number, sar_sensor_hw_client->module_number);

	/* Attribute */
	ret = SAR_sensor_ATTR_register(&mSAR_sensor_ATTR);
	if (ret < 0)
		goto init_err;

	/* Input Device */
	ret = SAR_sensor_report_register();
	if (ret < 0)
		goto init_err;	

	ASUS_SAR_SENSOR_IRQ = SAR_sensor_gpio_register(g_i2c_client, &mSAR_sensor_GPIO);
	if (ASUS_SAR_SENSOR_IRQ < 0)
		goto init_err;	

	log("Driver INIT ---\n");
	return 0;

init_err:
	err("Driver INIT ERROR ---\n");
	return ret;
}

static void __exit SAR_sensor_exit(void)
{
	log("Driver EXIT +++\n");

	/* i2c Unregistration */	
	SAR_sensor_i2c_unregister();

	/*Report Unregistration*/
	SAR_sensor_report_unregister();

	/*ATTR Unregistration*/
	SAR_sensor_ATTR_unregister();	

	wake_lock_destroy(&g_sar_sensor_wake_lock);
	mutex_destroy(&g_sar_sensor_lock);
	kfree(g_sar_sensor_data);
	
	destroy_workqueue(SAR_sensor_workqueue);
	
	log("Driver EXIT ---\n");
}

module_init(SAR_sensor_init);
module_exit(SAR_sensor_exit);

MODULE_AUTHOR("Show_Cai <show_cai@asus.com>");
MODULE_DESCRIPTION("SAR sensor");
MODULE_LICENSE("GPL");

