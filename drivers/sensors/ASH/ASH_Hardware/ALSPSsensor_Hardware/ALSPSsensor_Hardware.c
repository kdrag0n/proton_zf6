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

/********************************/
/* ALSPS Sensor Hardware Module */
/******************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include "ALSPSsensor_Hardware.h"
#include <linux/input/ASH.h>

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_HW"
#define SENSOR_TYPE_NAME	"ALSPS"

#undef dbg
#ifdef ASH_HW_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

/********************/
/* Global Variables */
/******************/
static ALSPS_I2C* g_ALSPS_I2C;
static struct i2c_client * g_i2c_client = NULL;

int mALSPS_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	if(g_ALSPS_I2C->ALSPS_probe == NULL){
		err("ALSPS_probe NOT implement. \n");
		return -EINVAL;
	}
	
	g_ALSPS_I2C->ALSPS_probe(client);

	/* We use Probe function to get i2c client */
	g_i2c_client = client;
	
	return 0;
}

int mALSPS_remove(struct i2c_client *client)
{
	if(g_ALSPS_I2C->ALSPS_remove == NULL){
		err("ALSPS_remove NOT implement. \n");
		return -EINVAL;
	}
	
	g_ALSPS_I2C->ALSPS_remove();
	return 0;
}

void mALSPS_shutdown(struct i2c_client *client)
{
	if(g_ALSPS_I2C->ALSPS_shutdown == NULL){
		err("ALSPS_shutdown NOT implement. \n");
	}
	
	g_ALSPS_I2C->ALSPS_shutdown();
}

int mALSPS_suspend(struct device *client)
{
	if(g_ALSPS_I2C->ALSPS_suspend == NULL){
		err("ALSPS_suspend NOT implement. \n");
		return -EINVAL;
	}
	
	g_ALSPS_I2C->ALSPS_suspend();
	return 0;
}

int mALSPS_resume(struct device *client)
{
	if(g_ALSPS_I2C->ALSPS_resume == NULL){
		err("ALSPS_resume NOT implement. \n");
		return -EINVAL;
	}
	
	g_ALSPS_I2C->ALSPS_resume();
	return 0;
}

static const struct dev_pm_ops alsps_dev_pm_ops = {
	.suspend = mALSPS_suspend,
	.resume = mALSPS_resume,
};

static struct i2c_driver ALSPS_i2c_driver_client = {
	.probe = mALSPS_probe,
	.remove = mALSPS_remove,
	.shutdown = mALSPS_shutdown,
	.driver.pm = &alsps_dev_pm_ops,
};

int ALSPS_i2c_register(ALSPS_I2C *alsps_i2c)
{
	if(alsps_i2c == NULL){
		err("%s : ALSPS_I2C is NULL pointer. \n", __FUNCTION__);
		return -EINVAL;
	}
	
	g_ALSPS_I2C = alsps_i2c;
	return 0;
}
EXPORT_SYMBOL(ALSPS_i2c_register);

int ALSPS_i2c_unregister(void)
{
	i2c_del_driver(&ALSPS_i2c_driver_client);

	return 0;
}
EXPORT_SYMBOL(ALSPS_i2c_unregister);

/***********************/
/*CM36686 I2c Driver*/
/**********************/
static const struct i2c_device_id cm36686_i2c_id[] = {
	{"cm36686", 0},
	{}
};

static struct of_device_id cm36686_match_table[] = {
	{ .compatible = "qcom,cm36686",},
	{},
};

static int ALSPS_hw_setI2cDriver(struct i2c_driver* i2c_driver_client, 
	int hardware_source)
{
	switch(hardware_source) {
		case ALSPS_hw_source_cm36686:
			dbg("set i2c client : cm36686 \n");
			i2c_driver_client->driver.name = "cm36686";
			i2c_driver_client->driver.owner = THIS_MODULE;
			i2c_driver_client->driver.of_match_table = cm36686_match_table;
			i2c_driver_client->id_table = cm36686_i2c_id;
			break;

		default:
			err("get hardware client ERROR : hardware enum=%d\n", hardware_source);
			return -EINVAL;
	}
	
	return 0;
}

ALSPS_hw* ALSPS_hw_getHardwareClient(int hardware_source)
{
	ALSPS_hw* ALSPS_hw_client = NULL;
		
	switch(hardware_source) {
		case ALSPS_hw_source_cm36686:
			dbg("get hardware client : cm36686 \n");
			ALSPS_hw_client = ALSPS_hw_cm36686_getHardware();
			break;

		default:
			err("get hardware client ERROR : hardware enum=%d\n", hardware_source);
	}

	return ALSPS_hw_client;
}

ALSPS_hw* ALSPS_hw_getHardware(void)
{	
	ALSPS_hw* ALSPS_hw_client = NULL;
	int ALSPS_sensor_source;
	int ret = 0;
	
	/*check i2c function pointer*/
	if(g_ALSPS_I2C == NULL) {
		err("g_ALSPS_I2C is NULL. Please use 'ALSPS_i2c_register' first. \n");
		return NULL;
	}		
	
	/* i2c Registration */	
	for (ALSPS_sensor_source = 0; ALSPS_sensor_source < ALSPS_hw_source_max; 
			ALSPS_sensor_source++) {
				
		/* i2c Registration and g_client will get i2c client */
		ALSPS_hw_setI2cDriver(&ALSPS_i2c_driver_client, ALSPS_sensor_source);
		ret = i2c_add_driver(&ALSPS_i2c_driver_client);
		if ( ret != 0 ) {
			err("%s: i2c_add_driver ERROR(%d). \n", __FUNCTION__, ret);	
			return NULL;
		}
		if(g_i2c_client == NULL){
			err("%s: g_i2c_client is NULL pointer. \n", __FUNCTION__);	
			return NULL;
		}

		/* get hardware client and check the i2c status */
		ALSPS_hw_client = ALSPS_hw_getHardwareClient(ALSPS_sensor_source);
		if(ALSPS_hw_client == NULL){
			err("ALSPS_hw_client is NULL pointer. \n");
			return NULL;
		}
			
		if(ALSPS_hw_client->ALSPS_hw_init == NULL){
			err("ALSPS_hw_init is NULL pointer. \n");
			return NULL;
		}

		ret = ALSPS_hw_client->ALSPS_hw_init(g_i2c_client);
		if (ret < 0) {
			i2c_del_driver(&ALSPS_i2c_driver_client);
			log("%s %s Probe Fail. \n", __FUNCTION__, ALSPS_i2c_driver_client.driver.name);
			continue;
		}else{
			log("%s %s Probe Success. \n", __FUNCTION__, ALSPS_i2c_driver_client.driver.name);
			break;
		}
	
	}
			
	if(ALSPS_sensor_source == ALSPS_hw_source_max) {
		err("There is NO source can Probe.\n");
		return NULL;
	}

	return ALSPS_hw_client;
}
EXPORT_SYMBOL(ALSPS_hw_getHardware);