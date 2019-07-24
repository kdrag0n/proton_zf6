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

/**************************************************/
/* ALSPS FRGB Sensor Hardware Module */
/************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include "ALSPS_FRGB_Hardware.h"
#include <linux/input/ASH.h>

/**************************/
/* Debug and Log System */
/************************/
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

/***********************/
/* Global Variables */
/**********************/
static ALSPS_FRGB_I2C* g_ALSPS_FRGB_I2C;
static struct i2c_client * g_i2c_client = NULL;

int mALSPS_FRGB_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	if(g_ALSPS_FRGB_I2C->ALSPS_FRGB_probe == NULL){
		err("ALSPS_FRGB_probe NOT implement. \n");
		return -EINVAL;
	}
	
	g_ALSPS_FRGB_I2C->ALSPS_FRGB_probe(client);

	/* We use Probe function to get i2c client */
	g_i2c_client = client;
	
	return 0;
}

int mALSPS_FRGB_remove(struct i2c_client *client)
{
	if(g_ALSPS_FRGB_I2C->ALSPS_FRGB_remove == NULL){
		err("ALSPS_FRGB_remove NOT implement. \n");
		return -EINVAL;
	}
	
	g_ALSPS_FRGB_I2C->ALSPS_FRGB_remove();
	return 0;
}

void mALSPS_FRGB_shutdown(struct i2c_client *client)
{
	if(g_ALSPS_FRGB_I2C->ALSPS_FRGB_shutdown == NULL){
		err("ALSPS_FRGB_shutdown NOT implement. \n");
	}
	
	g_ALSPS_FRGB_I2C->ALSPS_FRGB_shutdown();
}

int mALSPS_FRGB_suspend(struct device *client)
{
	if(g_ALSPS_FRGB_I2C->ALSPS_FRGB_suspend == NULL){
		err("ALSPS_FRGB_suspend NOT implement. \n");
		return -EINVAL;
	}
	
	g_ALSPS_FRGB_I2C->ALSPS_FRGB_suspend();
	return 0;
}

int mALSPS_FRGB_resume(struct device *client)
{
	if(g_ALSPS_FRGB_I2C->ALSPS_FRGB_resume == NULL){
		err("ALSPS_FRGB_resume NOT implement. \n");
		return -EINVAL;
	}
	
	g_ALSPS_FRGB_I2C->ALSPS_FRGB_resume();
	return 0;
}

static const struct dev_pm_ops alsps_dev_pm_ops = {
	.suspend = mALSPS_FRGB_suspend,
	.resume = mALSPS_FRGB_resume,
};

static struct i2c_driver ALSPS_FRGB_i2c_driver_client = {
	.probe = mALSPS_FRGB_probe,
	.remove = mALSPS_FRGB_remove,
	.shutdown = mALSPS_FRGB_shutdown,
	.driver.pm = &alsps_dev_pm_ops,
};

int ALSPS_FRGB_i2c_register(ALSPS_FRGB_I2C *alsps_frgb_i2c)
{
	if(alsps_frgb_i2c == NULL){
		err("%s : ALSPS_FRGB_I2C is NULL pointer. \n", __FUNCTION__);
		return -EINVAL;
	}
	
	g_ALSPS_FRGB_I2C = alsps_frgb_i2c;
	return 0;
}
EXPORT_SYMBOL(ALSPS_FRGB_i2c_register);

int ALSPS_FRGB_i2c_unregister(void)
{
	i2c_del_driver(&ALSPS_FRGB_i2c_driver_client);

	return 0;
}
EXPORT_SYMBOL(ALSPS_FRGB_i2c_unregister);

/**************************/
/*VCNL36863 I2c Driver*/
/*************************/
static const struct i2c_device_id vcnl36863_i2c_id[] = {
	{"vcnl36863", 0},
	{}
};

static struct of_device_id vcnl36863_match_table[] = {
	{ .compatible = "qcom,vcnl36863",},
	{},
};

static int ALSPS_FRGB_hw_setI2cDriver(struct i2c_driver* i2c_driver_client, 
	int hardware_source)
{
	switch(hardware_source) {
		case ALSPS_FRGB_hw_source_vcnl36863:
			dbg("set i2c client : vcnl36863 \n");
			i2c_driver_client->driver.name = "vcnl36863";
			i2c_driver_client->driver.owner = THIS_MODULE;
			i2c_driver_client->driver.of_match_table = vcnl36863_match_table;
			i2c_driver_client->id_table = vcnl36863_i2c_id;
			break;
		default:
			err("get hardware client ERROR : hardware enum=%d\n", hardware_source);
			return -EINVAL;
	}
	
	return 0;
}

ALSPS_FRGB_hw* ALSPS_FRGB_hw_getHardwareClient(int hardware_source)
{
	ALSPS_FRGB_hw* ALSPS_FRGB_hw_client = NULL;
		
	switch(hardware_source) {
		case ALSPS_FRGB_hw_source_vcnl36863:
			dbg("get hardware client : vcnl36863 \n");
			ALSPS_FRGB_hw_client = ALSPS_FRGB_hw_vcnl36863_getHardware();
			break;
		default:
			err("get hardware client ERROR : hardware enum=%d\n", hardware_source);
	}

	return ALSPS_FRGB_hw_client;
}

ALSPS_FRGB_hw* ALSPS_FRGB_hw_getHardware(void)
{	
	ALSPS_FRGB_hw* ALSPS_FRGB_hw_client = NULL;
	int ALSPS_FRGB_sensor_source;
	int ret = 0;
	
	/*check i2c function pointer*/
	if(g_ALSPS_FRGB_I2C == NULL) {
		err("g_ALSPS_FRGB_I2C is NULL. Please use 'ALSPS_FRGB_i2c_register' first. \n");
		return NULL;
	}		
	
	/* i2c Registration */	
	for (ALSPS_FRGB_sensor_source = 0; ALSPS_FRGB_sensor_source < ALSPS_FRGB_hw_source_max; 
			ALSPS_FRGB_sensor_source++) {
				
		/* i2c Registration and g_client will get i2c client */
		ALSPS_FRGB_hw_setI2cDriver(&ALSPS_FRGB_i2c_driver_client, ALSPS_FRGB_sensor_source);
		ret = i2c_add_driver(&ALSPS_FRGB_i2c_driver_client);
		if ( ret != 0 ) {
			err("%s: i2c_add_driver ERROR(%d). \n", __FUNCTION__, ret);	
			return NULL;
		}
		if(g_i2c_client == NULL){
			err("%s: g_i2c_client is NULL pointer. \n", __FUNCTION__);	
			return NULL;
		}

		/* get hardware client and check the i2c status */
		ALSPS_FRGB_hw_client = ALSPS_FRGB_hw_getHardwareClient(ALSPS_FRGB_sensor_source);
		if(ALSPS_FRGB_hw_client == NULL){
			err("ALSPS_FRGB_hw_client is NULL pointer. \n");
			return NULL;
		}
			
		if(ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_init == NULL){
			err("ALSPS_FRGB_hw_init is NULL pointer. \n");
			return NULL;
		}

		ret = ALSPS_FRGB_hw_client->ALSPS_FRGB_hw_init(g_i2c_client);
		if (ret < 0) {
			i2c_del_driver(&ALSPS_FRGB_i2c_driver_client);
			log("%s %s Probe Fail. \n", __FUNCTION__, ALSPS_FRGB_i2c_driver_client.driver.name);
			continue;
		}else{
			log("%s %s Probe Success. \n", __FUNCTION__, ALSPS_FRGB_i2c_driver_client.driver.name);
			break;
		}
	
	}
			
	if(ALSPS_FRGB_sensor_source == ALSPS_FRGB_hw_source_max) {
		err("There is NO source can Probe.\n");
		return NULL;
	}

	return 	ALSPS_FRGB_hw_client;
}
EXPORT_SYMBOL(ALSPS_FRGB_hw_getHardware);
