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

/****************************************************/
/* ALSPS FRGB Sensor Hardware Module */
/****************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include "SARsensor_Hardware.h"
#include <linux/input/ASH.h>


/********************************/
/* Debug and Log System */
/********************************/
#define MODULE_NAME			"ASH_HW"
#define SENSOR_TYPE_NAME	"SAR"

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
/***********************/
static SAR_sensor_I2C* g_SAR_sensor_I2C;
static struct i2c_client * g_i2c_client = NULL;

int mSAR_sensor_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	if(g_SAR_sensor_I2C->SAR_sensor_probe == NULL){
		err("SAR_sensor_probe NOT implement. \n");
		return -EINVAL;
	}
	
	g_SAR_sensor_I2C->SAR_sensor_probe(client);

	/* We use Probe function to get i2c client */
	g_i2c_client = client;
	
	return 0;
}

int mSAR_sensor_remove(struct i2c_client *client)
{
	if(g_SAR_sensor_I2C->SAR_sensor_remove == NULL){
		err("SAR_sensor_remove NOT implement. \n");
		return -EINVAL;
	}
	
	g_SAR_sensor_I2C->SAR_sensor_remove();
	return 0;
}

void mSAR_sensor_shutdown(struct i2c_client *client)
{
	if(g_SAR_sensor_I2C->SAR_sensor_shutdown == NULL){
		err("SAR_sensor_shutdown NOT implement. \n");
	}
	
	g_SAR_sensor_I2C->SAR_sensor_shutdown();
}

int mSAR_sensor_suspend(struct device *client)
{
	if(g_SAR_sensor_I2C->SAR_sensor_suspend == NULL){
		err("SAR_sensor_suspend NOT implement. \n");
		return -EINVAL;
	}
	
	g_SAR_sensor_I2C->SAR_sensor_suspend();
	return 0;
}

int mSAR_sensor_resume(struct device *client)
{
	if(g_SAR_sensor_I2C->SAR_sensor_resume == NULL){
		err("SAR_sensor_resume NOT implement. \n");
		return -EINVAL;
	}
	
	g_SAR_sensor_I2C->SAR_sensor_resume();
	return 0;
}

static const struct dev_pm_ops sar_dev_pm_ops = {
	.suspend = mSAR_sensor_suspend,
	.resume = mSAR_sensor_resume,
};

static struct i2c_driver SAR_sensor_i2c_driver_client = {
	.probe = mSAR_sensor_probe,
	.remove = mSAR_sensor_remove,
	.shutdown = mSAR_sensor_shutdown,
	.driver.pm = &sar_dev_pm_ops,
};

int SAR_sensor_i2c_register(SAR_sensor_I2C *SAR_sensor_i2c)
{
	if(SAR_sensor_i2c == NULL){
		err("%s : SAR_sensor_I2C is NULL pointer. \n", __FUNCTION__);
		return -EINVAL;
	}
	
	g_SAR_sensor_I2C = SAR_sensor_i2c;
	return 0;
}
EXPORT_SYMBOL(SAR_sensor_i2c_register);

int SAR_sensor_i2c_unregister(void)
{
	i2c_del_driver(&SAR_sensor_i2c_driver_client);

	return 0;
}
EXPORT_SYMBOL(SAR_sensor_i2c_unregister);

/**************************/
/*sx9310 I2c Driver*/
/*************************/
#ifdef USE_SX9310
static struct i2c_device_id sx9310_i2c_id[] = {
	{ "sx9310", 0 },
	{ }
};

static struct of_device_id sx9310_match_table[] = {
	{ .compatible = "Semtech,sx9310",},
	{ },
};
MODULE_DEVICE_TABLE(i2c, sx9310_idtable);
#endif

static struct i2c_device_id sx9325_i2c_id[] = {
	{ "sx9325", 0 },
	{ }
};

static struct of_device_id sx9325_match_table[] = {
	{ .compatible = "Semtech,sx9325",},
	{ },
};
MODULE_DEVICE_TABLE(i2c, sx9325_idtable);

static int SAR_sensor_hw_setI2cDriver(struct i2c_driver* i2c_driver_client, int hardware_source)
{
	switch(hardware_source) {
#ifdef USE_SX9310
		case SAR_sensor_hw_source_sx9310:
			log("set i2c client : sx9310 \n");
			i2c_driver_client->driver.name = "sx9310";
			i2c_driver_client->driver.owner = THIS_MODULE;
			i2c_driver_client->driver.of_match_table = sx9310_match_table;
			i2c_driver_client->id_table = sx9310_i2c_id;
			break;
#endif
		case SAR_sensor_hw_source_sx9325:
			log("set i2c client : sx9325 \n");
			i2c_driver_client->driver.name = "sx9325";
			i2c_driver_client->driver.owner = THIS_MODULE;
			i2c_driver_client->driver.of_match_table = sx9325_match_table;
			i2c_driver_client->id_table = sx9325_i2c_id;
			break;
		default:
			err("get hardware client ERROR : hardware enum=%d\n", hardware_source);
			return -EINVAL;
	}
	
	return 0;
}

SAR_sensor_hw* SAR_sensor_hw_getHardwareClient(int hardware_source)
{
	SAR_sensor_hw* SAR_sensor_hw_client = NULL;
		
	switch(hardware_source) {
#ifdef USE_SX9310
		case SAR_sensor_hw_source_sx9310:
			log("get hardware client : sx9310 \n");
			SAR_sensor_hw_client = SAR_sensor_hw_sx9310_getHardware();
			break;
#endif
		case SAR_sensor_hw_source_sx9325:
			log("get hardware client : sx9325 \n");
			SAR_sensor_hw_client = SAR_sensor_hw_sx9325_getHardware();
			break;
		default:
			err("get hardware client ERROR : hardware enum=%d\n", hardware_source);
	}

	return SAR_sensor_hw_client;
}

SAR_sensor_hw* SAR_sensor_hw_getHardware(void)
{	
	SAR_sensor_hw* SAR_sensor_hw_client = NULL;
	int SAR_sensor_sensor_source = 0;
	int ret = 0;
	
	/*check i2c function pointer*/
	if(g_SAR_sensor_I2C == NULL) {
		err("g_SAR_sensor_I2C is NULL. Please use 'SAR_sensor_i2c_register' first. \n");
		return NULL;
	}		
	
	/* i2c Registration */	
	for (SAR_sensor_sensor_source = 0; SAR_sensor_sensor_source < SAR_sensor_hw_source_max; 
			SAR_sensor_sensor_source++) {
				
		/* i2c Registration and g_client will get i2c client */
		SAR_sensor_hw_setI2cDriver(&SAR_sensor_i2c_driver_client, SAR_sensor_sensor_source);
		ret = i2c_add_driver(&SAR_sensor_i2c_driver_client);
		if ( ret != 0 ) {
			err("%s: i2c_add_driver ERROR(%d). \n", __FUNCTION__, ret);	
			return NULL;
		}
		if(g_i2c_client == NULL){
			err("%s: g_i2c_client is NULL pointer. \n", __FUNCTION__);	
			return NULL;
		}

		/* get hardware client and check the i2c status */
		SAR_sensor_hw_client = SAR_sensor_hw_getHardwareClient(SAR_sensor_sensor_source);
		if(SAR_sensor_hw_client == NULL){
			err("SAR_sensor_hw_client is NULL pointer. \n");
			return NULL;
		}
			
		if(SAR_sensor_hw_client->SAR_sensor_hw_init == NULL){
			err("SAR_sensor_hw_init is NULL pointer. \n");
			return NULL;
		}

		ret = SAR_sensor_hw_client->SAR_sensor_hw_init(g_i2c_client);
		if (ret < 0) {
			i2c_del_driver(&SAR_sensor_i2c_driver_client);
			log("%s %s Probe Fail. \n", __FUNCTION__, SAR_sensor_i2c_driver_client.driver.name);
			continue;
		}else{
			log("%s %s Probe Success. \n", __FUNCTION__, SAR_sensor_i2c_driver_client.driver.name);
			break;
		}
	
	}
			
	if(SAR_sensor_sensor_source == SAR_sensor_hw_source_max) {
		err("There is NO source can Probe.\n");
		return NULL;
	}

	return 	SAR_sensor_hw_client;
}
EXPORT_SYMBOL(SAR_sensor_hw_getHardware);
