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

/*********************************/
/* Hall Sensor Hardware Module */
/********************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input/ASH.h>

 /**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_HW"
#define SENSOR_TYPE_NAME		"Hallsensor"

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
static HALLsensor_Platform* g_HALLsensor_Platform;

static int hall_sensor_probe(struct platform_device *pdev)
{	
	if(g_HALLsensor_Platform->HALLsensor_probe == NULL){
		err("HALLsensor_probe NOT implement. \n");
		return -EINVAL;
	}

	g_HALLsensor_Platform->HALLsensor_probe(pdev);
	return 0;
}

static int hall_sensor_suspend(struct platform_device *pdev, pm_message_t state)
{
	if(g_HALLsensor_Platform->HALLsensor_suspend == NULL){
		err("HALLsensor_suspend NOT implement. \n");
		return -EINVAL;
	}

	g_HALLsensor_Platform->HALLsensor_suspend();
	return 0;
}

static int hall_sensor_resume(struct platform_device *pdev)
{
	if(g_HALLsensor_Platform->HALLsensor_resume == NULL){
		err("HALLsensor_resume NOT implement. \n");
		return -EINVAL;
	}

	g_HALLsensor_Platform->HALLsensor_resume();
	return 0;
}

static const struct platform_device_id hall_id_table[] = {
        {"hall_sensor", 1},
};

static struct of_device_id hallsensor_match_table[] = {
	{ .compatible = "qcom,hall",},
	{},
};

static struct platform_driver hall_sensor_driver = {
	.driver = {
		.name = "hall_sensor",
		.owner = THIS_MODULE,
		.of_match_table = hallsensor_match_table,
	},
	.probe          = hall_sensor_probe,	
	.suspend	= hall_sensor_suspend,
	.resume	= hall_sensor_resume,
	.id_table	= hall_id_table,
};

int HALLsensor_platform_register(HALLsensor_Platform *hall_platform)
{
	int ret = 0;
	
	if(hall_platform == NULL){
		err("%s: HALLsensor_Platform is NULL pointer. \n", __FUNCTION__);
		return -EINVAL;
	}
	
	g_HALLsensor_Platform = hall_platform;
	
	/* Platform Driver Registeration */	
	ret = platform_driver_register(&hall_sensor_driver);
	if (ret != 0){
		err("%s: platform_driver_register Error(%d)\n", __FUNCTION__, ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(HALLsensor_platform_register);

int HALLsensor_platform_unregister(void)
{
	platform_driver_unregister(&hall_sensor_driver);

	return 0;
}
EXPORT_SYMBOL(HALLsensor_platform_unregister);

