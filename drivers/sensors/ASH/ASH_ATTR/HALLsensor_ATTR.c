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
/* Hall Sensor Attribute */
/************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/input/ASH.h>
#include <linux/slab.h>

HALLsensor_ATTR *g_hall_ATTR = NULL;
struct device *g_hallsensor_dev;

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_ATTR"
#define SENSOR_TYPE_NAME		"Hall"

#undef dbg
#ifdef ASH_ATTR_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

static ssize_t show_action_status(struct device *dev,struct device_attribute *attr, char *buf)
{
	bool action_status = false;

	if(g_hall_ATTR->show_action_status == NULL) {
		err("show_action_status NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}

	action_status=g_hall_ATTR->show_action_status();
	
	return sprintf(buf, "%d\n", action_status);
}

/* +++ For stress test debug +++ */
static ssize_t show_hall_sensor_int_count(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if(g_hall_ATTR->show_hall_sensor_int_count == NULL) {
		err("show_hall_sensor_int_count NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	return sprintf(buf, "%d\n", g_hall_ATTR->show_hall_sensor_int_count());
}

static ssize_t show_hall_sensor_event_count(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if(g_hall_ATTR->show_hall_sensor_event_count == NULL) {
		err("show_hall_sensor_event_count NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	return sprintf(buf, "%d\n", g_hall_ATTR->show_hall_sensor_event_count());
}

static ssize_t show_hall_sensor_error_mesg(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	char* error_mesg=kzalloc(sizeof(char [ERROR_MESG_SIZE]), GFP_KERNEL);	
	
	if(g_hall_ATTR->show_hall_sensor_error_mesg== NULL) {
		err("show_hall_sensor_error_mesg NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}

	ret = g_hall_ATTR->show_hall_sensor_error_mesg(error_mesg);
	
	return sprintf(buf, "%s\n", error_mesg);
}
/* --- For stress test debug --- */

static ssize_t show_hall_sensor_enable(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	bool bOn = false;
	
       if(g_hall_ATTR->show_hall_sensor_enable== NULL) {
		err("show_hall_sensor_enable NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}

	bOn = g_hall_ATTR->show_hall_sensor_enable();
	if(bOn)
		return sprintf(buf, "on\n");
	else
		return sprintf(buf, "off\n");	
}

static ssize_t store_hall_sensor_enable(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	bool bOn = false;
	
	if(g_hall_ATTR->store_hall_sensor_enable == NULL) {
		err("store_hall_sensor_enable NOT SUPPORT. \n");
		return count;
	}

	/*check input character*/
	if (0 == strncmp(buf, "off", 3))
		bOn = false;
	else if (0 == strncmp(buf, "on", 2)) 
		bOn = true;
	else
		return -EINVAL;

	if(g_hall_ATTR->store_hall_sensor_enable(bOn) < 0)
		return -EINVAL;
	log("Hall Sensor switch %s\n", bOn?"on":"off");	
	
	return count;
}

static ssize_t show_hall_sensor_debounce(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	if(g_hall_ATTR->show_hall_sensor_debounce == NULL) {
		err("show_hall_sensor_debounce NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	return sprintf(buf, "%d\n", g_hall_ATTR->show_hall_sensor_debounce());
}

static ssize_t store_hall_sensor_debounce(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	int debounce = 0;

	if(g_hall_ATTR->store_hall_sensor_debounce == NULL) {
		err("store_hall_sensor_debounce NOT SUPPORT. \n");
		return count;
	}
	
	sscanf(buf, "%d", &debounce);
	if(g_hall_ATTR->store_hall_sensor_debounce(debounce) < 0)
		return -EINVAL;
	log("Hall Sensor store debounce: %d\n", debounce);	
	
	return count;
}

static struct device_attribute hallsensor_property_attrs[] = {
	/*read only*/
	__ATTR(status, 0444, show_action_status, NULL),
		/* +++ For stress test debug +++ */
	__ATTR(int_counter, 0444, show_hall_sensor_int_count, NULL),
	__ATTR(event_counter, 0444, show_hall_sensor_event_count, NULL),
	__ATTR(error_mesg, 0444, show_hall_sensor_error_mesg, NULL),
		/* --- For stress test debug --- */
	/*read/write*/
	__ATTR(switch, 0664, show_hall_sensor_enable, store_hall_sensor_enable),
	__ATTR(debounce, 0664, show_hall_sensor_debounce, store_hall_sensor_debounce),
};

int HALLsensor_ATTR_register(HALLsensor_ATTR *mATTR)
{
	int ret = 0;
	int ATTR_index;
		
	g_hall_ATTR = mATTR;

	/* hallsensor device */
	g_hallsensor_dev = ASH_ATTR_device_create(hallsensor);
	if (IS_ERR(g_hallsensor_dev) || g_hallsensor_dev == NULL) {
		ret = PTR_ERR(g_hallsensor_dev);
		err("%s: ASH_ATTR_device_create ERROR(%d).\n", __FUNCTION__, ret);
		return ret;
	}	
	for (ATTR_index=0; ATTR_index < ARRAY_SIZE(hallsensor_property_attrs); ATTR_index++) {
		ret = device_create_file(g_hallsensor_dev, &hallsensor_property_attrs[ATTR_index]);
		if (ret)
			return ret;
	}
	
	return 0;
}

int HALLsensor_ATTR_unregister(void)
{
	ASH_ATTR_device_remove(hallsensor);
	return 0;
}

int HALLsensor_ATTR_create(struct device_attribute *mhallsensor_attr)
{
	int ret = 0;
	if(mhallsensor_attr == NULL) {
		err("%s: the device_attribute is NULL pointer.\n", __FUNCTION__);
		return -EINVAL;
	}
	ret = device_create_file(g_hallsensor_dev, mhallsensor_attr);
	if (ret){		
		err("%s: device_create_file ERROR(%d). \n", __FUNCTION__, ret);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL(HALLsensor_ATTR_create);

