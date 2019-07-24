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

 /****************************/
/* Light Sensor Atrribute */
/***************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/input/ASH.h>

#define RGB 1
#define BUF_SIZE	(10)
lsensor_ATTR *g_light_ATTR = NULL;
struct device *g_lsensor_dev;

/*******************************/
/* Debug and Log System */
/*****************************/
#define MODULE_NAME			"ASH_ATTR"
#define SENSOR_TYPE_NAME		"light"


#undef dbg
#ifdef ASH_ATTR_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

ssize_t  ATT_light_show_vendor(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	if(strcmp(g_light_ATTR->info_type->vendor, "") == 0) {
		err("Show vendor NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	return sprintf(buf, "%s\n", g_light_ATTR->info_type->vendor);
}

ssize_t  ATT_light_show_module_number(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	if(strcmp(g_light_ATTR->info_type->module_number, "") == 0) {
		err("Show module number NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	return sprintf(buf, "%s\n", g_light_ATTR->info_type->module_number);
}

/**************************/
/*Calibration Function*/
/************************/
ssize_t ATT_light_show_calibration(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int calvalue;
	
	if(g_light_ATTR->ATTR_Calibration->light_show_calibration == NULL) {
		err("light_show_calibration NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	calvalue = g_light_ATTR->ATTR_Calibration->light_show_calibration();
	dbg("Light Sensor show Calibration: %d\n", calvalue);
	return sprintf(buf, "%d\n", calvalue);
}	

ssize_t ATT_light_store_calibration(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long calvalue;

	if(g_light_ATTR->ATTR_Calibration->light_store_calibration == NULL) {
		err("light_store_calibration NOT SUPPORT. \n");
		return count;
	}
	
	if ((kstrtoul(buf, 10, &calvalue) < 0))
		return -EINVAL;
	if(calvalue < 0) {
		err("Light Sensor store Calibration with NEGATIVE value. (%lu) \n", calvalue);
		return -EINVAL;
	}

	log("Light Sensor store Calibration: %lu\n", calvalue);	
	if(g_light_ATTR->ATTR_Calibration->light_store_calibration(calvalue) < 0)
		return -EINVAL;	
			
	return count;
}

ssize_t  ATT_light_show_gain(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int gainvalue = 0;
	if(g_light_ATTR->ATTR_Calibration->light_show_gain == NULL) {
		err("light_show_gain NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	gainvalue = g_light_ATTR->ATTR_Calibration->light_show_gain();
	return sprintf(buf, "%d.%05d\n", 
		gainvalue/LIGHT_GAIN_ACCURACY_CALVALUE, gainvalue%LIGHT_GAIN_ACCURACY_CALVALUE);
}

ssize_t ATT_light_show_adc(struct device *dev, struct device_attribute *attr, char *buf)
{
	int adc = 0;
	if(g_light_ATTR->ATTR_Calibration->light_show_adc == NULL) {
		err("light_show_adc NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	adc = g_light_ATTR->ATTR_Calibration->light_show_adc();
	return sprintf(buf, "%d\n", adc);
}

/********************/
/*BMMI Function*/
/*******************/
ssize_t  ATT_light_show_atd_test(struct device *dev, struct device_attribute *attr, char *buf)
{
	bool atd_test = false;
	if(g_light_ATTR->ATTR_BMMI->light_show_atd_test== NULL) {
		err("light_show_atd_test NOT SUPPORT. \n");
		return sprintf(buf, "%d\n", atd_test);
	}
	atd_test = g_light_ATTR->ATTR_BMMI->light_show_atd_test();
	return sprintf(buf, "%d\n", atd_test);	
}

/************************/
/*Hardware Function*/
/***********************/
ssize_t  ATT_light_show_read_reg(struct device *dev, struct device_attribute *attr, char *buf)
{	
	int i2c_reg_addr = 0, i2c_reg_value = 0;
	
	if(g_light_ATTR->ATTR_Hardware->light_show_reg== NULL) {
		err("IRsensor_store_reg NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}

	i2c_reg_addr = g_light_ATTR->ATTR_Hardware->show_reg_addr;
	i2c_reg_value = g_light_ATTR->ATTR_Hardware->light_show_reg(i2c_reg_addr);
	
	return sprintf(buf, "%d\n", i2c_reg_value);
}

ssize_t  ATT_light_store_read_reg(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	int i2c_reg_addr = 0;
	
	if(g_light_ATTR->ATTR_Hardware->light_show_reg== NULL) {
		err("IRsensor_store_reg NOT SUPPORT. \n");
		return count;
	}

	sscanf(buf, "%x", &i2c_reg_addr);
	g_light_ATTR->ATTR_Hardware->show_reg_addr=i2c_reg_addr;
	
	return count;
}
ssize_t  ATT_light_store_write_reg(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	int i2c_reg_addr = 0, i2c_reg_value = 0;
	
	if(g_light_ATTR->ATTR_Hardware->light_store_reg== NULL) {
		err("IRsensor_store_reg NOT SUPPORT. \n");
		return count;
	}
	
	sscanf(buf, "%x %d", &i2c_reg_addr, &i2c_reg_value);

	log("IRsensor_store_reg, addr=%02X, value=%02X\n", i2c_reg_addr, i2c_reg_value);
	if(g_light_ATTR->ATTR_Hardware->light_store_reg(i2c_reg_addr, i2c_reg_value) < 0)
		return -EINVAL;		
	
	return count;
}

/******************/
/*HAL Function*/
/*****************/
ssize_t  ATT_light_show_switch_onoff(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	bool bOn;
	if(g_light_ATTR->ATTR_HAL->light_show_switch_onoff== NULL) {
		err("light_show_switch_onoff NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	bOn = g_light_ATTR->ATTR_HAL->light_show_switch_onoff();
	if(bOn)
		return sprintf(buf, "on\n");
	else
		return sprintf(buf, "off\n");	
}

ssize_t  ATT_light_store_switch_onoff(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	bool bOn;

	if(g_light_ATTR->ATTR_HAL->light_store_switch_onoff == NULL) {
		err("light_store_switch_onoff NOT SUPPORT. \n");
		return count;
	}
	
	/*check input character*/
	if (0 == strncmp(buf, "off", 3))
		bOn = false;
	else if (0 == strncmp(buf, "on", 2)) 
		bOn = true;
	else
		return -EINVAL;

	log("Light Sensor switch %s\n", bOn?"on":"off");
	if(g_light_ATTR->ATTR_HAL->light_store_switch_onoff(bOn) < 0)
		return -EINVAL;		
	
	return count;
}

ssize_t ATT_light_show_lux(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int lux;
	if(g_light_ATTR->ATTR_HAL->light_show_lux== NULL) {
		err("light_show_lux NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	lux = g_light_ATTR->ATTR_HAL->light_show_lux();
	return sprintf(buf, "%d\n", lux);
}

/************************/
/*Extension Function*/
/***********************/
ssize_t  ATT_light_show_allreg(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	bool ret;
	
	if(g_light_ATTR->ATTR_Extension->light_show_allreg== NULL) {
		err("IRsensor_show_allreg NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	ret=g_light_ATTR->ATTR_Extension->light_show_allreg();
	return sprintf(buf, "%d\n", ret);
}

ssize_t  ATT_light_show_sensitivity(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int sensitivity = 0;
	if(g_light_ATTR->ATTR_Extension->light_show_sensitivity == NULL) {
		err("light_show_sensitivity NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	sensitivity = g_light_ATTR->ATTR_Extension->light_show_sensitivity();
	return sprintf(buf, "%d\n", sensitivity);
}

ssize_t  ATT_light_store_sensitivity(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long sensitivity;	

	if(g_light_ATTR->ATTR_Extension->light_store_sensitivity == NULL) {
		err("light_store_sensitivity NOT SUPPORT. \n");
		return count;
	}
	
	if ((kstrtoul(buf, 10, &sensitivity) < 0))
		return -EINVAL;
	if(sensitivity < 0) {
		err("Light Sensor store Sensitivity with NEGATIVE value. (%lu) \n", sensitivity);
		return -EINVAL;
	}

	log("Light Sensor store Sensitivity: %lu\n", sensitivity);
	if(g_light_ATTR->ATTR_Extension->light_store_sensitivity(sensitivity) < 0)
		return -EINVAL;	
	
	return count;
}

ssize_t  ATT_light_show_log_threshold(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int log_threshold = 0;
	if(g_light_ATTR->ATTR_Extension->light_show_log_threshold == NULL) {
		err("light_show_log_threshold NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	log_threshold = g_light_ATTR->ATTR_Extension->light_show_log_threshold();
	return sprintf(buf, "%d\n", log_threshold);
}

ssize_t  ATT_light_store_log_threshold(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long log_threshold;	

	if(g_light_ATTR->ATTR_Extension->light_store_log_threshold == NULL) {
		err("light_store_log_threshold NOT SUPPORT. \n");
		return count;
	}
	
	if ((kstrtoul(buf, 10, &log_threshold) < 0))
		return -EINVAL;
	if(log_threshold < 0) {
		err("Light Sensor store Log Threshold with NEGATIVE value. (%lu) \n", log_threshold);
		return -EINVAL;
	}

	log("Light Sensor store Log Threshold: %lu\n", log_threshold);
	if(g_light_ATTR->ATTR_Extension->light_store_log_threshold(log_threshold) < 0)
		return -EINVAL;	
	
	return count;
}

/* +++ For stress test debug +++ */
ssize_t  ATT_light_show_int_count(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int l_int_counter=0;
	
	if(g_light_ATTR->ATTR_Extension->light_show_int_count == NULL) {
		err("light_show_int_count NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	l_int_counter = g_light_ATTR->ATTR_Extension->light_show_int_count();
	return sprintf(buf, "%d\n", l_int_counter);
}

ssize_t  ATT_light_show_event_count(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int l_event_counter=0;
	
	if(g_light_ATTR->ATTR_Extension->light_show_event_count == NULL) {
		err("light_show_event_count NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	l_event_counter = g_light_ATTR->ATTR_Extension->light_show_event_count();
	return sprintf(buf, "%d\n", l_event_counter);
}

ssize_t  ATT_light_show_error_mesg(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int ret;
	char* error_mesg=kzalloc(sizeof(char [ERROR_MESG_SIZE]), GFP_KERNEL);	
	
	if(g_light_ATTR->ATTR_Extension->light_show_error_mesg== NULL) {
		err("IRsensor_show_error_mesg NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}

	ret = g_light_ATTR->ATTR_Extension->light_show_error_mesg(error_mesg);
	
	return sprintf(buf, "%s\n", error_mesg);
}
/* --- For stress test debug --- */

/*For transition period from 3/5 to 2/4*/
ssize_t  ATT_light_show_selection(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int selection = 0;
	if(g_light_ATTR->ATTR_Extension->light_show_selection == NULL) {
		err("light_show_selection NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	selection = g_light_ATTR->ATTR_Extension->light_show_selection();
	return sprintf(buf, "%d\n", selection);
}

ssize_t  ATT_light_store_selection(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long selection;	

	if(g_light_ATTR->ATTR_Extension->light_store_selection == NULL) {
		err("light_store_selection NOT SUPPORT. \n");
		return count;
	}
	
	if ((kstrtoul(buf, 10, &selection) < 0))
		return -EINVAL;
	if(selection < 0) {
		err("Light Sensor store selection with NEGATIVE value. (%lu) \n", selection);
		return -EINVAL;
	}

	log("Light Sensor store selection: %lu\n", selection);
	if(g_light_ATTR->ATTR_Extension->light_store_selection(selection) < 0)
		return -EINVAL;	
	
	return count;
}

static struct device_attribute light_property_attrs[] = {
	/*read only*/
	__ATTR(vendor, 0444, ATT_light_show_vendor, NULL),
	__ATTR(module_number, 0444, ATT_light_show_module_number, NULL),
	__ATTR(adc, 0444, ATT_light_show_adc, NULL),	
	__ATTR(gain, 0444, ATT_light_show_gain, NULL),
	__ATTR(atd_status, 0444, ATT_light_show_atd_test, NULL),
	__ATTR(lux, 0444, ATT_light_show_lux, NULL),
	__ATTR(dump_reg, 0444, ATT_light_show_allreg, NULL),
		/* +++ For stress test debug +++ */
	__ATTR(int_counter, 0444, ATT_light_show_int_count, NULL),
	__ATTR(event_counter, 0444, ATT_light_show_event_count, NULL),
	__ATTR(error_mesg, 0444, ATT_light_show_error_mesg, NULL),
		/* --- For stress test debug --- */
	/*read/write*/
	__ATTR(switch, 0664, ATT_light_show_switch_onoff, ATT_light_store_switch_onoff),
	__ATTR(cal, 0664, ATT_light_show_calibration, ATT_light_store_calibration),
	__ATTR(sensitivity, 0664, ATT_light_show_sensitivity, ATT_light_store_sensitivity),
	__ATTR(read_reg, 0664, ATT_light_show_read_reg, ATT_light_store_read_reg),
	__ATTR(write_reg, 0220, NULL, ATT_light_store_write_reg),
	__ATTR(log_threshold, 0664, ATT_light_show_log_threshold, ATT_light_store_log_threshold),

	/*For transition period from 3/5 to 2/4*/
	__ATTR(selection, 0664, ATT_light_show_selection, ATT_light_store_selection),
};

int lsensor_ATTR_register(lsensor_ATTR *mATTR)
{
	int ret = 0;
	int ATTR_index;
	
	g_light_ATTR=mATTR;
	
	/*lsensor device*/
	g_lsensor_dev = ASH_ATTR_device_create(lsensor);
	if (IS_ERR(g_lsensor_dev) || g_lsensor_dev == NULL) {
		err("%s: lsensor create ERROR.\n", __FUNCTION__);
		ret = PTR_ERR(g_lsensor_dev);
		return ret;
	}
	for (ATTR_index=0; ATTR_index < ARRAY_SIZE(light_property_attrs); ATTR_index++) {
		ret = device_create_file(g_lsensor_dev, &light_property_attrs[ATTR_index]);
		if (ret){
			return ret;
		}
	}
	
	return 0;
}
EXPORT_SYMBOL(lsensor_ATTR_register);

int lsensor_ATTR_unregister(void)
{
	ASH_ATTR_device_remove(lsensor);
	return 0;
}
EXPORT_SYMBOL(lsensor_ATTR_unregister);

int lsensor_ATTR_create(struct device_attribute *mlsensor_attr)
{
	int ret = 0;
	if(mlsensor_attr == NULL) {
		err("%s: the device_attribute is NULL point. \n", __FUNCTION__);
		return -EINVAL;
	}
	ret = device_create_file(g_lsensor_dev, mlsensor_attr);
	if (ret){		
		err("%s: device_create_file ERROR(%d). \n", __FUNCTION__, ret);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL(lsensor_ATTR_create);

