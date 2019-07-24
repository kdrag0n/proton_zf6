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
/* Proximity Sensor Atrribute */
/********************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/input/ASH.h>

#define RGB 1
#define BUF_SIZE	(10)
psensor_ATTR *g_psensor_ATTR = NULL;
struct device *g_psensor_dev;

/*******************************/
/* Debug and Log System */
/*****************************/
#define MODULE_NAME			"ASH_ATTR"
#define SENSOR_TYPE_NAME		"proximity"


#undef dbg
#ifdef ASH_ATTR_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

ssize_t  ATT_proximity_show_vendor(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	if(strcmp(g_psensor_ATTR->info_type->vendor, "") == 0) {
		err("Show vendor NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	return sprintf(buf, "%s\n", g_psensor_ATTR->info_type->vendor);
}

ssize_t  ATT_proximity_show_module_number(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	if(strcmp(g_psensor_ATTR->info_type->module_number, "") == 0) {
		err("Show module number NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	return sprintf(buf, "%s\n", g_psensor_ATTR->info_type->module_number);
}

/**************************/
/*Calibration Function*/
/************************/
ssize_t ATT_proximity_show_calibration_hi(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int calvalue;

	if(g_psensor_ATTR->ATTR_Calibration->proximity_show_calibration_hi== NULL) {
		err("proximity_show_calibration_hi NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	calvalue = g_psensor_ATTR->ATTR_Calibration->proximity_show_calibration_hi();
	dbg("Proximity show High Calibration: %d\n", calvalue);
	return sprintf(buf, "%d\n", calvalue);
}

ssize_t ATT_proximity_store_calibration_hi(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long calvalue;
	
	if(g_psensor_ATTR->ATTR_Calibration->proximity_store_calibration_hi == NULL) {
		err("proximity_store_calibration_hi NOT SUPPORT. \n");
		return count;
	}
	
	if ((kstrtoul(buf, 10, &calvalue) < 0))
		return -EINVAL;		
	if(calvalue < 0) {
		err("Proximity store High Calibration with NEGATIVE value. (%lu) \n", calvalue);
		return -EINVAL;	
	}	

	log("Proximity store High Calibration: %lu\n", calvalue);
	if(g_psensor_ATTR->ATTR_Calibration->proximity_store_calibration_hi(calvalue) < 0)
		return -EINVAL;	
	
	return count;
}

ssize_t ATT_proximity_show_calibration_lo(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int calvalue;

	if(g_psensor_ATTR->ATTR_Calibration->proximity_show_calibration_lo == NULL) {
		err("proximity_show_calibration_lo NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	calvalue = g_psensor_ATTR->ATTR_Calibration->proximity_show_calibration_lo();
	dbg("Proximity show Low Calibration: %d\n", calvalue);
	return sprintf(buf, "%d\n", calvalue);
}

ssize_t ATT_proximity_store_calibration_lo(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long calvalue;

	if(g_psensor_ATTR->ATTR_Calibration->proximity_store_calibration_lo == NULL) {
		err("proximity_store_calibration_lo NOT SUPPORT. \n");
		return count;
	}
	
	if ((kstrtoul(buf, 10, &calvalue) < 0))
		return -EINVAL;	
	if(calvalue < 0) {
		err("Proximity store Low Calibration with NEGATIVE value. (%lu) \n", calvalue);
		return -EINVAL;
	}

	log("Proximity store Low Calibration: %lu\n", calvalue);
	if(g_psensor_ATTR->ATTR_Calibration->proximity_store_calibration_lo(calvalue) < 0)
		return -EINVAL;	
	
	return count;
}

ssize_t ATT_proximity_show_calibration_inf(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int calvalue;

	if(g_psensor_ATTR->ATTR_Calibration->proximity_show_calibration_inf == NULL) {
		err("proximity_show_calibration_inf NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	calvalue = g_psensor_ATTR->ATTR_Calibration->proximity_show_calibration_inf();
	dbg("Proximity show Inf Calibration: %d\n", calvalue);
	return sprintf(buf, "%d\n", calvalue);
}

ssize_t ATT_proximity_store_calibration_inf(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long calvalue;

	if(g_psensor_ATTR->ATTR_Calibration->proximity_store_calibration_inf == NULL) {
		err("proximity_store_calibration_inf NOT SUPPORT. \n");
		return count;
	}
	
	if ((kstrtoul(buf, 10, &calvalue) < 0))
		return -EINVAL;	
	if(calvalue < 0) {
		err("Proximity store Inf Calibration with NEGATIVE value. (%lu) \n", calvalue);
		return -EINVAL;
	}

	log("Proximity store Inf Calibration: %lu\n", calvalue);
	if(g_psensor_ATTR->ATTR_Calibration->proximity_store_calibration_inf(calvalue) < 0)
		return -EINVAL;	
	
	return count;
}

ssize_t ATT_proximity_show_adc(struct device *dev, struct device_attribute *attr, char *buf)
{
	int adc = 0;
	if(g_psensor_ATTR->ATTR_Calibration->proximity_show_adc == NULL) {
		err("proximity_show_adc NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	adc = g_psensor_ATTR->ATTR_Calibration->proximity_show_adc();
	return sprintf(buf, "%d\n", adc);
}

/********************/
/*BMMI Function*/
/*******************/
ssize_t  ATT_proximity_show_atd_test(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	bool atd_test = false;
	if(g_psensor_ATTR->ATTR_BMMI->proximity_show_atd_test== NULL) {
		err("proximity_show_atd_test NOT SUPPORT. \n");
		return sprintf(buf, "%d\n", atd_test);
	}
	atd_test = g_psensor_ATTR->ATTR_BMMI->proximity_show_atd_test();
	return sprintf(buf, "%d\n", atd_test);	
}

/************************/
/*Hardware Function*/
/***********************/
ssize_t  ATT_proximity_show_read_reg(struct device *dev, struct device_attribute *attr, char *buf)
{	
	int i2c_reg_addr = 0, i2c_reg_value = 0;
	
	if(g_psensor_ATTR->ATTR_Hardware->proximity_show_reg== NULL) {
		err("IRsensor_store_reg NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}

	i2c_reg_addr = g_psensor_ATTR->ATTR_Hardware->show_reg_addr;
	i2c_reg_value = g_psensor_ATTR->ATTR_Hardware->proximity_show_reg(i2c_reg_addr);
	
	return sprintf(buf, "%d\n", i2c_reg_value);
}

ssize_t  ATT_proximity_store_read_reg(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	int i2c_reg_addr = 0;
	
	if(g_psensor_ATTR->ATTR_Hardware->proximity_show_reg== NULL) {
		err("IRsensor_store_reg NOT SUPPORT. \n");
		return count;
	}

	sscanf(buf, "%x", &i2c_reg_addr);
	g_psensor_ATTR->ATTR_Hardware->show_reg_addr=i2c_reg_addr;
	
	return count;
}
ssize_t  ATT_proximity_store_write_reg(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	int i2c_reg_addr = 0, i2c_reg_value = 0;
	
	if(g_psensor_ATTR->ATTR_Hardware->proximity_store_reg== NULL) {
		err("IRsensor_store_reg NOT SUPPORT. \n");
		return count;
	}
	
	sscanf(buf, "%x %d", &i2c_reg_addr, &i2c_reg_value);

	log("IRsensor_store_reg, addr=%02X, value=%02X\n", i2c_reg_addr, i2c_reg_value);
	if(g_psensor_ATTR->ATTR_Hardware->proximity_store_reg(i2c_reg_addr, i2c_reg_value) < 0)
		return -EINVAL;		
	
	return count;
}

/******************/
/*HAL Function*/
/*****************/
ssize_t  ATT_proximity_show_switch_onoff(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	bool bOn;
	if(g_psensor_ATTR->ATTR_HAL->proximity_show_switch_onoff== NULL) {
		err("proximity_show_switch_onoff NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	bOn = g_psensor_ATTR->ATTR_HAL->proximity_show_switch_onoff();
	if(bOn)
		return sprintf(buf, "on\n");
	else
		return sprintf(buf, "off\n");	
}

ssize_t  ATT_proximity_store_switch_onoff(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	bool bOn;

	if(g_psensor_ATTR->ATTR_HAL->proximity_store_switch_onoff == NULL) {
		err("proximity_store_switch_onoff NOT SUPPORT. \n");
		return count;
	}
	
	/*check input character*/
	if (0 == strncmp(buf, "off", 3))
		bOn = false;
	else if (0 == strncmp(buf, "on", 2)) 
		bOn = true;
	else
		return -EINVAL;

	log("Proximity switch %s\n", bOn?"on":"off");
	if(g_psensor_ATTR->ATTR_HAL->proximity_store_switch_onoff(bOn) < 0)
		return -EINVAL;		
	
	return count;
}

ssize_t ATT_proximity_show_status(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	bool proximity_status;
	if(g_psensor_ATTR->ATTR_HAL->proximity_show_status== NULL) {
		err("proximity_show_status NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	proximity_status = g_psensor_ATTR->ATTR_HAL->proximity_show_status();
	if(proximity_status)
		return sprintf(buf, "close\n");
	else
		return sprintf(buf, "away\n");	
}

/************************/
/*Extension Function*/
/***********************/
ssize_t  ATT_proximity_show_allreg(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	bool ret;
	
	if(g_psensor_ATTR->ATTR_Extension->proximity_show_allreg== NULL) {
		err("IRsensor_show_allreg NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	ret=g_psensor_ATTR->ATTR_Extension->proximity_show_allreg();
	return sprintf(buf, "%d\n", ret);
}

ssize_t  ATT_proximity_show_polling_mode(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	bool bOn;
	if(g_psensor_ATTR->ATTR_Extension->proximity_show_polling_mode == NULL) {
		err("proximity_show_polling_mode NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	bOn = g_psensor_ATTR->ATTR_Extension->proximity_show_polling_mode();
	if(bOn)
		return sprintf(buf, "on\n");
	else
		return sprintf(buf, "off\n");
}

ssize_t  ATT_proximity_store_polling_mode(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	bool bOn;

	if(g_psensor_ATTR->ATTR_Extension->proximity_store_polling_mode== NULL) {
		err("proximity_store_polling_mode NOT SUPPORT. \n");
		return count;
	}
	
	/*check input character*/
	if (0 == strncmp(buf, "off", 3))
		bOn = false;
	else if (0 == strncmp(buf, "on", 2)) 
		bOn = true;
	else
		return -EINVAL;

	log("Proximity polling mode %s\n", bOn?"on":"off");	
	if(g_psensor_ATTR->ATTR_Extension->proximity_store_polling_mode(bOn) < 0)
		return -EINVAL;	
	
	return count;	
}

/* +++ For stress test debug +++ */
ssize_t  ATT_proximity_show_int_count(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int p_int_counter=0;
	
	if(g_psensor_ATTR->ATTR_Extension->proximity_show_int_count == NULL) {
		err("proximity_show_int_count NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	p_int_counter = g_psensor_ATTR->ATTR_Extension->proximity_show_int_count();
	return sprintf(buf, "%d\n", p_int_counter);
}

ssize_t  ATT_proximity_show_event_count(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int p_event_counter=0;
	
	if(g_psensor_ATTR->ATTR_Extension->proximity_show_event_count == NULL) {
		err("proximity_show_event_count NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	p_event_counter = g_psensor_ATTR->ATTR_Extension->proximity_show_event_count();
	return sprintf(buf, "%d\n", p_event_counter);
}

ssize_t  ATT_proximity_show_error_mesg(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int ret;
	char* error_mesg=kzalloc(sizeof(char [ERROR_MESG_SIZE]), GFP_KERNEL);	
	
	if(g_psensor_ATTR->ATTR_Extension->proximity_show_error_mesg== NULL) {
		err("IRsensor_show_error_mesg NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}

	ret = g_psensor_ATTR->ATTR_Extension->proximity_show_error_mesg(error_mesg);
	
	return sprintf(buf, "%s\n", error_mesg);
}
/* --- For stress test debug --- */

/*For auto calibration*/
ssize_t  ATT_proximity_show_autok(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	bool bOn;
	if(g_psensor_ATTR->ATTR_Extension->proximity_show_autok == NULL) {
		err("proximity_show_autok NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	bOn = g_psensor_ATTR->ATTR_Extension->proximity_show_autok();
	if(bOn)
		return sprintf(buf, "on\n");
	else
		return sprintf(buf, "off\n");
}

ssize_t  ATT_proximity_store_autok(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	bool bOn;

	if(g_psensor_ATTR->ATTR_Extension->proximity_store_autok== NULL) {
		err("proximity_store_autok NOT SUPPORT. \n");
		return count;
	}
	
	/*check input character*/
	if (0 == strncmp(buf, "off", 3))
		bOn = false;
	else if (0 == strncmp(buf, "on", 2)) 
		bOn = true;
	else
		return -EINVAL;

	log("Proximity autok %s\n", bOn?"on":"off");	
	if(g_psensor_ATTR->ATTR_Extension->proximity_store_autok(bOn) < 0)
		return -EINVAL;	
	
	return count;	
}

ssize_t  ATT_proximity_show_autokmin(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int autokmin = 0;
	if(g_psensor_ATTR->ATTR_Extension->proximity_show_autokmin== NULL) {
		err("proximity_show_autokmin NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	autokmin = g_psensor_ATTR->ATTR_Extension->proximity_show_autokmin();
	return sprintf(buf, "%d\n", autokmin);
}

ssize_t  ATT_proximity_store_autokmin(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long autokmin;	

	if(g_psensor_ATTR->ATTR_Extension->proximity_store_autokmin== NULL) {
		err("proximity_store_autokmin NOT SUPPORT. \n");
		return count;
	}
	
	if ((kstrtoul(buf, 10, &autokmin) < 0))
		return -EINVAL;
	if(autokmin < 0) {
		err("Proximity store autokmin with NEGATIVE value. (%lu) \n", autokmin);
		return -EINVAL;
	}

	log("Proximity store autokmin: %lu\n", autokmin);
	if(g_psensor_ATTR->ATTR_Extension->proximity_store_autokmin(autokmin) < 0)
		return -EINVAL;	
	
	return count;
}

ssize_t  ATT_proximity_show_autokmax(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int autokmax = 0;
	if(g_psensor_ATTR->ATTR_Extension->proximity_show_autokmax== NULL) {
		err("proximity_show_autokmax NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	autokmax = g_psensor_ATTR->ATTR_Extension->proximity_show_autokmax();
	return sprintf(buf, "%d\n", autokmax);
}

ssize_t  ATT_proximity_store_autokmax(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long autokmax;	

	if(g_psensor_ATTR->ATTR_Extension->proximity_store_autokmax== NULL) {
		err("proximity_store_autokmax NOT SUPPORT. \n");
		return count;
	}
	
	if ((kstrtoul(buf, 10, &autokmax) < 0))
		return -EINVAL;
	if(autokmax < 0) {
		err("Proximity store autokmax with NEGATIVE value. (%lu) \n", autokmax);
		return -EINVAL;
	}

	log("Proximity store autokmax: %lu\n", autokmax);
	if(g_psensor_ATTR->ATTR_Extension->proximity_store_autokmax(autokmax) < 0)
		return -EINVAL;	
	
	return count;
}

/*For transition period from 3/5 to 2/4*/
ssize_t  ATT_proximity_show_selection(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int selection = 0;
	if(g_psensor_ATTR->ATTR_Extension->proximity_show_selection== NULL) {
		err("proximity_show_selection NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	selection = g_psensor_ATTR->ATTR_Extension->proximity_show_selection();
	return sprintf(buf, "%d\n", selection);
}

ssize_t  ATT_proximity_store_selection(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long selection;	

	if(g_psensor_ATTR->ATTR_Extension->proximity_store_selection== NULL) {
		err("proximity_store_selection NOT SUPPORT. \n");
		return count;
	}
	
	if ((kstrtoul(buf, 10, &selection) < 0))
		return -EINVAL;
	if(selection < 0) {
		err("Proximity store selection with NEGATIVE value. (%lu) \n", selection);
		return -EINVAL;
	}

	log("Proximity store selection: %lu\n", selection);
	if(g_psensor_ATTR->ATTR_Extension->proximity_store_selection(selection) < 0)
		return -EINVAL;	
	
	return count;
}

/*For power key turn on screen and enable touch*/
ssize_t  ATT_proximity_show_enable_touch(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int touch_enable = 0;
	if(g_psensor_ATTR->ATTR_Extension->proximity_show_touch_enable== NULL) {
		err("proximity_show_touch_enable NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	touch_enable = g_psensor_ATTR->ATTR_Extension->proximity_show_touch_enable();
	return sprintf(buf, "%d\n", touch_enable);
}

ssize_t  ATT_proximity_store_enable_touch(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long touch_enable;	

	if(g_psensor_ATTR->ATTR_Extension->proximity_store_touch_enable== NULL) {
		err("proximity_store_touch_enable NOT SUPPORT. \n");
		return count;
	}
	
	if ((kstrtoul(buf, 10, &touch_enable) < 0))
		return -EINVAL;
	
	if(touch_enable == 1){
		if(g_psensor_ATTR->ATTR_Extension->proximity_store_touch_enable(true) < 0)
			return -EINVAL;	
	}else if(touch_enable == 0){
		if(g_psensor_ATTR->ATTR_Extension->proximity_store_touch_enable(false) < 0)
			return -EINVAL;	
	}else {
		err("Proximity store touch enable with NEGATIVE value. (%lu) \n", touch_enable);
		return -EINVAL;
	}
	log("Proximity store touch_enable: %lu\n", touch_enable);
	
	return count;
}

/*For load calibration data*/
ssize_t  ATT_proximity_store_load_calibration_data(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long load_calibration_data;	

	if(g_psensor_ATTR->ATTR_Extension->proximity_store_load_calibration_data== NULL) {
		err("proximity_store_load_calibration_data NOT SUPPORT. \n");
		return count;
	}
	
	if ((kstrtoul(buf, 10, &load_calibration_data) < 0))
		return -EINVAL;
	
	if(load_calibration_data == 1){
		if(g_psensor_ATTR->ATTR_Extension->proximity_store_load_calibration_data() < 0)
			return -EINVAL;	
	}else {
		err("Proximity store load calibration data is not 1. (%lu) \n", load_calibration_data);
		return -EINVAL;
	}
	log("Proximity store load_calibration_data: %lu\n", load_calibration_data);
	
	return count;
}

/*For enable anti-oil workaround*/
ssize_t  ATT_proximity_show_anti_oil_enable(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int anti_oil_enable = 0;
	if(g_psensor_ATTR->ATTR_Extension->proximity_show_anti_oil_enable== NULL) {
		err("proximity_show_anti_oil_enable NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	anti_oil_enable = g_psensor_ATTR->ATTR_Extension->proximity_show_anti_oil_enable();
	return sprintf(buf, "%d\n", anti_oil_enable);
}

ssize_t  ATT_proximity_store_anti_oil_enable(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long anti_oil_enable;	

	if(g_psensor_ATTR->ATTR_Extension->proximity_store_anti_oil_enable== NULL) {
		err("proximity_store_anti_oil_enable NOT SUPPORT. \n");
		return count;
	}
	
	if ((kstrtoul(buf, 10, &anti_oil_enable) < 0))
		return -EINVAL;
	
	if(anti_oil_enable == 1){
		if(g_psensor_ATTR->ATTR_Extension->proximity_store_anti_oil_enable(true) < 0)
			return -EINVAL;	
	}else if(anti_oil_enable == 0){
		if(g_psensor_ATTR->ATTR_Extension->proximity_store_anti_oil_enable(false) < 0)
			return -EINVAL;	
	}else {
		err("Proximity store anti-oil enable with NEGATIVE value. (%lu) \n", anti_oil_enable);
		return -EINVAL;
	}
	log("Proximity store anti_oil_enable: %lu\n", anti_oil_enable);
	
	return count;
}

static struct device_attribute proximity_property_attrs[] = {
	/*read only*/
	__ATTR(vendor, 0444, ATT_proximity_show_vendor, NULL),
	__ATTR(module_number, 0444, ATT_proximity_show_module_number, NULL),
	__ATTR(proxm, 0444, ATT_proximity_show_adc, NULL),
	__ATTR(atd_status, 0444, ATT_proximity_show_atd_test, NULL),
	__ATTR(proxm_status, 0444, ATT_proximity_show_status, NULL),	
	__ATTR(dump_reg, 0444, ATT_proximity_show_allreg, NULL),
		/* +++ For stress test debug +++ */
	__ATTR(int_counter, 0444, ATT_proximity_show_int_count, NULL),
	__ATTR(event_counter, 0444, ATT_proximity_show_event_count, NULL),
	__ATTR(error_mesg, 0444, ATT_proximity_show_error_mesg, NULL),
		/* --- For stress test debug --- */
	/*read/write*/
	__ATTR(switch, 0664, ATT_proximity_show_switch_onoff, ATT_proximity_store_switch_onoff),
	__ATTR(hi_cal, 0664, ATT_proximity_show_calibration_hi, ATT_proximity_store_calibration_hi),
	__ATTR(low_cal, 0664, ATT_proximity_show_calibration_lo, ATT_proximity_store_calibration_lo),
	__ATTR(inf_cal, 0664, ATT_proximity_show_calibration_inf, ATT_proximity_store_calibration_inf),
	__ATTR(poll_mode, 0664, ATT_proximity_show_polling_mode, ATT_proximity_store_polling_mode),
	__ATTR(read_reg, 0664, ATT_proximity_show_read_reg, ATT_proximity_store_read_reg),
	__ATTR(write_reg, 0220, NULL, ATT_proximity_store_write_reg),
	__ATTR(autok, 0664, ATT_proximity_show_autok, ATT_proximity_store_autok),
	__ATTR(autokmin, 0664, ATT_proximity_show_autokmin, ATT_proximity_store_autokmin),
	__ATTR(autokmax, 0664, ATT_proximity_show_autokmax, ATT_proximity_store_autokmax),

	/*For transition period from 3/5 to 2/4*/
	__ATTR(selection, 0664, ATT_proximity_show_selection, ATT_proximity_store_selection),
	
	/*For power key turn on screen and enable touch*/
	__ATTR(enable_touch, 0664, ATT_proximity_show_enable_touch, ATT_proximity_store_enable_touch),
	
	/*For load calibration data*/
	__ATTR(load_cal, 0220, NULL, ATT_proximity_store_load_calibration_data),
	
	/*For power key turn on screen and enable touch*/
	__ATTR(enable_anti_oil, 0664, ATT_proximity_show_anti_oil_enable, ATT_proximity_store_anti_oil_enable),
};

int psensor_ATTR_register(psensor_ATTR *mATTR)
{
	int ret = 0;
	int ATTR_index;
	
	g_psensor_ATTR=mATTR;
	
	/* psensor device */
	g_psensor_dev = ASH_ATTR_device_create(psensor);
	if (IS_ERR(g_psensor_dev) || g_psensor_dev == NULL) {
		ret = PTR_ERR(g_psensor_dev);
		err("%s: psensor create ERROR.\n", __FUNCTION__);
		return ret;
	}	
	for (ATTR_index=0; ATTR_index < ARRAY_SIZE(proximity_property_attrs); ATTR_index++) {
		ret = device_create_file(g_psensor_dev, &proximity_property_attrs[ATTR_index]);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL(psensor_ATTR_register);

int psensor_ATTR_unregister(void)
{
	ASH_ATTR_device_remove(psensor);
	return 0;
}
EXPORT_SYMBOL(psensor_ATTR_unregister);

int psensor_ATTR_create(struct device_attribute *mpsensor_attr)
{
	int ret = 0;
	if(mpsensor_attr == NULL) {
		err("%s: the device_attribute is NULL point. \n", __FUNCTION__);
		return -EINVAL;
	}
	ret = device_create_file(g_psensor_dev, mpsensor_attr);
	if (ret){		
		err("%s: device_create_file ERROR(%d). \n", __FUNCTION__, ret);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL(psensor_ATTR_create);

