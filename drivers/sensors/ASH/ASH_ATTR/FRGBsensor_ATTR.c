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

 /***********************************/
/* Front RGB Sensor Atrribute */
/**********************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/input/ASH.h>

#define BUF_SIZE	(10)
FRGBsensor_ATTR *g_FRGB_ATTR = NULL;
struct device *g_FRGB_dev;

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_ATTR"
#define SENSOR_TYPE_NAME	"FRGB"


#undef dbg
#ifdef ASH_ATTR_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

ssize_t  ATT_FRGB_show_vendor(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	if(strcmp(g_FRGB_ATTR->info_type->vendor, "") == 0) {
		err("Show vendor NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	return sprintf(buf, "%s\n", g_FRGB_ATTR->info_type->vendor);
}

ssize_t  ATT_FRGB_show_module_number(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	if(strcmp(g_FRGB_ATTR->info_type->module_number, "") == 0) {
		err("Show module number NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	return sprintf(buf, "%s\n", g_FRGB_ATTR->info_type->module_number);
}

/*************************/
/*Calibration Function*/
/************************/
ssize_t ATT_FRGB_show_red(struct device *dev, struct device_attribute *attr, char *buf)
{
	int adc = 0;
	if(g_FRGB_ATTR->ATTR_Calibration->FRGB_show_red == NULL) {
		err("FRGB_show_red NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	adc = g_FRGB_ATTR->ATTR_Calibration->FRGB_show_red();
	return sprintf(buf, "%d\n", adc);
}

ssize_t ATT_FRGB_show_green(struct device *dev, struct device_attribute *attr, char *buf)
{
	int adc = 0;
	if(g_FRGB_ATTR->ATTR_Calibration->FRGB_show_green == NULL) {
		err("FRGB_show_green NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	adc = g_FRGB_ATTR->ATTR_Calibration->FRGB_show_green();
	return sprintf(buf, "%d\n", adc);
}

ssize_t ATT_FRGB_show_blue(struct device *dev, struct device_attribute *attr, char *buf)
{
	int adc = 0;
	if(g_FRGB_ATTR->ATTR_Calibration->FRGB_show_blue == NULL) {
		err("FRGB_show_blue NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	adc = g_FRGB_ATTR->ATTR_Calibration->FRGB_show_blue();
	return sprintf(buf, "%d\n", adc);
}

ssize_t ATT_FRGB_show_ir(struct device *dev, struct device_attribute *attr, char *buf)
{
	int adc = 0;
	if(g_FRGB_ATTR->ATTR_Calibration->FRGB_show_ir == NULL) {
		err("FRGB_show_IR NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	adc = g_FRGB_ATTR->ATTR_Calibration->FRGB_show_ir();
	return sprintf(buf, "%d\n", adc);
}

/********************/
/*BMMI Function*/
/******************/
ssize_t  ATT_FRGB_show_atd_test(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	bool atd_test = false;
	if(g_FRGB_ATTR->ATTR_BMMI->FRGB_show_atd_test== NULL) {
		err("FRGB_show_atd_test NOT SUPPORT. \n");
		return sprintf(buf, "%d\n", atd_test);
	}
	atd_test = g_FRGB_ATTR->ATTR_BMMI->FRGB_show_atd_test();
	return sprintf(buf, "%d\n", atd_test);	
}

/************************/
/*Hardware Function*/
/***********************/
ssize_t  ATT_FRGB_show_read_reg(struct device *dev, struct device_attribute *attr, char *buf)
{	
	int i2c_reg_addr = 0, i2c_reg_value = 0;
	
	if(g_FRGB_ATTR->ATTR_Hardware->FRGB_show_reg== NULL) {
		err("FRGB_show_reg NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}

	i2c_reg_addr = g_FRGB_ATTR->ATTR_Hardware->show_reg_addr;
	i2c_reg_value = g_FRGB_ATTR->ATTR_Hardware->FRGB_show_reg(i2c_reg_addr);
	
	return sprintf(buf, "%d\n", i2c_reg_value);
}

ssize_t  ATT_FRGB_store_read_reg(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	int i2c_reg_addr = 0;
	
	if(g_FRGB_ATTR->ATTR_Hardware->FRGB_show_reg== NULL) {
		err("FRGB_show_reg NOT SUPPORT. \n");
		return count;
	}

	sscanf(buf, "%x", &i2c_reg_addr);
	g_FRGB_ATTR->ATTR_Hardware->show_reg_addr=i2c_reg_addr;
	
	return count;
}
ssize_t  ATT_FRGB_store_write_reg(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	int i2c_reg_addr = 0, i2c_reg_value = 0;
	
	if(g_FRGB_ATTR->ATTR_Hardware->FRGB_store_reg== NULL) {
		err("FRGB_store_reg NOT SUPPORT. \n");
		return count;
	}
	
	sscanf(buf, "%x %d", &i2c_reg_addr, &i2c_reg_value);

	log("FRGB_store_reg, addr=%02X, value=%02X\n", i2c_reg_addr, i2c_reg_value);
	if(g_FRGB_ATTR->ATTR_Hardware->FRGB_store_reg(i2c_reg_addr, i2c_reg_value) < 0)
		return -EINVAL;		
	
	return count;
}

/******************/
/*HAL Function*/
/*****************/
ssize_t  ATT_FRGB_show_switch_onoff(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	bool bOn;
	if(g_FRGB_ATTR->ATTR_HAL->FRGB_show_switch_onoff== NULL) {
		err("FRGB_show_switch_onoff NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	bOn = g_FRGB_ATTR->ATTR_HAL->FRGB_show_switch_onoff();
	if(bOn)
		return sprintf(buf, "on\n");
	else
		return sprintf(buf, "off\n");	
}

ssize_t  ATT_FRGB_store_switch_onoff(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	bool bOn;

	if(g_FRGB_ATTR->ATTR_HAL->FRGB_store_switch_onoff == NULL) {
		err("FRGB_store_switch_onoff NOT SUPPORT. \n");
		return count;
	}
	
	/*check input character*/
	if (0 == strncmp(buf, "off", 3))
		bOn = false;
	else if (0 == strncmp(buf, "on", 2)) 
		bOn = true;
	else
		return -EINVAL;

	log("FRGB switch %s\n", bOn?"on":"off");
	if(g_FRGB_ATTR->ATTR_HAL->FRGB_store_switch_onoff(bOn) < 0)
		return -EINVAL;		
	
	return count;
}

/*********************/
/*Extension Function*/
/********************/
ssize_t  ATT_FRGB_show_allreg(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	bool ret;
	
	if(g_FRGB_ATTR->ATTR_Extension->FRGB_show_allreg== NULL) {
		err("FRGB_show_allreg NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	ret=g_FRGB_ATTR->ATTR_Extension->FRGB_show_allreg();
	return sprintf(buf, "%d\n", ret);
}

ssize_t  ATT_FRGB_show_log_threshold(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int log_threshold = 0;
	if(g_FRGB_ATTR->ATTR_Extension->FRGB_show_log_threshold == NULL) {
		err("FRGB_show_log_threshold NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	log_threshold = g_FRGB_ATTR->ATTR_Extension->FRGB_show_log_threshold();
	return sprintf(buf, "%d\n", log_threshold);
}

ssize_t  ATT_FRGB_store_log_threshold(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long log_threshold;	

	if(g_FRGB_ATTR->ATTR_Extension->FRGB_store_log_threshold == NULL) {
		err("FRGB_store_log_threshold NOT SUPPORT. \n");
		return count;
	}
	
	if ((kstrtoul(buf, 10, &log_threshold) < 0))
		return -EINVAL;
	if(log_threshold < 0) {
		err("FRGB store Log Threshold with NEGATIVE value. (%lu) \n", log_threshold);
		return -EINVAL;
	}

	log("FRGB store Log Threshold: %lu\n", log_threshold);
	if(g_FRGB_ATTR->ATTR_Extension->FRGB_store_log_threshold(log_threshold) < 0)
		return -EINVAL;	
	
	return count;
}

/* +++ For stress test debug +++ */
ssize_t  ATT_FRGB_show_error_mesg(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int ret;
	char* error_mesg=kzalloc(sizeof(char [ERROR_MESG_SIZE]), GFP_KERNEL);	
	
	if(g_FRGB_ATTR->ATTR_Extension->FRGB_show_error_mesg== NULL) {
		err("FRGB_show_error_mesg NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}

	ret = g_FRGB_ATTR->ATTR_Extension->FRGB_show_error_mesg(error_mesg);
	
	return sprintf(buf, "%s\n", error_mesg);
}
/* --- For stress test debug --- */

static struct device_attribute frgb_property_attrs[] = {
	/*read only*/
	__ATTR(vendor, 0444, ATT_FRGB_show_vendor, NULL),
	__ATTR(module_number, 0444, ATT_FRGB_show_module_number, NULL),
	__ATTR(red, 0444, ATT_FRGB_show_red, NULL),
	__ATTR(green, 0444, ATT_FRGB_show_green, NULL),
	__ATTR(blue, 0444, ATT_FRGB_show_blue, NULL),
	__ATTR(ir, 0444, ATT_FRGB_show_ir, NULL),
	__ATTR(atd_status, 0444, ATT_FRGB_show_atd_test, NULL),
	__ATTR(dump_reg, 0444, ATT_FRGB_show_allreg, NULL),
		/* +++ For stress test debug +++ */
	__ATTR(error_mesg, 0444, ATT_FRGB_show_error_mesg, NULL),
		/* --- For stress test debug --- */
	/*read/write*/
	__ATTR(switch, 0664, ATT_FRGB_show_switch_onoff, ATT_FRGB_store_switch_onoff),
	__ATTR(read_reg, 0664, ATT_FRGB_show_read_reg, ATT_FRGB_store_read_reg),
	__ATTR(write_reg, 0220, NULL, ATT_FRGB_store_write_reg),
	__ATTR(log_threshold, 0664, ATT_FRGB_show_log_threshold, ATT_FRGB_store_log_threshold),
};

int FRGBsensor_ATTR_register(FRGBsensor_ATTR *mATTR)
{
	int ret = 0;
	int ATTR_index;
	
	g_FRGB_ATTR=mATTR;
	
	/* psensor device */
	g_FRGB_dev = ASH_ATTR_device_create(frgbsensor);
	if (IS_ERR(g_FRGB_dev) || g_FRGB_dev == NULL) {
		ret = PTR_ERR(g_FRGB_dev);
		err("%s: FRGB sensor create ERROR.\n", __FUNCTION__);
		return ret;
	}	
	for (ATTR_index=0; ATTR_index < ARRAY_SIZE(frgb_property_attrs); ATTR_index++) {
		ret = device_create_file(g_FRGB_dev, &frgb_property_attrs[ATTR_index]);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL(FRGBsensor_ATTR_register);

int FRGBsensor_ATTR_unregister(void)
{
	ASH_ATTR_device_remove(frgbsensor);
	return 0;
}
EXPORT_SYMBOL(FRGBsensor_ATTR_unregister);

int FRGBsensor_ATTR_create(struct device_attribute *mfrgbsensor_attr)
{
	int ret = 0;
	if(mfrgbsensor_attr == NULL) {
		err("%s: the device_attribute is NULL point. \n", __FUNCTION__);
		return -EINVAL;
	}
	ret = device_create_file(g_FRGB_dev, mfrgbsensor_attr);
	if (ret){		
		err("%s: device_create_file ERROR(%d). \n", __FUNCTION__, ret);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL(FRGBsensor_ATTR_create);
