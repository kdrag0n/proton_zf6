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

 /***********************************/
/* Front SAR Sensor Atrribute */
/**********************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/input/ASH.h>

#define BUF_SIZE	(10)
SAR_sensor_ATTR *g_sar_sensor_ATTR = NULL;
struct device *g_sar_sensor_dev;

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_ATTR"
#define SENSOR_TYPE_NAME	"SAR"


#undef dbg
#ifdef ASH_ATTR_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

ssize_t ATT_SAR_sensor_show_vendor(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if(strcmp(g_sar_sensor_ATTR->info_type->vendor, "") == 0) {
		err("Show vendor NOT SUPPORT.\n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	return sprintf(buf, "%s\n", g_sar_sensor_ATTR->info_type->vendor);
}

ssize_t ATT_SAR_sensor_show_module_number(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	if(strcmp(g_sar_sensor_ATTR->info_type->module_number, "") == 0) {
		err("Show module number NOT SUPPORT.\n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	return sprintf(buf, "%s\n", g_sar_sensor_ATTR->info_type->module_number);
}

/********************/
/*BMMI Function*/
/******************/
ssize_t ATT_SAR_sensor_show_atd_test(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	bool atd_test = false;
	if(g_sar_sensor_ATTR->ATTR_BMMI->SAR_sensor_show_atd_test== NULL) {
		err("SAR_sensor_show_atd_test NOT SUPPORT.\n");
		return sprintf(buf, "%d\n", atd_test);
	}
	atd_test = g_sar_sensor_ATTR->ATTR_BMMI->SAR_sensor_show_atd_test();
	return sprintf(buf, "%d\n", atd_test);
}

ssize_t ATT_SAR_sensor_show_raw_data(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret = 0;
	if(g_sar_sensor_ATTR->ATTR_BMMI->SAR_sensor_show_raw_data == NULL) {
		err("SAR_sensor_show_raw_data NOT SUPPORT.\n");
		return sprintf(buf, "SAR_sensor_show_raw_data NOT SUPPORT.\n");
	}
	ret = g_sar_sensor_ATTR->ATTR_BMMI->SAR_sensor_show_raw_data();
	if(ret < 0) {
		err("Dump raw data Fail (ret = %d)\n", ret);
		return sprintf(buf, "Dump raw data Fail (ret = %d)\n", ret);
	}
	return sprintf(buf, "done\n");
}
/************************/
/*Hardware Function*/
/***********************/
ssize_t ATT_SAR_show_read_reg(struct device *dev, 
	struct device_attribute *attr, char *buf)
{	
	int i2c_reg_addr = 0, i2c_reg_value = 0;
	
	if(g_sar_sensor_ATTR->ATTR_Hardware->SAR_show_reg == NULL) {
		err("SAR_show_reg NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}

	i2c_reg_addr = g_sar_sensor_ATTR->ATTR_Hardware->show_reg_addr;
	i2c_reg_value = g_sar_sensor_ATTR->ATTR_Hardware->SAR_show_reg(i2c_reg_addr);
	
	return sprintf(buf, "%d\n", i2c_reg_value);
}

ssize_t ATT_SAR_store_read_reg(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	int i2c_reg_addr = 0;
	
	if(g_sar_sensor_ATTR->ATTR_Hardware->SAR_show_reg == NULL) {
		err("SAR_show_reg NOT SUPPORT. \n");
		return count;
	}

	sscanf(buf, "%x", &i2c_reg_addr);
	g_sar_sensor_ATTR->ATTR_Hardware->show_reg_addr=i2c_reg_addr;
	
	return count;
}

ssize_t ATT_SAR_store_write_reg(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	int i2c_reg_addr = 0, i2c_reg_value = 0;
	
	if(g_sar_sensor_ATTR->ATTR_Hardware->SAR_store_reg == NULL) {
		err("SAR_store_reg NOT SUPPORT. \n");
		return count;
	}
	
	sscanf(buf, "%x %x", &i2c_reg_addr, &i2c_reg_value);

	log("SAR_store_reg, addr=%02X, value=%02X\n", i2c_reg_addr, i2c_reg_value);
	if(g_sar_sensor_ATTR->ATTR_Hardware->SAR_store_reg(i2c_reg_addr, i2c_reg_value) < 0)
		return -EINVAL;		
	
	return count;
}

ssize_t ATT_SAR_sensor_show_manual_offset_cal(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int value = 0;
	
	if(g_sar_sensor_ATTR->ATTR_Hardware->SAR_sensor_show_manual_offset_cal == NULL) {
		err("SAR_sensor_show_manual_offset_cal NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	value = g_sar_sensor_ATTR->ATTR_Hardware->SAR_sensor_show_manual_offset_cal();

	return sprintf(buf, "0x%x\n", value);
}

ssize_t ATT_SAR_store_manual_offset_cal(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	int value = 0;
	
	if(g_sar_sensor_ATTR->ATTR_Hardware->SAR_sensor_store_manual_offset_cal == NULL) {
		err("SAR_sensor_store_manual_offset_calibration NOT SUPPORT. \n");
		return count;
	}
	
	sscanf(buf, "%d", &value);

	log("SAR_sensor_store_manual_offset_cal, value=%d\n", value);
	if(g_sar_sensor_ATTR->ATTR_Hardware->SAR_sensor_store_manual_offset_cal(value) < 0){
		return -EINVAL;
	}
	return count;
}

/******************/
/*HAL Function*/
/*****************/
ssize_t ATT_SAR_sensor_show_switch_onoff(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	bool bOn;
	if(g_sar_sensor_ATTR->ATTR_HAL->SAR_sensor_show_switch_onoff == NULL) {
		err("SAR_sensor_show_switch_onoff NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	bOn = g_sar_sensor_ATTR->ATTR_HAL->SAR_sensor_show_switch_onoff();
	if (bOn)
		return sprintf(buf, "on\n");
	else
		return sprintf(buf, "off\n");	
}

ssize_t ATT_SAR_sensor_store_switch_onoff(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	bool bOn;

	if(g_sar_sensor_ATTR->ATTR_HAL->SAR_sensor_store_switch_onoff == NULL) {
		err("SAR_store_switch_onoff NOT SUPPORT. \n");
		return count;
	}
	
	/*check input character*/
	if (0 == strncmp(buf, "off", 3))
		bOn = false;
	else if (0 == strncmp(buf, "on", 2)) 
		bOn = true;
	else
		return -EINVAL;

	log("SAR switch %s\n", bOn?"on":"off");
	if(g_sar_sensor_ATTR->ATTR_HAL->SAR_sensor_store_switch_onoff(bOn) < 0)
		return -EINVAL;		
	
	return count;
}

ssize_t ATT_SAR_sensor_show_Interrupt_detect_status(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int status = 0;
	if(g_sar_sensor_ATTR->ATTR_HAL->SAR_sensor_show_Interrupt_detect_status == NULL) {
		err("SAR_sensor_show_Interrupt_detect_status NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	status = g_sar_sensor_ATTR->ATTR_HAL->SAR_sensor_show_Interrupt_detect_status();
	if(status < 0) {
		err("SAR_sensor_show_Interrupt_detect_status get status fail !\n");
		return sprintf(buf, "Fail to get interrupt detect status!\n");
	} else	{
		/* Near = 1, Far = 0 */
		return sprintf(buf, "%d\n", status);
	}
}

ssize_t ATT_SAR_sensor_show_sar_status(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	bool status = 0;
	if(g_sar_sensor_ATTR->ATTR_HAL->SAR_sensor_show_sar_status == NULL) {
		err("SAR_sensor_show_sar_status NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	status = g_sar_sensor_ATTR->ATTR_HAL->SAR_sensor_show_sar_status();
	if(status)
		return sprintf(buf, "1\n");
	else
		return sprintf(buf, "0\n");
}

/*************************/
/*Extension Function*/
/*************************/
ssize_t ATT_SAR_sensor_show_allreg(struct device *dev,struct device_attribute *attr, char *buf)
{
	if(g_sar_sensor_ATTR->ATTR_Extension->SAR_sensor_show_allreg == NULL) {
		err("SAR_show_allreg NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	g_sar_sensor_ATTR->ATTR_Extension->SAR_sensor_show_allreg();
	return sprintf(buf, "done\n");
}

static struct device_attribute sar_sensor_property_attrs[] = {
	/*read only*/
	__ATTR(vendor, 0444, ATT_SAR_sensor_show_vendor, NULL),
	__ATTR(module_number, 0444, ATT_SAR_sensor_show_module_number, NULL),
	__ATTR(atd_status, 0444, ATT_SAR_sensor_show_atd_test, NULL),
	__ATTR(raw_data, 0444, ATT_SAR_sensor_show_raw_data, NULL),
	__ATTR(dump_reg, 0444, ATT_SAR_sensor_show_allreg, NULL),
	__ATTR(Interrupt_detect_status, 0444, ATT_SAR_sensor_show_Interrupt_detect_status, NULL),
	__ATTR(sar_status, 0444, ATT_SAR_sensor_show_sar_status, NULL),
	/*read/write*/
	__ATTR(switch, 0664, ATT_SAR_sensor_show_switch_onoff, ATT_SAR_sensor_store_switch_onoff),
	__ATTR(read_reg, 0664, ATT_SAR_show_read_reg, ATT_SAR_store_read_reg),
	__ATTR(manual_offset_calibration, 0664, ATT_SAR_sensor_show_manual_offset_cal, ATT_SAR_store_manual_offset_cal),
	__ATTR(write_reg, 0220, NULL, ATT_SAR_store_write_reg),
};

int SAR_sensor_ATTR_register(SAR_sensor_ATTR *mATTR)
{
	int ret = 0;
	int ATTR_index;
	g_sar_sensor_ATTR = mATTR;
	/* SAR sensor device */
	g_sar_sensor_dev = ASH_ATTR_device_create(sarsensor);
	if(IS_ERR(g_sar_sensor_dev) || g_sar_sensor_dev == NULL) {
		ret = PTR_ERR(g_sar_sensor_dev);
		err("%s: SAR sensor create ERROR.\n", __FUNCTION__);
		return ret;
	}	
	for(ATTR_index = 0; ATTR_index < ARRAY_SIZE(sar_sensor_property_attrs); ATTR_index++) {
		ret = device_create_file(g_sar_sensor_dev, &sar_sensor_property_attrs[ATTR_index]);
		if (ret)
			return ret;
	}
	return 0;
}
EXPORT_SYMBOL(SAR_sensor_ATTR_register);

int SAR_sensor_ATTR_unregister(void)
{
	ASH_ATTR_device_remove(sarsensor);
	return 0;
}
EXPORT_SYMBOL(SAR_sensor_ATTR_unregister);

int SAR_sensor_ATTR_create(struct device_attribute *mSAR_sensor_attr)
{
	int ret = 0;
	if(mSAR_sensor_attr == NULL) {
		err("%s: the device_attribute is NULL point. \n", __FUNCTION__);
		return -EINVAL;
	}
	ret = device_create_file(g_sar_sensor_dev, mSAR_sensor_attr);
	if (ret){		
		err("%s: device_create_file ERROR(%d). \n", __FUNCTION__, ret);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL(SAR_sensor_ATTR_create);
