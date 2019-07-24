/* 
 * Copyright (C) 2014 ASUSTek Inc.
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
/* IR Sensor CM36675 Module */
/******************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/input/ASH.h>
#include "cm36675.h"

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_HW"
#define SENSOR_TYPE_NAME		"IRsensor"

#undef dbg
#ifdef ASH_HW_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

/************************************/
/*Global static variable and function*/
/***********************************/
static struct i2c_client	*g_i2c_client = NULL;
static int cm36675_IRsensor_hw_check_ID(void);
static int cm36675_proximity_hw_set_config(void);
static int cm36675_IRsensor_hw_init(struct i2c_client* client);
static int cm36675_IRsensor_hw_show_allreg(void);
static int cm36675_IRsensor_hw_get_interrupt(void);
static int cm36675_IRsensor_hw_set_register(uint8_t reg, int value);
static int cm36675_IRsensor_hw_get_register(uint8_t reg);
static int cm36675_proximity_hw_turn_onoff(bool bOn);
static int cm36675_proximity_hw_get_adc(void);
static int cm36675_proximity_hw_set_hi_threshold(int hi_threshold);
static int cm36675_proximity_hw_set_lo_threshold(int low_threshold);
static int cm36675_proximity_hw_set_led_current(uint8_t led_current_reg);
static int cm36675_proximity_hw_set_led_duty_ratio(uint8_t led_duty_ratio_reg);
static int cm36675_proximity_hw_set_persistence(uint8_t persistence);
static int cm36675_proximity_hw_set_integration(uint8_t integration);
static int cm36675_proximity_hw_set_autoK(int autok);

/**************************/
/* i2c read/write register */
/*************************/
static int cm36675_IRsensor_hw_check_ID(void)
{
	int ret = 0;
	uint8_t data_buf[2] = {0, 0};	

	/* Check the Device ID */
	ret = i2c_read_reg_u16(g_i2c_client, ID_REG, data_buf);
	if ((ret < 0) || (data_buf[0] != 0x75)) {
		err("%s ERROR(ID_REG : 0x%02X%02X). \n", __FUNCTION__, data_buf[1], data_buf[0]);
		return -ENOMEM;
	}
	log("%s Success(ID_REG : 0x%02X%02X). \n", __FUNCTION__, data_buf[1], data_buf[0]);
	return 0;
}

static int cm36675_proximity_hw_set_config(void)
{
	int ret = 0;	
	ret = cm36675_proximity_hw_set_led_current(CM36675_LED_I_110);
	if (ret < 0)		
		return ret;

	ret = cm36675_proximity_hw_set_led_duty_ratio(CM36675_PS_PERIOD_32);
	if (ret < 0)		
		return ret;

	ret = cm36675_proximity_hw_set_persistence(CM36675_PS_PERS_1);
	if (ret < 0)		
		return ret;

	ret = cm36675_proximity_hw_set_integration(CM36675_PS_IT_2T);
	if (ret < 0)		
		return ret;
		
	return 0;	
}

/*****************/
/*IR Sensor Part*/
/****************/
static int cm36675_IRsensor_hw_init(struct i2c_client* client)
{
	int ret = 0;

	g_i2c_client = client;
	
	/* Check the Device ID 
	 * Do Not return when check ID
	 */
	ret = cm36675_IRsensor_hw_check_ID();
	
	 /*Set Proximity config */
	ret =cm36675_proximity_hw_set_config();
	if (ret < 0) {		
		return ret;
	}

	return 0;
}

static int cm36675_IRsensor_hw_show_allreg(void)
{
	int ret = 0;	
	uint8_t buf[2] = {0};
	int reg = 0;
	for (; reg< CM36675_NUM_REGS; reg++)	
	{
		ret = i2c_read_reg_u16(g_i2c_client, reg, buf);
		log("Show All Register (0x%X) = 0x%02X%02X\n", reg, buf[1], buf[0]);
		if(ret < 0) 
		{
			err("%s: show all Register ERROR. (REG:0x%X)\n", __FUNCTION__, reg);
			return ret;
		}
	}
	return 0;
}

static int cm36675_IRsensor_hw_set_register(uint8_t reg, int value)
{	
	int ret = 0;	
	uint8_t buf[2] = {0};

	buf[1] = value/256;
	buf[0] = value%256;
	
	ret = i2c_write_reg_u16(g_i2c_client, reg, buf);
	log("Set Register Value (0x%X) = 0x%02X%02X\n", reg, buf[1], buf[0]);
	if(ret < 0) 
	{
		err("Set Register Value ERROR. (REG:0x%X)\n", reg);
		return ret;
	}
	
	return 0;
}

static int cm36675_IRsensor_hw_get_register(uint8_t reg)
{
	int ret = 0;	
	uint8_t buf[2] = {0};
	int value;
	
	ret = i2c_read_reg_u16(g_i2c_client, reg, buf);
	log("Get Register Value (0x%X) = 0x%02X%02X\n", reg, buf[1], buf[0]);
	if(ret < 0) 
	{
		err("IR Sensor Get Register Value ERROR. (REG:0x%X)\n", reg);
		return ret;
	}

	value =  buf[1]*256 + buf[0];
	
	return value;
}

static int cm36675_IRsensor_hw_get_interrupt(void)
{
	uint8_t buf[2] = {0};
	bool check_flag = false;
	int irsensor_int = 0;
	
	/* Read INT_FLAG will clean the interrupt */
	i2c_read_reg_u16(g_i2c_client, INT_FLAG, buf);

	/*Proximity Sensor work */
	if (buf[1]&INT_FLAG_PS_IF_AWAY || buf[1]&INT_FLAG_PS_IF_CLOSE) 
	{
		check_flag =true;
		if (buf[1]&INT_FLAG_PS_IF_AWAY) {
			irsensor_int |= IRSENSOR_INT_PS_AWAY;	
		}else if (buf[1]&INT_FLAG_PS_IF_CLOSE) {		
			irsensor_int |= IRSENSOR_INT_PS_CLOSE;
		}else {
			err("Can NOT recognize the Proximity INT_FLAG (0x%02X%02X)\n", buf[1], buf[0]);
			return -1;
		}
	}

	/* Interrupt Error */
	if(check_flag == false){		
		err("Can NOT recognize the INT_FLAG (0x%02X%02X)\n", buf[1], buf[0]);
		return -1;
	}

	return irsensor_int;
}

/****************/
/*Proximity Part*/
/****************/
static int cm36675_proximity_hw_turn_onoff(bool bOn)
{
	int ret = 0;	
	uint8_t power_state_data_buf[2] = {0, 0};
	uint8_t power_state_data_origin[2] = {0, 0};

	/* read power status */
	ret = i2c_read_reg_u16(g_i2c_client, PS_CONF1, power_state_data_buf);
	if (ret < 0) {
		err("Proximity read PS_CONF1 ERROR\n");
		return ret;
	}
	dbg("Proximity read PS_CONF1 (0x%02X%02X) \n", power_state_data_buf[1], power_state_data_buf[0]);
	memcpy(power_state_data_origin, power_state_data_buf, sizeof(power_state_data_buf));
	
	if (bOn == 1)	{	/* power on */		
		power_state_data_buf[0] &= CM36675_PS_SD_MASK;
		power_state_data_buf[1] |= CM36675_PS_INT_IN_AND_OUT;
		
		ret = i2c_write_reg_u16(g_i2c_client, PS_CONF1, power_state_data_buf);
		if (ret < 0) {
			err("Proximity power on ERROR (PS_CONF1) \n");
			return ret;
		} else {
			log("Proximity power on (PS_CONF1 : 0x%X -> 0x%X) \n", 
				power_state_data_origin[0], power_state_data_buf[0]);
		}
	} else	{	/* power off */		
		power_state_data_buf[0] |= CM36675_PS_SD;
	      	power_state_data_buf[1] &= CM36675_PS_INT_MASK;
		
		ret = i2c_write_reg_u16(g_i2c_client, PS_CONF1, power_state_data_buf);
		if (ret < 0) {
			err("Proximity power off ERROR (PS_CONF1) \n");
			return ret;
		} else {
			log("Proximity power off (PS_CONF1 : 0x%X -> 0x%X) \n", 
				power_state_data_origin[0], power_state_data_buf[0]);
		}
	}
	
	return 0;
}

static int cm36675_proximity_hw_get_adc(void)
{
	int ret = 0;
	int adc = 0;
	uint8_t adc_buf[2] = {0, 0};	

	ret = i2c_read_reg_u16(g_i2c_client, PS_DATA, adc_buf);
	if (ret < 0) {
		err("Proximity get adc ERROR. (PS_DATA)\n");
		return ret;
	}
	adc = (adc_buf[1] << 8) + adc_buf[0];
	dbg("Proximity get adc : 0x%02X%02X\n", adc_buf[1], adc_buf[0]); 
	
	return adc;
}

static int cm36675_proximity_hw_set_hi_threshold(int hi_threshold)
{
	int ret = 0;
	uint8_t data_buf[2] = {0, 0};	
	
	/*Set Proximity High Threshold*/
	data_buf[0] = hi_threshold % (1<<8);
	data_buf[1] = hi_threshold /  (1<<8);
	ret = i2c_write_reg_u16(g_i2c_client, PS_THDH, data_buf);
	if(ret < 0) {
		err("Proximity write High Threshold ERROR. (PS_THDH : 0x%02X%02X)\n", data_buf[1], data_buf[0]);
	    	return ret;
	} else {
	    	log("Proximity write High Threshold (PS_THDH : 0x%02X%02X)\n", data_buf[1], data_buf[0]);
	}		

	return 0;
}

static int cm36675_proximity_hw_set_lo_threshold(int low_threshold)
{
	int ret = 0;
	uint8_t data_buf[2] = {0, 0};	

	/*Set Proximity Low Threshold*/	
	data_buf[0] = low_threshold % (1<<8);
	data_buf[1] = low_threshold /  (1<<8);
	ret = i2c_write_reg_u16(g_i2c_client, PS_THDL, data_buf);
	if(ret < 0) {
		err("Proximity write Low Threshold ERROR. (PS_THDL : 0x%02X%02X)\n", data_buf[1], data_buf[0]);
	    	return ret;
	} else {
	    	log("Proximity write Low Threshold (PS_THDL : 0x%02X%02X)\n", data_buf[1], data_buf[0]);
	}

	return 0;
}

static int cm36675_proximity_hw_set_led_current(uint8_t led_current_reg)
{
	int ret = 0;
	uint8_t data_buf[2] = {0, 0};	
	uint8_t data_origin[2] = {0, 0};	

	/* set LED config */
	ret = i2c_read_reg_u16(g_i2c_client, PS_CONF3, data_buf);
	if (ret < 0) {
		err("Proximity read PS_CONF3 ERROR\n");
		return ret;
	}
	dbg("Proximity read PS_CONF3 (0x%02X%02X) \n", data_buf[1], data_buf[0]);
	memcpy(data_origin, data_buf, strlen(data_buf));

	led_current_reg <<= CM36675_LED_I_SHIFT;
	data_buf[1] &= CM36675_LED_I_MASK;
	data_buf[1] |= led_current_reg;
		
	ret = i2c_write_reg_u16(g_i2c_client, PS_CONF3, data_buf);
	if (ret < 0) {
		err("Proximity set LED Current (PS_CONF3) ERROR \n");
		return ret;
	} else {
		log("Proximity set LED Current (PS_CONF3 : 0x%X -> 0x%X) \n", 
			data_origin[1], data_buf[1]);
	}

	return 0;
}

static int cm36675_proximity_hw_set_led_duty_ratio(uint8_t led_duty_ratio_reg)
{
	int ret = 0;	
	uint8_t data_origin[2] = {0, 0};
	uint8_t data_buf[2] = {0, 0};	

	/* set Proximity LED Duty Ratio */
	ret = i2c_read_reg_u16(g_i2c_client, PS_CONF1, data_buf);
	if (ret < 0) {
		err("Proximity read PS_CONF1 ERROR\n");
		return ret;
	}
	dbg("Proximity read PS_CONF1 (0x%02X%02X) \n", data_buf[1], data_buf[0]);
	memcpy(data_origin, data_buf, strlen(data_buf));

	led_duty_ratio_reg <<= CM36675_PS_DR_SHIFT;
	data_buf[0] &= CM36675_PS_DR_MASK;
	data_buf[0] |= led_duty_ratio_reg;

	ret = i2c_write_reg_u16(g_i2c_client, PS_CONF1, data_buf);
	if (ret < 0) {
		err("Proximity set LED Duty Ratio (PS_CONF1) ERROR \n");
		return ret;
	} else {
		log("Proximity set LED Duty Ratio (PS_CONF1 : 0x%X -> 0x%X) \n", 
			data_origin[0], data_buf[0]);
	}

	return 0;	
}

static int cm36675_proximity_hw_set_persistence(uint8_t persistence)
{
	int ret = 0;	
	uint8_t data_origin[2] = {0, 0};
	uint8_t data_buf[2] = {0, 0};	

	/* set Proximity Persistence */
	ret = i2c_read_reg_u16(g_i2c_client, PS_CONF1, data_buf);
	if (ret < 0) {
		err("Proximity read PS_CONF1 ERROR\n");
		return ret;
	}
	dbg("Proximity read PS_CONF1 (0x%02X%02X) \n", data_buf[1], data_buf[0]);
	memcpy(data_origin, data_buf, strlen(data_buf));

	persistence <<= CM36675_PS_PERS_SHIFT;
	data_buf[0] &= CM36675_PS_PERS_MASK;
	data_buf[0] |= persistence;

	ret = i2c_write_reg_u16(g_i2c_client, PS_CONF1, data_buf);
	if (ret < 0) {
		err("Proximity set Persistence (PS_CONF1) ERROR \n");
		return ret;
	} else {
		log("Proximity set Persistence (PS_CONF1 : 0x%X -> 0x%X) \n", 
			data_origin[0], data_buf[0]);
	}

	return 0;
}

static int cm36675_proximity_hw_set_integration(uint8_t integration)
{
	int ret = 0;	
	uint8_t data_origin[2] = {0, 0};
	uint8_t data_buf[2] = {0, 0};	

	/* set Proximity Integration */
	ret = i2c_read_reg_u16(g_i2c_client, PS_CONF1, data_buf);
	if (ret < 0) {
		err("Proximity read PS_CONF1 ERROR\n");
		return ret;
	}
	dbg("Proximity read PS_CONF1 (0x%02X%02X) \n", data_buf[1], data_buf[0]);
	memcpy(data_origin, data_buf, strlen(data_buf));

	integration <<= CM36675_PS_IT_SHIFT;
	data_buf[0] &= CM36675_PS_IT_MASK;
	data_buf[0] |= integration;

	ret = i2c_write_reg_u16(g_i2c_client, PS_CONF1, data_buf);
	if (ret < 0) {
		err("Proximity set Integration (PS_CONF1) ERROR \n");
		return ret;
	} else {
		log("Proximity set Integration (PS_CONF1 : 0x%X -> 0x%X) \n", 
			data_origin[0], data_buf[0]);
	}

	return 0;
}

static int cm36675_proximity_hw_set_autoK(int autok)
{
	int ret = 0;
	uint8_t data_buf[2] = {0, 0};	
	int hi_threshold, low_threshold;

	/*Get High threshold value and adjust*/
	ret = i2c_read_reg_u16(g_i2c_client, PS_THDH, data_buf);
	if (ret < 0) {
		err("Proximity get High Threshold ERROR. (PS_THDH)\n");
		return ret;
	}
	hi_threshold = (data_buf[1] << 8) + data_buf[0];
	dbg("Proximity get High Threshold : 0x%02X%02X\n", data_buf[1], data_buf[0]); 	
	cm36675_proximity_hw_set_hi_threshold(hi_threshold + autok);

	/*Get Low threshold value and adjust*/
	ret = i2c_read_reg_u16(g_i2c_client, PS_THDL, data_buf);
	if (ret < 0) {
		err("Proximity get Low Threshold ERROR. (PS_THDL)\n");
		return ret;
	}
	low_threshold = (data_buf[1] << 8) + data_buf[0];
	dbg("Proximity get Low Threshold : 0x%02X%02X\n", data_buf[1], data_buf[0]); 
	
	cm36675_proximity_hw_set_lo_threshold(low_threshold + autok);

	return 0;
}

static struct psensor_hw psensor_hw_cm36675 = {
	.proximity_low_threshold_default = CM36675_PROXIMITY_THDL_DEFAULT,
	.proximity_hi_threshold_default = CM36675_PROXIMITY_THDH_DEFAULT,
	.proximity_crosstalk_default = CM36675_PROXIMITY_INF_DEFAULT,
	.proximity_autok_min = CM36675_PROXIMITY_AUTOK_MIN,
	.proximity_autok_max = CM36675_PROXIMITY_AUTOK_MAX,
	
	.proximity_hw_turn_onoff = cm36675_proximity_hw_turn_onoff,
	.proximity_hw_get_adc = cm36675_proximity_hw_get_adc,
	.proximity_hw_set_hi_threshold = cm36675_proximity_hw_set_hi_threshold,
	.proximity_hw_set_lo_threshold = cm36675_proximity_hw_set_lo_threshold,
	.proximity_hw_set_autoK = cm36675_proximity_hw_set_autoK,
};

static struct IRsensor_hw IRsensor_hw_cm36675 = {	
	.vendor = "Capella",
	.module_number = "cm36675",

	.IRsensor_hw_check_ID = cm36675_IRsensor_hw_check_ID,
	.IRsensor_hw_init = cm36675_IRsensor_hw_init,
	.IRsensor_hw_get_interrupt = cm36675_IRsensor_hw_get_interrupt,
	.IRsensor_hw_show_allreg = cm36675_IRsensor_hw_show_allreg,
	.IRsensor_hw_set_register = cm36675_IRsensor_hw_set_register,
	.IRsensor_hw_get_register = cm36675_IRsensor_hw_get_register,

	.mpsensor_hw = &psensor_hw_cm36675,
};

IRsensor_hw* Psensor_hw_cm36675_getHardware(void)
{
	IRsensor_hw* Psensor_hw_client = NULL;
	Psensor_hw_client = &IRsensor_hw_cm36675;
	return Psensor_hw_client;
}
