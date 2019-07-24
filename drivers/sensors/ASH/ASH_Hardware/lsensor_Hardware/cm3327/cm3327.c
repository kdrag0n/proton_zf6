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
/* Light Sensor CM3327 Module */
/********************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/input/ASH.h>
#include "cm3327.h"

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_HW"
#define SENSOR_TYPE_NAME	"Lsensor"

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
static int cm3327_IRsensor_hw_check_ID(void);
static int cm3327_light_hw_set_config(void);

static int cm3327_IRsensor_hw_init(struct i2c_client* client);
static int cm3327_IRsensor_hw_show_allreg(void);
static int cm3327_IRsensor_hw_get_interrupt(void);
static int cm3327_IRsensor_hw_set_register(uint8_t reg, int value);
static int cm3327_IRsensor_hw_get_register(uint8_t reg);

static int cm3327_light_hw_turn_onoff(bool bOn);
static int cm3327_light_hw_get_r(void);
static int cm3327_light_hw_get_g(void);
static int cm3327_light_hw_get_b(void);
static int cm3327_light_hw_get_ir(void);
static int cm3327_light_hw_set_hi_threshold(int hi_threshold);
static int cm3327_light_hw_set_lo_threshold(int low_threshold);
static int cm3327_light_hw_set_persistence(uint8_t persistence);
static int cm3327_light_hw_set_integration(uint8_t integration);

/**************************/
/* i2c read/write register */
/*************************/
static int cm3327_IRsensor_hw_check_ID(void)
{
	int ret = 0;
	uint8_t data_buf[2] = {0, 0};	

	/* Check the Device ID */
	ret = i2c_read_reg_u16(g_i2c_client, ID_REG, data_buf);
	if ((ret < 0) || (data_buf[0] != 0x93)) {
		err("%s ERROR(ID_REG : 0x%02X%02X). \n", __FUNCTION__, data_buf[1], data_buf[0]);
		return -ENOMEM;
	}
	log("%s Success(ID_REG : 0x%02X%02X). \n", __FUNCTION__, data_buf[1], data_buf[0]);
	return 0;
}

static int cm3327_light_hw_set_config(void)
{
	int ret = 0;	
	ret = cm3327_light_hw_set_persistence(CM3327_ALS_PERS_1);
	if (ret < 0)		
		return ret;

	ret = cm3327_light_hw_set_integration(CM3327_RGBIR_IT_120MS);
	if (ret < 0)		
		return ret;

	return 0;	
}

/*****************/
/*IR Sensor Part*/
/****************/
static int cm3327_IRsensor_hw_init(struct i2c_client* client)
{
	int ret = 0;

	g_i2c_client = client;
	
	/* Check the Device ID 
	 * Do Not return when check ID
	 */
	ret = cm3327_IRsensor_hw_check_ID();
	
	 /*Set Light Sensor config*/
	ret =cm3327_light_hw_set_config();
	if (ret < 0) {		
		return ret;
	}
	
	return 0;
}

static int cm3327_IRsensor_hw_show_allreg(void)
{
	int ret = 0;	
	uint8_t buf[2] = {0};
	int reg = 0;
	for (; reg< CM3327_NUM_REGS; reg++)	
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

static int cm3327_IRsensor_hw_set_register(uint8_t reg, int value)
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

static int cm3327_IRsensor_hw_get_register(uint8_t reg)
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

static int cm3327_IRsensor_hw_get_interrupt(void)
{
	uint8_t buf[2] = {0};
	bool check_flag = false;
	int irsensor_int = 0;
	
	/* Read INT_FLAG will clean the interrupt */
	i2c_read_reg_u16(g_i2c_client, INT_FLAG, buf);

	/* Light Sensor work */
	if(buf[1]&INT_FLAG_ALS_IF_L || buf[1]&INT_FLAG_ALS_IF_H) 
	{	
		check_flag = true;
		irsensor_int |= IRSENSOR_INT_ALS;
	} 

	/* Interrupt Error */
	if(check_flag == false){		
		err("Can NOT recognize the INT_FLAG (0x%02X%02X)\n", buf[1], buf[0]);
		return -1;
	}

	return irsensor_int;
}

/*********************/
/* Light Sensor Part */
/********************/
static int cm3327_light_hw_turn_onoff(bool bOn)
{
	int ret = 0;	
	uint8_t power_state_data_buf[2] = {0, 0};
	uint8_t power_state_data_origin[2] = {0, 0};

	/* read power status */
	ret = i2c_read_reg_u16(g_i2c_client, ALS_CONF, power_state_data_buf);	
	if (ret < 0) {
		err("Light Sensor read ALS_CONF ERROR\n");
		return ret;
	}
	dbg("Light Sensor read ALS_CONF (0x%02X%02X) \n", power_state_data_buf[1], power_state_data_buf[0]);
	
	memcpy(power_state_data_origin, power_state_data_buf, sizeof(power_state_data_buf));

	if (bOn == 1)	{	/* power on */			
		power_state_data_buf[0] &= CM3327_ALS_SD_MASK;
		power_state_data_buf[1] |= CM3327_ALS_INT_EN;

		ret = i2c_write_reg_u16(g_i2c_client, ALS_CONF, power_state_data_buf);
		if (ret < 0) {
			err("Light Sensor power on ERROR (ALS_CONF) \n");
			return ret;
		} else {
			log("Light Sensor power on (ALS_CONF : 0x%X -> 0x%X) \n", 
				power_state_data_origin[0], power_state_data_buf[0]);
		}		
	} else {	/* power off */	
		power_state_data_buf[0] |= CM3327_ALS_SD;
		power_state_data_buf[1] &= CM3327_ALS_INT_MASK;
		
		ret = i2c_write_reg_u16(g_i2c_client, ALS_CONF, power_state_data_buf);
		if (ret < 0) {
			err("Light Sensor power off ERROR (ALS_CONF) \n");
			return ret;
		} else {
			log("Light Sensor power off (ALS_CONF : 0x%X -> 0x%X) \n", 
				power_state_data_origin[0], power_state_data_buf[0]);
		}
	}	

	return 0;
}

static int cm3327_light_hw_get_r(void){
	int ret = 0;
	int adc = 0;
	uint8_t adc_buf[2] = {0, 0};	
	
	ret = i2c_read_reg_u16(g_i2c_client, R_DATA, adc_buf);
	if (ret < 0) {
		err("Light Sensor Get adc ERROR (ALS_DATA)\n");
		return ret;
	}
	adc = (adc_buf[1] << 8) + adc_buf[0];
	dbg("Light Sensor Get adc : 0x%02X%02X\n", adc_buf[1], adc_buf[0]); 
	
	return adc;
}

static int cm3327_light_hw_get_g(void)
{
	int ret = 0;
	int adc = 0;
	uint8_t adc_buf[2] = {0, 0};	
	
	ret = i2c_read_reg_u16(g_i2c_client, G_DATA, adc_buf);
	if (ret < 0) {
		err("Light Sensor Get adc ERROR (ALS_DATA)\n");
		return ret;
	}
	adc = (adc_buf[1] << 8) + adc_buf[0];
	dbg("Light Sensor Get adc : 0x%02X%02X\n", adc_buf[1], adc_buf[0]); 
	
	return adc;
}

static int cm3327_light_hw_get_b(void)
{
	int ret = 0;
	int adc = 0;
	uint8_t adc_buf[2] = {0, 0};	
	
	ret = i2c_read_reg_u16(g_i2c_client, B_DATA, adc_buf);
	if (ret < 0) {
		err("Light Sensor Get adc ERROR (ALS_DATA)\n");
		return ret;
	}
	adc = (adc_buf[1] << 8) + adc_buf[0];
	dbg("Light Sensor Get adc : 0x%02X%02X\n", adc_buf[1], adc_buf[0]); 
	
	return adc;
}

static int cm3327_light_hw_get_ir(void)
{
	int ret = 0;
	int adc = 0;
	uint8_t adc_buf[2] = {0, 0};	
	
	ret = i2c_read_reg_u16(g_i2c_client, IR_DATA, adc_buf);
	if (ret < 0) {
		err("Light Sensor Get adc ERROR (ALS_DATA)\n");
		return ret;
	}
	adc = (adc_buf[1] << 8) + adc_buf[0];
	dbg("Light Sensor Get adc : 0x%02X%02X\n", adc_buf[1], adc_buf[0]); 
	
	return adc;
}

static int cm3327_light_hw_set_hi_threshold(int hi_threshold)
{
	int ret = 0;	
	uint8_t data_buf[2] = {0, 0};	

	/*Set Light Sensor High Threshold*/	
	data_buf[0] = hi_threshold % (1<<8);
	data_buf[1] = hi_threshold /  (1<<8);
	ret = i2c_write_reg_u16(g_i2c_client, ALS_THDH, data_buf);
	if(ret < 0) {
		err("[i2c] Light Sensor write High Threshold ERROR. (ALS_THDH : 0x%02X%02X)\n", data_buf[1], data_buf[0]);
	    	return ret;
	} else {
	    	dbg("[i2c] Light Sensor write High Threshold (ALS_THDH : 0x%02X%02X)\n", data_buf[1], data_buf[0]);
	}		

	return 0;
}

static int cm3327_light_hw_set_lo_threshold(int low_threshold)
{
	int ret = 0;	
	uint8_t data_buf[2] = {0, 0};	

	/*Set Light Sensor Low Threshold*/		
	data_buf[0] = low_threshold % (1<<8);
	data_buf[1] = low_threshold /  (1<<8);
	ret = i2c_write_reg_u16(g_i2c_client, ALS_THDL, data_buf);
	if(ret < 0) {
		err("[i2c] Light Sensor write Low Threshold ERROR. (ALS_THDL : 0x%02X%02X)\n", data_buf[1], data_buf[0]);
	    	return ret;
	} else {
	    	dbg("[i2c] Light Sensor write Low Threshold (ALS_THDL : 0x%02X%02X)\n", data_buf[1], data_buf[0]);
	}

	return 0;
}

static int cm3327_light_hw_set_persistence(uint8_t persistence)
{
	int ret = 0;	
	uint8_t data_origin[2] = {0, 0};
	uint8_t data_buf[2] = {0, 0};	

	/* set Light Sensor Persistence */
	ret = i2c_read_reg_u16(g_i2c_client, ALS_CONF, data_buf);
	if (ret < 0) {
		err("Light Sensor read ALS_CONF ERROR\n");
		return ret;
	}
	dbg("Light Sensor read ALS_CONF (0x%02X%02X) \n", data_buf[1], data_buf[0]);
	memcpy(data_origin, data_buf, strlen(data_buf));

	persistence <<= CM3327_ALS_PERS_SHIFT;
	data_buf[1] &= CM3327_ALS_PERS_MASK;
	data_buf[1] |= persistence;

	ret = i2c_write_reg_u16(g_i2c_client, ALS_CONF, data_buf);
	if (ret < 0) {
		err("Light Sensor set Persistence (ALS_CONF) ERROR \n");
		return ret;
	} else {
		log("Light Sensor set Persistence (ALS_CONF : 0x%X -> 0x%X) \n", 
			data_origin[0], data_buf[0]);
	}

	return 0;
}

static int cm3327_light_hw_set_integration(uint8_t integration)
{
	int ret = 0;	
	uint8_t data_origin[2] = {0, 0};
	uint8_t data_buf[2] = {0, 0};	

	/* set Light Sensor Integration */
	ret = i2c_read_reg_u16(g_i2c_client, ALS_CONF, data_buf);
	if (ret < 0) {
		err("Light Sensor read ALS_CONF ERROR\n");
		return ret;
	}
	dbg("Light Sensor read ALS_CONF (0x%02X%02X) \n", data_buf[1], data_buf[0]);
	memcpy(data_origin, data_buf, strlen(data_buf));

	integration <<= CM3327_RGBIR_IT_SHIFT;
	data_buf[0] &= CM3327_RGBIR_IT_MASK;
	data_buf[0] |= integration;

	ret = i2c_write_reg_u16(g_i2c_client, ALS_CONF, data_buf);
	if (ret < 0) {
		err("Light Sensor set Integration (ALS_CONF) ERROR \n");
		return ret;
	} else {
		log("Light Sensor set Integration (ALS_CONF : 0x%X -> 0x%X) \n", 
			data_origin[0], data_buf[0]);
	}

	return 0;	
}

static struct lsensor_hw lsensor_hw_cm3327 = {
	.light_max_threshold = CM3327_LIGHT_MAX_THRESHOLD,
	.light_200lux_default = CM3327_LIGHT_200LUX_DEFAULT,
	.light_1000lux_default = CM3327_LIGHT_1000LUX_DEFAULT,
		
	.light_hw_turn_onoff = cm3327_light_hw_turn_onoff,
	.light_hw_get_r = cm3327_light_hw_get_r,
	.light_hw_get_g = cm3327_light_hw_get_g,
	.light_hw_get_b = cm3327_light_hw_get_b,
	.light_hw_get_ir = cm3327_light_hw_get_ir,
	.light_hw_set_hi_threshold = cm3327_light_hw_set_hi_threshold,
	.light_hw_set_lo_threshold = cm3327_light_hw_set_lo_threshold,
};

static struct IRsensor_hw IRsensor_hw_cm3327 = {	
	.vendor = "Capella",
	.module_number = "cm3327",

	.IRsensor_hw_check_ID = cm3327_IRsensor_hw_check_ID,
	.IRsensor_hw_init = cm3327_IRsensor_hw_init,
	.IRsensor_hw_get_interrupt = cm3327_IRsensor_hw_get_interrupt,
	.IRsensor_hw_show_allreg = cm3327_IRsensor_hw_show_allreg,
	.IRsensor_hw_set_register = cm3327_IRsensor_hw_set_register,
	.IRsensor_hw_get_register = cm3327_IRsensor_hw_get_register,

	.mlsensor_hw = &lsensor_hw_cm3327,
};

IRsensor_hw* Lsensor_hw_cm3327_getHardware(void)
{
	IRsensor_hw* Lsensor_hw_client = NULL;
	Lsensor_hw_client = &IRsensor_hw_cm3327;
	return Lsensor_hw_client;
}
