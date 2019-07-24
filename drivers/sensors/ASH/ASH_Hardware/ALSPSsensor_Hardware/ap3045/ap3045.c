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
/* IR Sensor CM36686 Module */
/******************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/input/ASH.h>
#include "ap3045.h"

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME	"ASH_HW"
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
static int ap3045_IRsensor_hw_check_ID(void);
static int ap3045_proximity_hw_set_config(void);
//static int ap3045_light_hw_set_config(void);
//static int ap3045_IRsensor_hw_set_interrupt(void);

static int ap3045_IRsensor_hw_init(struct i2c_client* client);
static int ap3045_IRsensor_hw_show_allreg(void);
static int ap3045_IRsensor_hw_get_interrupt(void);
static int ap3045_proximity_hw_turn_onoff(bool bOn);
static int ap3045_proximity_hw_get_adc(void);
static int ap3045_proximity_hw_set_hi_threshold(int hi_threshold);
static int ap3045_proximity_hw_set_lo_threshold(int low_threshold);
static int ap3045_proximity_hw_clean_interrupt_flag(void);
static int ap3045_proximity_hw_clean_interrupt_control(void);
static int ap3045_proximity_hw_set_ps_time(void);
static int ap3045_proximity_hw_set_led_current(uint8_t led_current);
static int ap3045_proximity_hw_set_ps_mean_time(void);
static int ap3045_light_hw_set_ALS_gain(void);
static int ap3045_light_hw_set_ALS_time(void);
//static int ap3045_proximity_hw_set_persistence(uint8_t persistence);
//static int ap3045_proximity_hw_set_integration(uint8_t integration);
//static int ap3045_proximity_hw_set_mean(uint8_t mean);

static int ap3045_light_hw_turn_onoff(bool bOn);
static int ap3045_light_hw_get_adc(void);
static int ap3045_light_hw_set_hi_threshold(int hi_threshold);
static int ap3045_light_hw_set_lo_threshold(int low_threshold);
//static int ap3045_light_hw_set_persistence(uint8_t persistence);
//static int ap3045_light_hw_set_integration(uint8_t integration);

static u8 ap3045_reg[AP3045_NUM_CACHABLE_REGS] = 
{0x00,0x01,0x02,0x06,0x07,0x08,0x0A,0x0C,0x0D,0x0F,
    0x10,0x1B,0x26,0x27,0x2A,0x2B,0x2C,0x2D,0x2E,0x2F,
    0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,
    0x3A,0x3B,0x3C,0x3D,0x80,0x81,0x82,0x83,0x84,0x85,
    0x86,0xE1};

/**************************/
/* i2c read/write register */
/*************************/
static int ap3045_IRsensor_hw_check_ID(void)
{	
	uint8_t data_buf = 0;	

	/* Check the Device ID */
	data_buf = i2c_read_reg_u8(g_i2c_client, AP3045_REG_SYS_PID);
	if (data_buf != 0x0b) {
		err("ap3045_IRsensor_hw_check_ID ERROR. (ID_REG : 0x%02X)\n", AP3045_REG_SYS_PID);
		return -ENOMEM;
	}
	log("ap3045_IRsensor_hw_check_ID (0x%X) \n", data_buf);
	return 0;
}

static int ap3045_proximity_hw_set_config(void)
{
	int ret = 0;	

	ret = ap3045_proximity_hw_clean_interrupt_flag();
	if (ret < 0)		
		return ret;
	ret = ap3045_proximity_hw_clean_interrupt_control();
	if (ret < 0)
		return ret;
	ret = ap3045_proximity_hw_set_ps_time();
	if (ret < 0)
		return ret;
	ret = ap3045_proximity_hw_set_led_current(AP3045_SYS_LED_050);
	if (ret < 0)		
		return ret;
	ret = ap3045_proximity_hw_set_ps_mean_time();
	if (ret < 0)
		return ret;
	ret = ap3045_light_hw_set_ALS_gain();
	if (ret < 0)		
		return ret;
	ret = ap3045_light_hw_set_ALS_time();
	if (ret < 0)		
		return ret;

	return 0;
}

/*****************/
/*IR Sensor Part*/
/****************/
static int ap3045_IRsensor_hw_init(struct i2c_client* client)
{
	int ret = 0;

	g_i2c_client = client;
	
	/* Check the Hardware Device */
	ret = ap3045_IRsensor_hw_check_ID();
	if (ret < 0) {		
		return ret;
	}

	 /*Set Proximity config */
	ret =ap3045_proximity_hw_set_config();
	if (ret < 0) {		
		return ret;
	}

	/*Set Light Sensor config*/
//	ret =ap3045_light_hw_set_config();
//	if (ret < 0) {		
//		return ret;
//	}

	/*Set Interrupt Control*/
//	ret = ap3045_IRsensor_hw_set_interrupt();
//	if (ret < 0) {		
//		return ret;
//	}

	return 0;
}

static int ap3045_IRsensor_hw_show_allreg(void)
{	
	int reg = 0;
	uint8_t buf = 0;
	for(;reg < AP3045_NUM_CACHABLE_REGS; reg++)
	{
		buf = i2c_read_reg_u8(g_i2c_client, ap3045_reg[reg]);
		log("Show All Register (0x%X) = 0x%02X\n", ap3045_reg[reg], buf);
		if(buf < 0) 
		{
			err("IR Sensor show all Register ERROR. (REG:0x%X)\n", ap3045_reg[reg]);
			return buf;
		}
	}
	
	return 0;
}

static int ap3045_IRsensor_hw_set_register(uint8_t reg, int value)
{	
	
	int ret = 0;	
		
	ret = i2c_write_reg_u8(g_i2c_client, reg, value);
	log("Set Register Value (0x%X) = 0x%02X\n", reg, value);
	if(ret < 0) 
	{
		err("IR Sensor Set Register Value ERROR. (REG:0x%X)\n", reg);
		return ret;
	}
	
	return 0;
}

static int ap3045_IRsensor_hw_get_register(uint8_t reg)
{
	uint8_t buf = 0;
	
	buf = i2c_read_reg_u8(g_i2c_client, reg);
	log("Get Register Value (0x%X) = 0x%02X\n", reg, buf);
	if(buf < 0) 
	{
		err("IR Sensor Get Register Value ERROR. (REG:0x%X)\n", reg);
		return buf;
	}
	
	return buf;
}

static int ap3045_IRsensor_hw_get_interrupt(void)
{
	uint8_t buf = 0;
	uint8_t data_buf = 0;
	bool check_flag = false;
	int irsensor_int = 0;
	int ret = 0;
	
	// Read INT_FLAG and clean the interrupt
	buf = i2c_read_reg_u8(g_i2c_client, AP3045_REG_SYS_INTSTATUS);
	data_buf = buf;
	data_buf &= ~AP3045_REG_SYS_INT_PS_CLR;
	data_buf &= ~AP3045_REG_SYS_INT_LS_CLR;
	ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_SYS_INTSTATUS, data_buf);
	if (ret < 0) {
		err("ap3045_IRsensor_hw_get_interrupt clear ERROR \n");
		return ret;
	}
	dbg("ap3045_IRsensor_hw_get_interrupt clear (0x%02X) \n", data_buf);

	//Proximity Sensor work
	if (buf&AP3045_REG_SYS_INT_PMASK) 
	{
		check_flag =true;		
		if (buf&AP3045_OBJ_MASK) {
			irsensor_int |= IRSENSOR_INT_PS_CLOSE;
		}else{
			irsensor_int |= IRSENSOR_INT_PS_AWAY;
		}
	}

	// Light Sensor work
	if(buf&AP3045_REG_SYS_INT_AMASK) 
	{	
		check_flag =true;
		irsensor_int |= IRSENSOR_INT_ALS;
	} 

	// Interrupt Error
	if(check_flag == false) {
		if(buf == AP3045_REG_SYS_INT_CHECK) {
			log("IRsensor INT_FLAG (0x%02X)\n", buf);
		} else {
			err("Can NOT recognize the INT_FLAG (0x%02X)\n", buf);
			ap3045_IRsensor_hw_show_allreg();
		}
	}
	
	return irsensor_int;
}

/****************/
/*Proximity Part*/
/****************/
static int ap3045_proximity_hw_turn_onoff(bool bOn)
{	
	int ret = 0;	
	uint8_t power_state_data_buf= 0;
	uint8_t power_state_data_origin = 0;

	/* read power status */
	power_state_data_buf = i2c_read_reg_u8(g_i2c_client, AP3045_REG_SYS_CONF);
	if (power_state_data_buf < 0) {
		err("Proximity read ap3045_REG_SYS_CONF ERROR\n");
		return power_state_data_buf;
	}
	dbg("Proximity read ap3045_REG_SYS_CONF (0x%02X) \n", power_state_data_buf);	
	power_state_data_origin = power_state_data_buf;
	
	if (bOn == 1)	{	/* power on */		
		power_state_data_buf |= AP3045_SYS_PS_ENABLE;
		
		ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_SYS_CONF, power_state_data_buf);
		if (ret < 0) {
			err("Proximity power on ERROR (ap3045_REG_SYS_CONF) \n");
			return ret;
		} else {
			log("Proximity power on (ap3045_REG_SYS_CONF : 0x%X -> 0x%X) \n", 
				power_state_data_origin, power_state_data_buf);
		}
	} else	{	/* power off */
		power_state_data_buf &= ~AP3045_SYS_PS_ENABLE;      	

		ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_SYS_CONF, power_state_data_buf);
		if (ret < 0) {
			err("Proximity power off ERROR (ap3045_REG_SYS_CONF) \n");
			return ret;
		} else {
			log("Proximity power off (ap3045_REG_SYS_CONF : 0x%X -> 0x%X) \n", 
				power_state_data_origin, power_state_data_buf);
		}
	}
	
	return 0;
}

static int ap3045_proximity_hw_get_adc(void)
{
	int ret = 0;
	int adc = 0;
	uint8_t adc_buf[2] = {0, 0};	

	adc_buf[0] = i2c_read_reg_u8(g_i2c_client, AP3045_REG_PS_DATA_LOW);
	if (ret < 0) {
		err("Proximity get adc ERROR. (ap3045_REG_PS_DATA_LOW)\n");
		return ret;
	}

	adc_buf[1] = i2c_read_reg_u8(g_i2c_client, AP3045_REG_PS_DATA_HIGH);
	if (ret < 0) {
		err("Proximity get adc ERROR. (ap3045_REG_PS_DATA_HIGH)\n");
		return ret;
	}
	
	adc = (adc_buf[1] << 8) + adc_buf[0];
	dbg("Proximity get adc : 0x%02X%02X\n", adc_buf[1], adc_buf[0]); 
	
	return adc;
}

static int ap3045_proximity_hw_clean_interrupt_flag(void)
{
	int ret = 0;	
	uint8_t data_buf = 0;	

	// clean interrupt flag
	ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_SYS_INTSTATUS, data_buf);
	if (ret < 0) {
		err("ap3045_proximity_hw_clean_interrupt_flag write ERROR \n");
		return ret;
	} else {
		log("ap3045_proximity_hw_clean_interrupt_flag write (0x%X) \n", data_buf);
	}

	return 0;
}

static int ap3045_proximity_hw_clean_interrupt_control(void)
{
	int ret = 0;	
	uint8_t data_buf = 168;	

	// clean interrupt flag
	ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_SYS_INTCTRL, data_buf);
	if (ret < 0) {
		err("ap3045_proximity_hw_clean_interrupt_control write ERROR \n");
		return ret;
	} else {
		log("ap3045_proximity_hw_clean_interrupt_control write (0x%X) \n", data_buf);
	}

	return 0;
}

static int ap3045_proximity_hw_set_ps_time(void)
{
	int ret = 0;
	uint8_t data_buf = 1;

	ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_PS_TIME, data_buf);
	if (ret < 0) {
		err("ap3045_proximity_hw_set_ps_time ERROR \n");
		return ret;
	} else {
		log("ap3045_proximity_hw_set_ps_time write (0x%X) \n", data_buf);
	}

	return 0;
}

static int ap3045_proximity_hw_set_hi_threshold(int hi_threshold)
{	
	int ret = 0;
	uint8_t data_buf[2] = {0,0};

	//Set Proximity High Threshold
	hi_threshold = hi_threshold / 5;
	data_buf[0] = hi_threshold % (1<<8);
	data_buf[1] = hi_threshold /  (1<<8);
	ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_PS_THDH_L, data_buf[0]);
	if(ret < 0) {
		err("Proximity write High Threshold lsb ERROR. (ap3045_REG_PS_THDH_L : 0x%02X)\n",  data_buf[0]);
	    	return ret;
	} else {
	    	log("Proximity write High Threshold lsb (ap3045_REG_PS_THDH_L : 0x%02X)\n", data_buf[0]);
	}		

	ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_PS_THDH_H, data_buf[1]);
	if(ret < 0) {
		err("Proximity write High Threshold msb ERROR. (ap3045_REG_PS_THDH_H : 0x%02X)\n",  data_buf[1]);
	    	return ret;
	} else {
	    	log("Proximity write High Threshold msb (ap3045_REG_PS_THDH_H : 0x%02X)\n", data_buf[1]);
	}	
	
	return 0;
}

static int ap3045_proximity_hw_set_lo_threshold(int low_threshold)
{
	int ret = 0;
	uint8_t data_buf[2] = {0, 0};	

	//Set Proximity Low Threshold
	low_threshold = low_threshold / 5;
	data_buf[0] = low_threshold % (1<<8);
	data_buf[1] = low_threshold /  (1<<8);
	ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_PS_THDL_L, data_buf[0]);
	if(ret < 0) {
		err("Proximity write Low Threshold lsb ERROR. (ap3045_REG_PS_THDL_L : 0x%02X)\n", data_buf[0]);
	    	return ret;
	} else {
	    	log("Proximity write Low Threshold lsb (ap3045_REG_PS_THDL_L : 0x%02X)\n", data_buf[0]);
	}

	ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_PS_THDL_H, data_buf[1]);
	if(ret < 0) {
		err("Proximity write Low Threshold msb ERROR. (ap3045_REG_PS_THDL_H : 0x%02X)\n",  data_buf[1]);
	    	return ret;
	} else {
	    	log("Proximity write Low Threshold msb (ap3045_REG_PS_THDL_H : 0x%02X)\n", data_buf[1]);
	}	
	
	return 0;
}

static int ap3045_proximity_hw_set_led_current(uint8_t led_current)
{
	int ret = 0;	
	uint8_t data_origin = 0;
	uint8_t data_buf = 0;	

	// read LED current
	data_buf = i2c_read_reg_u8(g_i2c_client, AP3045_REG_PS_LEDD);
	if (data_buf < 0) {
		err("ap3045_proximity_hw_set_led_current read ERROR\n");
		return data_buf;
	}
	dbg("ap3045_proximity_hw_set_led_current read (0x%02X) \n", data_buf);
	
	data_origin = data_buf;		
	data_buf = led_current;
	
	// write LED current
	data_buf <<= AP3045_SYS_LED_CONTROL_SHIFT;
	ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_PS_LEDD, data_buf);
	if (ret < 0) {
		err("ap3045_proximity_hw_set_led_current write ERROR \n");
		return ret;
	} else {
		log("ap3045_proximity_hw_set_led_current write (0x%X -> 0x%X) \n", data_origin, data_buf);
	}

	return 0;
}

static int ap3045_proximity_hw_set_ps_mean_time(void)
{
	int ret = 0;	
	uint8_t data_buf = 9;

	ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_PS_MEAN_TIME, data_buf);
	if (ret < 0) {
		err("ap3045_proximity_hw_set_ps_time ERROR \n");
		return ret;
	} else {
		log("ap3045_proximity_hw_set_ps_time write (0x%X) \n", data_buf);
	}

	return 0;
}

static int ap3045_light_hw_set_ALS_gain(void) {
	int ret = 0;	
	uint8_t data_buf = 2;
	
	// set ALS gain
	ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_ALS_GAIN, data_buf);
	if (ret < 0) {
		err("ap3045_proximity_hw_set_ALS_gain write ERROR \n");
		return ret;
	} else {
		log("ap3045_proximity_hw_set_ALS_gain write (0x%X) \n", data_buf);
	}

	return 0;
}

static int ap3045_light_hw_set_ALS_time(void) {
	int ret = 0;	
	uint8_t data_buf = 28;
	
	// set ALS time
	ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_ALS_TIME, data_buf);
	if (ret < 0) {
		err("ap3045_proximity_hw_set_ALS_time write ERROR \n");
		return ret;
	} else {
		log("ap3045_proximity_hw_set_ALS_time write (0x%X) \n", data_buf);
	}

	return 0;
}
/*
static int ap3045_proximity_hw_set_mean(uint8_t mean)
{
	int ret = 0;	
	uint8_t data_origin= 0;
	uint8_t data_buf = 0;	

	// read Proximity Mean state
	data_buf = i2c_read_reg_u8(g_i2c_client, AP3045_REG_PS_MEAN);
	if (data_buf < 0) {
		err("ap3045_proximity_hw_set_mean read ERROR\n");
		return data_buf;
	}
	dbg("ap3045_proximity_hw_set_mean read (0x%02X) \n", data_buf);

	//mean <<= ap3045_SYS_LED_GAIN_SHIFT;
	//data_buf &=ap3045_SYS_LED_GAIN_MASK;
	data_buf = mean;

	ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_PS_MEAN, data_buf);
	if (ret < 0) {
		err("ap3045_proximity_hw_set_mean write ERROR \n");
		return ret;
	} else {
		log("ap3045_proximity_hw_set_mean write (0x%X -> 0x%X) \n", data_origin, data_buf);
	}
	
	return 0;
}
*/
/*
static int ap3045_proximity_hw_set_integration(uint8_t integration)
{
	int ret = 0;	
	uint8_t data_origin= 0;
	uint8_t data_buf = 0;	

	// read Proximity Integration state
	data_buf = i2c_read_reg_u8(g_i2c_client, AP3045_REG_PS_INTEGR);
	if (data_buf < 0) {
		err("ap3045_proximity_hw_set_integration read ERROR\n");
		return data_buf;
	}
	dbg("ap3045_proximity_hw_set_integration read (0x%02X) \n", data_buf);

	data_origin = data_buf;
	data_buf = integration;

	ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_PS_INTEGR, data_buf);
	if (ret < 0) {
		err("ap3045_proximity_hw_set_integration write ERROR \n");
		return ret;
	} else {
		log("ap3045_proximity_hw_set_integration write (0x%X -> 0x%X) \n", data_origin, data_buf);
	}
	
	return 0;
}
*/
/*
static int ap3045_proximity_hw_set_persistence(uint8_t persistence)
{
	int ret = 0;	
	uint8_t data_origin= 0;
	uint8_t data_buf = 0;	

	// read Proximity Integration state
	data_buf = i2c_read_reg_u8(g_i2c_client, AP3045_REG_PS_PERSIS);
	if (data_buf < 0) {
		err("ap3045_proximity_hw_set_persistence read ERROR\n");
		return data_buf;
	}
	dbg("ap3045_proximity_hw_set_persistence read (0x%02X) \n", data_buf);

	data_origin = data_buf;
	data_buf = persistence;

	ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_PS_PERSIS, data_buf);
	if (ret < 0) {
		err("ap3045_proximity_hw_set_persistence write ERROR \n");
		return ret;
	} else {
		log("ap3045_proximity_hw_set_persistence write (0x%X -> 0x%X) \n", data_origin, data_buf);
	}
	
	return 0;
}
*/

/*********************/
/* Light Sensor Part */
/********************/
static int ap3045_light_hw_turn_onoff(bool bOn)
{
	int ret = 0;	
	uint8_t power_state_data_buf = 0;
	uint8_t power_state_data_origin = 0;

	/* read power status */
	power_state_data_buf = i2c_read_reg_u8(g_i2c_client, AP3045_REG_SYS_CONF);	
	if (power_state_data_buf < 0) {
		err("Light Sensor read ap3045_REG_SYS_CONF ERROR\n");
		return power_state_data_buf;
	}
	dbg("Light Sensor read ap3045_REG_SYS_CONF (0x%02X) \n", power_state_data_buf);
	power_state_data_origin = power_state_data_buf;	

	if (bOn == 1)	{	/* power on */
		power_state_data_buf |= AP3045_SYS_ALS_ENABLE;

		ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_SYS_CONF, power_state_data_buf);
		if (ret < 0) {
			err("Light Sensor power on ERROR (ap3045_REG_SYS_CONF) \n");
			return ret;
		} else {
			log("Light Sensor power on (ap3045_REG_SYS_CONF : 0x%X -> 0x%X) \n", 
				power_state_data_origin, power_state_data_buf);
		}		
	} else	{	/* power off */
		power_state_data_buf &= ~AP3045_SYS_ALS_ENABLE;
		
		ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_SYS_CONF, power_state_data_buf);
		if (ret < 0) {
			err("Light Sensor power off ERROR (ap3045_REG_SYS_CONF) \n");
			return ret;
		} else {
			log("Light Sensor power off (ap3045_REG_SYS_CONF : 0x%X -> 0x%X) \n", 
				power_state_data_origin, power_state_data_buf);
		}
	}	

	return 0;
}

static int ap3045_light_hw_get_adc(void)
{
	int ret = 0;
	int adc = 0;
	uint8_t adc_buf[2] = {0, 0};	
	
	adc_buf[0] = i2c_read_reg_u8(g_i2c_client, AP3045_REG_ALS_DATA_LOW);
	if (ret < 0) {
		err("Light Sensor Get adc ERROR (ap3045_REG_ALS_DATA_LOW)\n");
		return ret;
	}

	adc_buf[1] = i2c_read_reg_u8(g_i2c_client, AP3045_REG_ALS_DATA_HIGH);
	if (ret < 0) {
		err("Light Sensor Get adc ERROR (ap3045_REG_ALS_DATA_HIGH)\n");
		return ret;
	}
	
	adc = (adc_buf[1] << 8) + adc_buf[0];
	dbg("Light Sensor Get adc : 0x%02X%02X\n", adc_buf[1], adc_buf[0]); 
	
	return adc;	
}

static int ap3045_light_hw_set_hi_threshold(int hi_threshold)
{
	int ret = 0;	
	uint8_t data_buf[2] = {0, 0};	

	//Set Light Sensor High Threshold
	data_buf[0] = hi_threshold % (1<<8);
	data_buf[1] = hi_threshold /  (1<<8);
	ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_ALS_THDH_L, data_buf[0]);
	if(ret < 0) {
		err("Light Sensor write High Threshold lsb ERROR. (ap3045_REG_ALS_THDH_L : 0x%02X)\n", data_buf[0]);
	    	return ret;
	} else {
	    	dbg("Light Sensor write High Threshold lsb (ALS_ap3045_REG_ALS_THDH_LTHDH : 0x%02X)\n", data_buf[0]);
	}		

	ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_ALS_THDH_H, data_buf[1]);
	if(ret < 0) {
		err("Light Sensor write High Threshold msb ERROR. (ap3045_REG_ALS_THDH_H : 0x%02X)\n", data_buf[1]);
	    	return ret;
	} else {
	    	dbg("Light Sensor write High Threshold msb (ap3045_REG_ALS_THDH_H : 0x%02X)\n", data_buf[1]);
	}	
	
	return 0;
}

static int ap3045_light_hw_set_lo_threshold(int low_threshold)
{
	int ret = 0;	
	uint8_t data_buf[2] = {0, 0};	

	//Set Light Sensor Low Threshold
	data_buf[0] = low_threshold % (1<<8);
	data_buf[1] = low_threshold /  (1<<8);
	ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_ALS_THDL_L, data_buf[0]);
	if(ret < 0) {
		err("Light Sensor write Low Threshold lsb ERROR. (ap3045_REG_ALS_THDL_L : 0x%02X)\n", data_buf[0]);
	    	return ret;
	} else {
	    	dbg("Light Sensor write Low Threshold lsb (ap3045_REG_ALS_THDL_L : 0x%02X)\n", data_buf[0]);
	}

	ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_ALS_THDL_H, data_buf[1]);
	if(ret < 0) {
		err("Light Sensor write Low Threshold msb ERROR. (ap3045_REG_ALS_THDL_H : 0x%02X)\n", data_buf[1]);
	    	return ret;
	} else {
	    	dbg("Light Sensor write Low Threshold msb (ap3045_REG_ALS_THDL_H : 0x%02X)\n", data_buf[1]);
	}
	
	return 0;
}

/*
static int ap3045_light_hw_set_integration(uint8_t integration)
{
	int ret = 0;	
	uint8_t data_origin= 0;
	uint8_t data_buf = 0;	

	// read Light Sensor Integration state 
	data_buf = i2c_read_reg_u8(g_i2c_client, AP3045_REG_ALS_GAIN);
	if (data_buf < 0) {
		err("ap3045_light_hw_set_integration read ERROR\n");
		return data_buf;
	}
	dbg("ap3045_light_hw_set_integration read (0x%02X) \n", data_buf);
	data_origin = data_buf;
	
	integration <<= AP3045_ALS_GAIN_SHIFT;
	data_buf &= AP3045_ALS_GAIN_MASK;
	data_buf |= integration;

	ret = i2c_write_reg_u8(g_i2c_client, AP3045_REG_ALS_GAIN, data_buf);
	if (ret < 0) {
		err("ap3045_light_hw_set_integration write ERROR \n");
		return ret;
	} else {
		log("ap3045_light_hw_set_integration write (0x%X -> 0x%X) \n", data_origin, data_buf);
	}
	
	return 0;

}
*/
static struct psensor_hw psensor_hw_ap3045 = {
	.proximity_low_threshold_default = AP3045_PROXIMITY_THDL_DEFAULT,
	.proximity_hi_threshold_default = AP3045_PROXIMITY_THDH_DEFAULT,
	
	.proximity_hw_turn_onoff = ap3045_proximity_hw_turn_onoff,
	.proximity_hw_get_adc = ap3045_proximity_hw_get_adc,
	.proximity_hw_set_hi_threshold = ap3045_proximity_hw_set_hi_threshold,
	.proximity_hw_set_lo_threshold = ap3045_proximity_hw_set_lo_threshold,
};

static struct lsensor_hw lsensor_hw_ap3045 = {
	.light_max_threshold = AP3045_LIGHT_MAX_THRESHOLD,
	.light_200lux_default = AP3045_LIGHT_200LUX_DEFAULT,
	.light_1000lux_default = AP3045_LIGHT_1000LUX_DEFAULT,
	
	.light_hw_turn_onoff = ap3045_light_hw_turn_onoff,
	.light_hw_get_adc = ap3045_light_hw_get_adc,
	.light_hw_set_hi_threshold = ap3045_light_hw_set_hi_threshold,
	.light_hw_set_lo_threshold = ap3045_light_hw_set_lo_threshold,
};

static struct IRsensor_hw IRsensor_hw_ap3045 = {
	.vendor = "Dyna Image",
	.module_number = "ap3045",

	.IRsensor_hw_check_ID = ap3045_IRsensor_hw_check_ID,
	.IRsensor_hw_init = ap3045_IRsensor_hw_init,
	.IRsensor_hw_get_interrupt = ap3045_IRsensor_hw_get_interrupt,
	.IRsensor_hw_show_allreg = ap3045_IRsensor_hw_show_allreg,
	.IRsensor_hw_set_register = ap3045_IRsensor_hw_set_register,
	.IRsensor_hw_get_register = ap3045_IRsensor_hw_get_register,

	.mpsensor_hw = &psensor_hw_ap3045,
	.mlsensor_hw = &lsensor_hw_ap3045,
};

IRsensor_hw* IRsensor_hw_ap3045_getHardware(void)
{
	IRsensor_hw* IRsensor_hw_client = NULL;
	IRsensor_hw_client = &IRsensor_hw_ap3045;
	return IRsensor_hw_client;
}

