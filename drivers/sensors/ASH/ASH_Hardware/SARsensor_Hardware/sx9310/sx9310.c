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

/*******************************************************/
/* ALSPS Front RGB Sensor CM36656 Module */
/******************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/input/ASH.h>
#include <linux/delay.h>
#include "sx9310.h"

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_HW"
#define SENSOR_TYPE_NAME		"SAR"

#undef dbg
#ifdef ASH_HW_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

/*****************************************/
/*Global static variable and function*/
/****************************************/
static struct i2c_client	*g_i2c_client = NULL;

static int sx9310_SAR_sensor_hw_check_ID(void);
static int sx9310_SAR_sensor_hw_init(struct i2c_client* client);
static int sx9310_SAR_sensor_hw_read_regStat(void);
static int sx9310_SAR_sensor_hw_get_interrupt(void);
static int sx9310_SAR_sensor_hw_show_allreg(void);
static int sx9310_SAR_sensor_hw_set_register(uint8_t reg, int value);
static int sx9310_SAR_sensor_hw_get_register(uint8_t reg);
static int sx9310_SAR_sensor_hw_turn_onoff(bool bOn);
static int sx9310_SAR_sensor_hw_get_proxuserful(uint8_t cs);
static int sx9310_SAR_sensor_hw_read_rawData(void);
static int sx9310_SAR_sensor_hw_get_manual_offset_calibration(void);
static int sx9310_SAR_sensor_hw_manual_offset_calibration(int val);

static int sx9310_SAR_sensor_hw_check_ID(void)
{
	u8 chip_id = 0;

	/* Check the Device ID */
	chip_id = i2c_read_reg_u8(g_i2c_client, SX9310_WHO_AM_I_REG);
	if(chip_id < 0) {
		err("%s: Read SX9310_WHO_AM_I_REG(0x%02x) FAIL.(err=%d)\n",
			__FUNCTION__, SX9310_WHO_AM_I_REG, chip_id);
		return -ENOMEM;
	} else if(chip_id != SX9310_CHIP_ID) {
		err("%s: FAIL, the Chip ID(0x%02x) is not as expected(0x%02x)",
			__FUNCTION__, chip_id, SX9310_CHIP_ID);
		return -ENOMEM;
	}
	log("%s: The Chip ID(reg:0x%02x) is 0x%02x.\n", __FUNCTION__,
		SX9310_WHO_AM_I_REG, chip_id);
	
	return 0;
}

static int sx9310_SAR_sensor_hw_check_HW_address(void)
{
	u8 i2c_addr = 0;

	/* Check the Device HW address */
	i2c_addr = i2c_read_reg_u8(g_i2c_client, SX9310_I2C_ADDR_REG);
	if (i2c_addr < 0) {
		err("%s: Read SX9310_I2C_ADDR_REG(0x%02x) FAIL.(err=%d)\n",
			__FUNCTION__, SX9310_I2C_ADDR_REG, i2c_addr);
		return -ENOMEM;
	} else if (i2c_addr != SX9310_I2C_ADDR){
		err("%s: FAIL, the I2C address(0x%02x) is not as expected(0x%02x)",
			__FUNCTION__, i2c_addr, SX9310_I2C_ADDR);
		return -ENOMEM;
	}
	log("%s: The I2C address is 0x%02x.\n", __FUNCTION__, i2c_addr);

	return 0;
}

static int sx9310_SAR_sensor_hw_reg_init(void)
{
	int ret = 0;
	int i = 0;
	
	for (i = 0; i < ARRAY_SIZE(sx9310_i2c_reg_setup); i++) {
		ret = i2c_write_reg_u8(g_i2c_client, sx9310_i2c_reg_setup[i].reg, sx9310_i2c_reg_setup[i].val);
		if (ret < 0) {
			err("%s: Write setup register(0x%02x) FAIL.(err=%d)\n", __FUNCTION__,
				sx9310_i2c_reg_setup[i].reg, ret);
			return ret;
		}
		log("%s: Write setup register(0x%02x) to 0x%02x).\n", __FUNCTION__,
			sx9310_i2c_reg_setup[i].reg, sx9310_i2c_reg_setup[i].val);
	}
	
	return ret;
}

static int sx9310_SAR_sensor_hw_init(struct i2c_client* client)
{
	int ret = 0;
	
	g_i2c_client = client;

	//sx9310_SAR_sensor_hw_show_allreg();

	/* Reset Interrupt status by SX9310_IRQSTAT_REG register */
	ret = i2c_read_reg_u8(g_i2c_client, SX9310_IRQSTAT_REG);
	if(ret < 0) {
		err("%s: Read SX9310_IRQSTAT_REG(0x%02x) FAIL.(err=%d)\n", 
			__FUNCTION__, SX9310_IRQSTAT_REG , ret);
		return ret;
	}
	log("%s: Read SX9310_IRQSTAT_REG(0x%02x): 0x%02x).\n", 
		__FUNCTION__, SX9310_IRQSTAT_REG , ret);

	/* Check the Device ID */
	ret = sx9310_SAR_sensor_hw_check_ID();
	if(ret < 0) {
		err("%s: Check chip ID FAIL(err=%d)", __FUNCTION__, ret);
		return ret;
	}

	/* Check the HW address */
	ret = sx9310_SAR_sensor_hw_check_HW_address();
	if(ret < 0) {
		err("%s: Check i2c address FAIL(err=%d)", __FUNCTION__, ret);
		return ret;
	}
	
	/* Init register value */
	ret = sx9310_SAR_sensor_hw_reg_init();
	if(ret < 0) {
		err("%s: Register initial FAIL(err=%d)", __FUNCTION__, ret);
		return ret;
	}
	
	return 0;
}

static int sx9310_SAR_sensor_hw_show_allreg(void)
{
	int i = 0;
	uint8_t data_buf = 0;

	/* Check the Device ID */
	sx9310_SAR_sensor_hw_check_ID();

	/* Check the HW address */
	sx9310_SAR_sensor_hw_check_HW_address();

	for (i = 0; i < SX9310_SETTING_REGISTER_MAX; i++)	{
		data_buf = i2c_read_reg_u8(g_i2c_client, sx9310_i2c_reg_setup[i].reg);
		if (data_buf < 0) {
			err("%s: Read setup register(0x%02x) FAIL.(err=%d)\n",
				__FUNCTION__, sx9310_i2c_reg_setup[i].reg, data_buf);
			return data_buf;
		}
		log("%s: Read setup register(0x%02x): 0x%02x).\n",
			__FUNCTION__, sx9310_i2c_reg_setup[i].reg, data_buf);
	}

	for (i = 0; i < SX9310_DATA_READBACK_REGISTER_MAX; i++)	{
		data_buf = i2c_read_reg_u8(g_i2c_client, sx9310_i2c_data_readback_reg[i].reg );
		if (data_buf < 0) {
			err("%s: Read readback register(0x%02x) FAIL.(err=%d)\n",
				__FUNCTION__, sx9310_i2c_data_readback_reg[i].reg, data_buf);
			return data_buf;
		}
		log("%s: Read readback register(0x%02x): 0x%02x).\n",
			__FUNCTION__, sx9310_i2c_data_readback_reg[i].reg, data_buf);
	}
	
	return 0;
}

static int sx9310_SAR_sensor_hw_read_regStat(void)
{
	u8 irq_status = 0;
	
	/* Clean interrupt */
	irq_status = i2c_read_reg_u8(g_i2c_client, SX9310_IRQSTAT_REG);
	if(irq_status < 0) {
		err("%s: Get SX9310_IRQSTAT_REG(0x%02x) FAIL.(err=%d)\n",
			__FUNCTION__, SX9310_IRQSTAT_REG, irq_status);
		return irq_status;
	}
	dbg("%s: SX9310_IRQSTAT_REG(0x%02x): 0x%02x).\n",
		__FUNCTION__, SX9310_IRQSTAT_REG, irq_status);
	
	if((irq_status&0x40) == 0x40) {
		return OBJECT_NEAR;
	} else if((irq_status&0x20) == 0x20) {
		return OBJECT_FAR;
	}
	
	return irq_status;
}

static int sx9310_SAR_sensor_hw_get_interrupt(void)
{
	uint8_t sensor_status = 0;

	/* Read detect status */
	sensor_status = i2c_read_reg_u8(g_i2c_client, SX9310_STAT0_REG);

	if (sensor_status < 0){
		err("%s: Get SX9310_STAT0_REG(0x%02x) FAIL.(err=%d)\n",
			__FUNCTION__, SX9310_STAT0_REG, sensor_status);
		return sensor_status;
	}
	dbg("%s: SX9310_STAT0_REG(0x%02x): 0x%02x).\n",
		__FUNCTION__, SX9310_STAT0_REG, sensor_status);

	return sensor_status;
}

static int sx9310_SAR_sensor_hw_set_register(uint8_t reg, int value)
{
	int ret = 0;
	uint8_t buf[2] = {0};

	buf[1] = value/256;
	buf[0] = value%256;
	
	ret = i2c_write_reg_u16(g_i2c_client, reg, buf);
	log("%s: Set register(0x%x) to 0x%02x%02x\n",
		__FUNCTION__, reg, buf[1], buf[0]);
	if(ret < 0) {
		err("%s: Set register(0x%x) FAIL.(ret=%d)\n",
			__FUNCTION__, reg, ret);
		return ret;
	}

	return 0;
}

static int sx9310_SAR_sensor_hw_get_register(uint8_t reg)
{
	int ret = 0;	
	uint8_t buf[2] = {0};
	int value = 0;
	
	ret = i2c_read_reg_u16(g_i2c_client, reg, buf);
	log("%s: Get register(0x%x): 0x%02x%02x\n",
		__FUNCTION__, reg, buf[1], buf[0]);
	if(ret < 0) {
		err("%s: Get register(0x%x) FAIL.(ret=%d)\n",
			__FUNCTION__, reg, ret);
		return ret;
	}

	value =  buf[1]*256 + buf[0];

	return value;
}

static int sx9310_SAR_sensor_hw_turn_onoff(bool bOn)
{
	int ret = 0;

	if(bOn) {
		/* perform a reset */
		ret = i2c_write_reg_u8(g_i2c_client, SX9310_SOFTRESET_REG, SX9310_SOFTRESET);
		if(ret < 0){
			err("%s: write SX9310_SOFTRESET_REG(0x%x) FAIL.(err=%d)\n", 
				__FUNCTION__, SX9310_SOFTRESET_REG, ret);
		}
		msleep(300);

		/* Init register value */
		ret = sx9310_SAR_sensor_hw_reg_init();
		if(ret < 0) {
			err("%s: Register initial FAIL(err=%d)", __FUNCTION__, ret);
			return ret;
		}
		ret = i2c_write_reg_u8(g_i2c_client, SX9310_CPS_CTRL0_REG, 0x25);
		if(ret < 0) {
			err("%s: Set SX9310_CPS_CTRL0_REG(0x%x) FAIL.(ret=%d)\n",
				__FUNCTION__, SX9310_CPS_CTRL0_REG, ret);
			return ret;
		}
		/* make sure everything is running */
		msleep(100);

		sx9310_SAR_sensor_hw_manual_offset_calibration(1);

		/* make sure no interrupts are pending since enabling irq will only
		 * work on next falling edge */
		ret = sx9310_SAR_sensor_hw_read_regStat();
		log("%s: Enable sar sensor!!\n", __FUNCTION__);
	} else {
		ret = i2c_write_reg_u8(g_i2c_client, SX9310_CPS_CTRL0_REG, 0x20);
		if(ret < 0) {
			err("%s: Set SX9310_CPS_CTRL0_REG(0x%x) FAIL.(ret=%d)\n",
				__FUNCTION__, SX9310_CPS_CTRL0_REG, ret);
			return ret;
		}
		ret = i2c_write_reg_u8(g_i2c_client, SX9310_IRQ_ENABLE_REG, 0x00);
		if(ret < 0) {
			err("%s: Set SX9310_IRQ_ENABLE_REG(0x%x) FAIL.(ret=%d)\n",
				__FUNCTION__, SX9310_IRQ_ENABLE_REG, ret);
			return ret;
		}
		log("%s: Disable sar sensor!!\n", __FUNCTION__);
	}
	
	return 0;
}

static int sx9310_SAR_sensor_hw_get_proxuserful(uint8_t cs)
{
	int ret = 0;
	u8 msb = 0, lsb = 0;
	u8 cpsrd = 0;
	s32 useful;
	
	cpsrd = i2c_read_reg_u8(g_i2c_client, SX9310_CPSRD);
	if(cpsrd < 0){
		err("%s: Read SX9310_CPSRD ERROR. (REG:0x%X, error = %d)\n",
			__FUNCTION__, SX9310_CPSRD, cpsrd);
		return ret;
	}
	
	ret = i2c_write_reg_u8(g_i2c_client, SX9310_CPSRD, cs);
	if(ret < 0) {
		err("%s: Set channel %u ERROR. (REG:0x%X, error = %d)\n",
			__FUNCTION__, cs, SX9310_CPSRD, ret);
		return ret;
	}
	msb = i2c_read_reg_u8(g_i2c_client, SX9310_USEMSB);
	if (msb < 0) {
		err("%s: Read SX9310_USEMSB(0x%02x) FAIL.(err=%d)\n",
			__FUNCTION__, SX9310_USEMSB, msb);
		return msb;
	}
	lsb = i2c_read_reg_u8(g_i2c_client, SX9310_USELSB);
	if (lsb < 0) {
		err("%s: Read SX9310_USELSB(0x%02x) FAIL.(err=%d)\n",
			__FUNCTION__, SX9310_USELSB, lsb);
		return lsb;
	}
	useful = (s32)((msb << 8) | lsb);
	log("%s: [CS:%d] Useful = %d\n", __FUNCTION__, cs, useful);
	
	ret = i2c_write_reg_u8(g_i2c_client, SX9310_CPSRD, cpsrd);
	if(ret < 0) {
		err("%s: Recovery SX9310_CPSRD ERROR. (REG:0x%X, error = %d)\n",
			__FUNCTION__, SX9310_CPSRD, ret);
		return ret;
	}

	return useful;
}

static int sx9310_SAR_sensor_hw_read_rawData(void)
{
	int ret = 0;
	u8 msb = 0, lsb = 0;
	unsigned int ii;
	u8 cpsrd = 0;
	s32 useful;
	s32 average;
	s32 diff;
	u16 offset;
	
	cpsrd = i2c_read_reg_u8(g_i2c_client, SX9310_CPSRD);
	if(cpsrd < 0){
		err("%s: Read SX9310_CPSRD ERROR. (REG:0x%X, error = %d)\n",
			__FUNCTION__, SX9310_CPSRD, cpsrd);
		return ret;
	}
	
	for (ii = 0; ii < USE_CHANNEL_NUM; ii++) {
		/* here to check the CSx */
		ret = i2c_write_reg_u8(g_i2c_client, SX9310_CPSRD, ii);
		if(ret < 0) {
			err("%s: Set channel %u ERROR. (REG:0x%X, error = %d)\n",
				__FUNCTION__, ii, SX9310_CPSRD, ret);
			return ret;
		}
		msleep(100);
		msb = i2c_read_reg_u8(g_i2c_client, SX9310_USEMSB);
		if (msb < 0) {
			err("%s: Read SX9310_USEMSB(0x%02x) FAIL.(err=%d)\n",
				__FUNCTION__, SX9310_USEMSB, msb);
			return msb;
		}
		lsb = i2c_read_reg_u8(g_i2c_client, SX9310_USELSB);
		if (lsb < 0) {
			err("%s: Read SX9310_USELSB(0x%02x) FAIL.(err=%d)\n",
				__FUNCTION__, SX9310_USELSB, lsb);
			return lsb;
		}
		useful = (s32)((msb << 8) | lsb);
		msb = i2c_read_reg_u8(g_i2c_client, SX9310_AVGMSB);
		if (msb < 0) {
			err("%s: Read SX9310_AVGMSB(0x%02x) FAIL.(err=%d)\n",
				__FUNCTION__, SX9310_AVGMSB, msb);
			return msb;
		}
		lsb = i2c_read_reg_u8(g_i2c_client, SX9310_AVGLSB);
		if (lsb < 0) {
			err("%s: Read SX9310_AVGLSB(0x%02x) FAIL.(err=%d)\n",
				__FUNCTION__, SX9310_AVGLSB, lsb);
			return lsb;
		}
		average = (s32)((msb << 8) | lsb);
		msb = i2c_read_reg_u8(g_i2c_client, SX9310_DIFFMSB);
		if (msb < 0) {
			err("%s: Read SX9310_DIFFMSB(0x%02x) FAIL.(err=%d)\n",
				__FUNCTION__, SX9310_DIFFMSB, msb);
			return msb;
		}
		lsb = i2c_read_reg_u8(g_i2c_client, SX9310_DIFFLSB);
		if (lsb < 0) {
			err("%s: Read SX9310_DIFFLSB(0x%02x) FAIL.(err=%d)\n",
				__FUNCTION__, SX9310_DIFFLSB, lsb);
			return lsb;
		}
		diff = (s32)((msb << 8) | lsb);
		msb = i2c_read_reg_u8(g_i2c_client, SX9310_OFFSETMSB);
		if (msb < 0) {
			err("%s: Read SX9310_OFFSETMSB(0x%02x) FAIL.(err=%d)\n",
				__FUNCTION__, SX9310_OFFSETMSB, msb);
			return msb;
		}
		lsb = i2c_read_reg_u8(g_i2c_client, SX9310_OFFSETLSB);
		if (lsb < 0) {
			err("%s: Read SX9310_OFFSETLSB(0x%02x) FAIL.(err=%d)\n",
				__FUNCTION__, SX9310_OFFSETLSB, lsb);
			return lsb;
		}
		offset = (u16)((msb << 8) | lsb);
		if (useful > 32767)
			useful -= 65536;
		if (average > 32767)
			average -= 65536;
		if (diff > 32767)
			diff -= 65536;
		log("%s: [CS:%d] Useful = %d Average = %d, DIFF = %d Offset = %d \n",
			__FUNCTION__, ii, useful, average, diff, offset);
	}
	
	ret = i2c_write_reg_u8(g_i2c_client, SX9310_CPSRD, cpsrd);
	if(ret < 0) {
		err("%s: Recovery SX9310_CPSRD ERROR. (REG:0x%X, error = %d)\n",
			__FUNCTION__, SX9310_CPSRD, ret);
		return ret;
	}
	
	return ret;
}

/* Get manual offset calibration Info. */
static int sx9310_SAR_sensor_hw_get_manual_offset_calibration(void)
{
	u8 reg_value = 0;
	
	reg_value = i2c_read_reg_u8(g_i2c_client, SX9310_IRQSTAT_REG);
	if(reg_value < 0) {
		err("%s: Get SX9310_IRQSTAT_REG(0x%02x) FAIL.(err=%d)\n",
			__FUNCTION__, SX9310_IRQSTAT_REG, reg_value);
		return reg_value;
	}
	log("%s: Manual offset calibration(reg=0x%02x): 0x%02x).\n",
		__FUNCTION__, SX9310_IRQSTAT_REG, reg_value);
	
	return reg_value;
}

/* Manual offset calibration . */
static int sx9310_SAR_sensor_hw_manual_offset_calibration(int val)
{
	int ret = 0;
	if(val) {
		ret = i2c_write_reg_u8(g_i2c_client, SX9310_IRQSTAT_REG, 0xFF);
		if(ret < 0) {
			err("%s: Set register(0x%02x) FAIL.(ret=%d)\n",
				__FUNCTION__, SX9310_IRQSTAT_REG, ret);
			return ret;
		}
		log("%s: Manual offset calibration(reg=0x%02x) to 0x%x\n",
			__FUNCTION__, SX9310_IRQSTAT_REG, 0xFF);
	} else {
		err("%s: Don't offset calibration data(val=%d).", val);
	}
	return ret;
}

static struct SAR_sensor_hw SAR_sensor_hw_sx9310 = {	
	.vendor = "Semtech",
	.module_number = "sx9310",

	.SAR_sensor_hw_check_ID              = sx9310_SAR_sensor_hw_check_ID,
	.SAR_sensor_hw_init                  = sx9310_SAR_sensor_hw_init,
	.SAR_sensor_hw_read_regStat          = sx9310_SAR_sensor_hw_read_regStat,
	.SAR_sensor_hw_get_interrupt         = sx9310_SAR_sensor_hw_get_interrupt,
	.SAR_sensor_hw_show_allreg           = sx9310_SAR_sensor_hw_show_allreg,
	.SAR_sensor_hw_set_register          = sx9310_SAR_sensor_hw_set_register,
	.SAR_sensor_hw_get_register          = sx9310_SAR_sensor_hw_get_register,
	.SAR_sensor_hw_turn_onoff            = sx9310_SAR_sensor_hw_turn_onoff,
	.SAR_sensor_hw__get_proxuserful      = sx9310_SAR_sensor_hw_get_proxuserful,
	.SAR_sensor_hw_read_rawData          = sx9310_SAR_sensor_hw_read_rawData,
	.SAR_sensor_hw_get_manual_offset_cal = sx9310_SAR_sensor_hw_get_manual_offset_calibration,
	.SAR_sensor_hw_manual_offset_cal     = sx9310_SAR_sensor_hw_manual_offset_calibration,
};

SAR_sensor_hw* SAR_sensor_hw_sx9310_getHardware(void)
{
	SAR_sensor_hw* SAR_sensor_hw_client = NULL;
	SAR_sensor_hw_client = &SAR_sensor_hw_sx9310;
	return SAR_sensor_hw_client;
}

