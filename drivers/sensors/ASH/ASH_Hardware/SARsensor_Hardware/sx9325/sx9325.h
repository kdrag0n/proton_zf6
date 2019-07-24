/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */
#ifndef SX9325_H
#define SX9325_H

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#define USE_CHANNEL_NUM 4
#define SX932x_I2C_ADDR 0x95//0x28

/*
*  I2C Registers
*/
//-Interrupt and status
#define SX932x_IRQSTAT_REG		0x00
#define SX932x_STAT0_REG		0x01
#define SX932x_STAT1_REG		0x02
#define SX932x_STAT2_REG		0x03
#define SX932x_STAT3_REG		0x04
#define SX932x_IRQ_ENABLE_REG	0x05
#define SX932x_IRQCFG0_REG		0x06
#define SX932x_IRQCFG1_REG		0x07
#define SX932x_IRQCFG2_REG		0x08
//-General control
#define SX932x_CTRL0_REG		0x10
#define SX932x_CTRL1_REG		0x11
#define SX932x_I2CADDR_REG		0x14
#define SX932x_CLKSPRD			0x15
//-AFE Control
#define SX932x_AFE_CTRL0_REG	0x20
#define SX932x_AFE_CTRL1_REG	0x21
#define SX932x_AFE_CTRL2_REG	0x22
#define SX932x_AFE_CTRL3_REG	0x23
#define SX932x_AFE_CTRL4_REG	0x24
#define SX932x_AFE_CTRL5_REG	0x25
#define SX932x_AFE_CTRL6_REG	0x26
#define SX932x_AFE_CTRL7_REG	0x27
#define SX932x_AFE_PH0_REG		0x28
#define SX932x_AFE_PH1_REG		0x29
#define SX932x_AFE_PH2_REG		0x2A
#define SX932x_AFE_PH3_REG		0x2B
#define SX932x_AFE_CTRL8		0x2C
#define SX932x_AFE_CTRL9		0x2D
//-Main Digital Processing (Prox) control
#define SX932x_PROX_CTRL0_REG	0x30
#define SX932x_PROX_CTRL1_REG	0x31
#define SX932x_PROX_CTRL2_REG	0x32
#define SX932x_PROX_CTRL3_REG	0x33
#define SX932x_PROX_CTRL4_REG	0x34
#define SX932x_PROX_CTRL5_REG	0x35
#define SX932x_PROX_CTRL6_REG	0x36
#define SX932x_PROX_CTRL7_REG	0x37
//-Advanced Digital Processing control
#define SX932x_ADV_CTRL0_REG	0x40
#define SX932x_ADV_CTRL1_REG	0x41
#define SX932x_ADV_CTRL2_REG	0x42
#define SX932x_ADV_CTRL3_REG	0x43
#define SX932x_ADV_CTRL4_REG	0x44
#define SX932x_ADV_CTRL5_REG	0x45
#define SX932x_ADV_CTRL6_REG	0x46
#define SX932x_ADV_CTRL7_REG	0x47
#define SX932x_ADV_CTRL8_REG	0x48
#define SX932x_ADV_CTRL9_REG	0x49
#define SX932x_ADV_CTRL10_REG	0x4A
#define SX932x_ADV_CTRL11_REG	0x4B
#define SX932x_ADV_CTRL12_REG	0x4C
#define SX932x_ADV_CTRL13_REG	0x4D
#define SX932x_ADV_CTRL14_REG	0x4E
#define SX932x_ADV_CTRL15_REG	0x4F
#define SX932x_ADV_CTRL16_REG	0x50
#define SX932x_ADV_CTRL17_REG	0x51
#define SX932x_ADV_CTRL18_REG	0x52
#define SX932x_ADV_CTRL19_REG	0x53
#define SX932x_ADV_CTRL20_REG	0x54
/*      Sensor Readback */
#define SX932x_CPSRD			0x60
#define SX932x_USEMSB			0x61
#define SX932x_USELSB			0x62
#define SX932x_AVGMSB			0x63
#define SX932x_AVGLSB			0x64
#define SX932x_DIFFMSB			0x65
#define SX932x_DIFFLSB			0x66
#define SX932x_OFFSETMSB		0x67
#define SX932x_OFFSETLSB		0x68
#define SX932x_SARMSB			0x69
#define SX932x_SARLSB			0x6A

#define SX932x_SOFTRESET_REG	0x9F
#define SX932x_WHOAMI_REG		0xFA
#define SX932x_REV_REG			0xFB

/*      IrqStat 0:Inactive 1:Active     */
#define SX932x_IRQSTAT_RESET_FLAG		0x80
#define SX932x_IRQSTAT_TOUCH_FLAG		0x40
#define SX932x_IRQSTAT_RELEASE_FLAG		0x20
#define SX932x_IRQSTAT_COMPDONE_FLAG	0x10
#define SX932x_IRQSTAT_CONV_FLAG		0x08
#define SX932x_IRQSTAT_PROG2_FLAG		0x04
#define SX932x_IRQSTAT_PROG1_FLAG		0x02
#define SX932x_IRQSTAT_PROG0_FLAG		0x01


/* RegStat0  */
#define SX932x_PROXSTAT_PH3_FLAG		0x08
#define SX932x_PROXSTAT_PH2_FLAG		0x04
#define SX932x_PROXSTAT_PH1_FLAG		0x02
#define SX932x_PROXSTAT_PH0_FLAG		0x01

/*      SoftReset */
#define SX932x_SOFTRESET				0xDE
#define SX932x_WHOAMI_VALUE				0x95//0x22  //just for sx9325
#define SX932x_REV_VALUE				0x22 //just for sx9325

#define LGE_SENSOR

/***
*   define platform data
*/
struct smtc_reg_data {
	uint8_t reg;
	uint8_t val;
};

static struct smtc_reg_data sx9325_i2c_data_readback_reg[] = {
	{
		.reg = SX932x_CPSRD,
		.val = 0x00,
	},
	{
		.reg = SX932x_USEMSB,
		.val = 0x00,
	},
	{
		.reg = SX932x_USELSB,
		.val = 0x00,
	},
	{
		.reg = SX932x_AVGMSB,
		.val = 0x00,
	},
	{
		.reg = SX932x_AVGLSB,
		.val = 0x00,
	},
	{
		.reg = SX932x_DIFFMSB,
		.val = 0x00,
	},
	{
		.reg = SX932x_DIFFLSB,
		.val = 0x00,
	},
	{
		.reg = SX932x_OFFSETMSB,
		.val = 0x00,
	},
	{
		.reg = SX932x_OFFSETLSB,
		.val = 0x00,
	},
	{
		.reg = SX932x_SARMSB,
		.val = 0x00,
	},
	{
		.reg = SX932x_SARLSB,
		.val = 0x00,
	},
};

/* Define Registers that need to be initialized to values different than
* default
*/
static struct smtc_reg_data sx9325_i2c_reg_setup[] = {
//Interrupt and config
	{
		.reg = SX932x_IRQ_ENABLE_REG,	//0x05
		.val = 0x70,					// Enavle Close and Far -> enable compensation interrupt
	},
	{
		.reg = SX932x_IRQCFG0_REG, 		//0x06
		.val = 0x00,       				//
	},
	{
		.reg = SX932x_IRQCFG1_REG,		//0x07  
		.val = 0x00,
	},
	{
		.reg = SX932x_IRQCFG2_REG,		//0x08
		.val = 0x00,					//Activ Low
	},
	//--------General control
	{
		.reg = SX932x_CTRL0_REG,    //0x10
		.val = 0x16,       // Scanperiod : 100ms(10110)
	},
	{
		.reg = SX932x_I2CADDR_REG,   //0x14
		.val = 0x00,       //I2C Address : 0x28
	},
	{
		.reg = SX932x_CLKSPRD,    //0x15
		.val = 0x00,       //
	},
	//--------AFE Control
	{
		.reg = SX932x_AFE_CTRL0_REG,   //0x20
		.val = 0x00,       // CSx pin during sleep mode : HZ
	},
	{
		.reg = SX932x_AFE_CTRL1_REG,   //0x21
		.val = 0x10,       //reserved
	},
	{
		.reg = SX932x_AFE_CTRL2_REG,   //0x22
		.val = 0x00,       //reserved
	},
	{
		.reg = SX932x_AFE_CTRL3_REG,   //0x23
		.val = 0x00,       //Analog Range(ph0/1) : Small
	},
	{
		.reg = SX932x_AFE_CTRL4_REG,   //0x24
		.val = 0x44,       //Sampling Freq(ph0/1) : 83.33khz(01000), Resolution(ph0/1) : 128(100)
	},
	{
		.reg = SX932x_AFE_CTRL5_REG,   //0x25
		.val = 0x00,       //reserved
	},
	{
		.reg = SX932x_AFE_CTRL6_REG,   //0x26
		.val = 0x01,       //big//Analog Range(ph2/3) : Small
	},
	{
		.reg = SX932x_AFE_CTRL7_REG,   //0x27
		.val = 0x44,       //Sampling Freq(ph2/3) : 83.33khz(01000), Resolution(ph2/3) : 128(100)
	},
	{
		.reg = SX932x_AFE_PH0_REG,   //0x28
		.val = 0x04,       // CS2:HZ CS1:Input CS0 :HZ
	},
	{
		.reg = SX932x_AFE_PH1_REG,     //0x29
		.val = 0x10,       // CS2:Input CS1:HZ Shield CS0 :HZ
	},
	{
		.reg = SX932x_AFE_PH2_REG,   //0x2A
		.val = 0x1B,       //CS2:HZ CS1:HZ CS0 :HZ  
	},
	{
		.reg = SX932x_AFE_PH3_REG,   //0x2B
		.val = 0x00,       //CS2:HZ CS1:HZ CS0 :HZ
	},
	{
		.reg = SX932x_AFE_CTRL8,    //0x2C
		.val = 0x12,       // input register(kohm) 4(0010)
	},
	{
		.reg = SX932x_AFE_CTRL9,    //0x2D
		.val = 0x08,       // Analg gain : x1(1000)
	},
	//--------PROX control
	{
		.reg = SX932x_PROX_CTRL0_REG,  //0x30
		.val = 0x09,       // Digital Gain(ph0/1) : off(001) Digital Filter(ph0/1) : 1-1/2(001)
	},
	{
		.reg = SX932x_PROX_CTRL1_REG,  //0x31
		.val = 0x09,       // Digital Gain(ph2/3) : off(001) Digital Filter(ph2/3) : 1-1/2(001)
	},
	{
		.reg = SX932x_PROX_CTRL2_REG,  //0x32
		.val = 0x08,       //AVGNEGTHRESH : 16384
	},
	{
		.reg = SX932x_PROX_CTRL3_REG,  // 0x33 
		.val = 0x20,       //AVGPOSTHRESH : 16384
	},
	{
		.reg = SX932x_PROX_CTRL4_REG,  //0x34 
		.val = 0x0C,       //AVGFREEZEDIS : on(0) ,AVGNEGFILT :1-1/2(001) ,AVGPOSFILT : 1-1/256(100)
	},
	{
		.reg = SX932x_PROX_CTRL5_REG,  //0x35
		.val = 0x00,       //FARCOND: PROXDIFF < (THRESH.HYST), HYST : None, CLOSEDEB : off ,FARDEB : off
	},
	{
		.reg = SX932x_PROX_CTRL6_REG,  //0x36
		.val = 0x1B,       // Prox Theshold(ph0/1) : 200
	},
	{
		.reg = SX932x_PROX_CTRL7_REG,  //0x37
		.val = 0x1B,       // Prox Theshold(ph2/3) : 200
	},
	//--------Advanced control (defult)
	{
		.reg = SX932x_ADV_CTRL0_REG,
		.val = 0x00,
	},
	{
		.reg = SX932x_ADV_CTRL1_REG,
		.val = 0x00,
	},
	{
		.reg = SX932x_ADV_CTRL2_REG,
		.val = 0x00,
	},
	{
		.reg = SX932x_ADV_CTRL3_REG,
		.val = 0x00,
	},
	{
		.reg = SX932x_ADV_CTRL4_REG,
		.val = 0x00,
	},
	{
		.reg = SX932x_ADV_CTRL5_REG,
		.val = 0x05,
	},
	{
		.reg = SX932x_ADV_CTRL6_REG,
		.val = 0x00,
	},
	{
		.reg = SX932x_ADV_CTRL7_REG,
		.val = 0x00,
	},
	{
		.reg = SX932x_ADV_CTRL8_REG,
		.val = 0x00,
	},
	{
		.reg = SX932x_ADV_CTRL9_REG,
		.val = 0x80,
	},
	{
		.reg = SX932x_ADV_CTRL10_REG,
		.val = 0x00,
	},
	{
		.reg = SX932x_ADV_CTRL11_REG,
		.val = 0x00,
	},
	{
		.reg = SX932x_ADV_CTRL12_REG,
		.val = 0x00,
	},
	{
		.reg = SX932x_ADV_CTRL13_REG,
		.val = 0x00,
	},
	{
		.reg = SX932x_ADV_CTRL14_REG,
		.val = 0x80,
	},
	{
		.reg = SX932x_ADV_CTRL15_REG,
		.val = 0x0C,
	},
	{
		.reg = SX932x_ADV_CTRL16_REG,
		.val = 0x00,
	},
	{
		.reg = SX932x_ADV_CTRL17_REG,
		.val = 0x00,
	},
	{
		.reg = SX932x_ADV_CTRL18_REG,
		.val = 0x00,
	},
	{
		.reg = SX932x_ADV_CTRL19_REG,
		.val = 0xF0,
	},
	{
		.reg = SX932x_ADV_CTRL20_REG,
		.val = 0xF0,
	},
	//--------Sensor enable
	{
		.reg = SX932x_CTRL1_REG,    //0x11
		.val = 0x24,       //enable PH2
	},
};
#endif
