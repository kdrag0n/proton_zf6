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

 /*******************************/
/* IR Sensor Hardware Module */
/******************************/
#ifndef __LINUX_AP3045_H
#define __LINUX_AP3045_H

#define AP3045_PROXIMITY_THDL_DEFAULT  (300)
#define AP3045_PROXIMITY_THDH_DEFAULT  (500)
#define AP3045_LIGHT_200LUX_DEFAULT		(300)
#define AP3045_LIGHT_1000LUX_DEFAULT	(1400)
#define AP3045_LIGHT_MAX_THRESHOLD		(65535)

/*****************/
/*Command Code*/
/****************/
#define AP3045_NUM_CACHABLE_REGS	42

/* AP3045 System Register set */
#define AP3045_REG_SYS_CONF        	        0x00
#define AP3045_REG_SYS_INTSTATUS       0x01
#define AP3045_REG_SYS_INTCTRL            0x02
#define AP3045_REG_SYS_PID                     0x04
#define AP3045_REG_SYS_WAITTIME         0x06

/* AP3045 ALS Register set */
#define AP3045_REG_ALS_GAIN        		0x07
#define AP3045_REG_ALS_PERSIS      		0x08
#define AP3045_REG_ALS_TIME			0x0A
#define AP3045_REG_ALS_CH0_DATA_LOW       0x2C
#define AP3045_REG_ALS_CH0_DATA_HIGH       0x2D
#define AP3045_REG_ALS_CH1_DATA_LOW       0x2E
#define AP3045_REG_ALS_CH1_DATA_HIGH       0x2F
#define AP3045_REG_ALS_DATA_LOW       0x30
#define AP3045_REG_ALS_DATA_HIGH       0x31
#define AP3045_REG_ALS_THDL_L      		0x32
#define AP3045_REG_ALS_THDL_H      	0x33
#define AP3045_REG_ALS_THDH_L      	0x34
#define AP3045_REG_ALS_THDH_H      	0x35

/* AP3045 PS Register set */
#define AP3045_REG_PS_LEDG         		0x0C /*PS GAIN*/
#define AP3045_REG_PS_TIME			0x0f
#define AP3045_REG_PS_PERSIS       		0x0D
#define AP3045_REG_PS_LEDD         		0x10 /*PS LED DRIVER*/
#define AP3045_REG_PS_MEAN_TIME		0x1B
#define AP3045_REG_PS_DATA_LOW         0x26
#define AP3045_REG_PS_DATA_HIGH         0x27
#define AP3045_REG_PS_CAL_L        		0x3A
#define AP3045_REG_PS_CAL_H        		0x3B
#define AP3045_REG_PS_THDL_L       		0x36
#define AP3045_REG_PS_THDL_H       	        0x37
#define AP3045_REG_PS_THDH_L      	 	0x38
#define AP3045_REG_PS_THDH_H       		0x39

/***************/
/*Control Data*/
/**************/
/* SYS Interrupt Control */
#define AP3045_SYS_ICLEAN_AUTO 		0x00
#define AP3045_SYS_ICLEAN_MANUAL	0x01

/* SYS LED Control */
#define AP3045_SYS_LED_GAIN_MASK	0xF3
#define AP3045_SYS_LED_GAIN_SHIFT	(2)
#define AP3045_SYS_LED_G1        		0x00	/* 0 puls */
#define AP3045_SYS_LED_G2         		0x01	/* 1 puls (default)*/
#define AP3045_SYS_LED_G4        		0x02	/* 2 puls  */
#define AP3045_SYS_LED_G8         		0x03	/* 3 puls  */

#define AP3045_SYS_LED_CONTROL_SHIFT	(6)
#define AP3045_SYS_LED_MAX			(3)
#define AP3045_SYS_LED_005        		0x00	/* 16.7% */
#define AP3045_SYS_LED_050         		0x01	/* 33.3% */
#define AP3045_SYS_LED_075         		0x02	/* 66.7% */
#define AP3045_SYS_LED_100         		0x03	/* 100% (default)*/

/* SYS Turn ON/OFF */
#define AP3045_SYS_DEV_DOWN        	0x00
#define AP3045_SYS_ALS_ENABLE      	0x01
#define AP3045_SYS_PS_ENABLE     	  	0x02
#define AP3045_SYS_ALS_PS_ENABLE   	0x03
#define AP3045_SYS_DEV_RESET      	 	0x04

/* SYS interrupt State */
#define AP3045_REG_SYS_INT_SHIFT   		(0)
#define AP3045_REG_SYS_INT_PS_CLR		(2)
#define AP3045_REG_SYS_INT_LS_CLR		(1)
#define AP3045_REG_SYS_INT_MASK			0x03
#define AP3045_REG_SYS_INT_PMASK		0x02
#define AP3045_REG_SYS_INT_AMASK		0x01
#define AP3045_REG_SYS_INT_CHECK		0x00
#define AP3426_OBJ_MASK					0x10
#define AP3426_OBJ_SHIFT					(4)

/* PS Integration */
#define AP3045_PS_INTEG_MAX				(63)
#define AP3045_PS_INTEG					0x00 	/*0x00~0x3f*/
#define AP3045_PS_INTEG_3				0x03
#define AP3045_PS_INTEG_5				0x04
#define AP3045_PS_INTEG_17				0x10
#define AP3045_PS_INTEG_38				0x25

/* PS Persistence */
#define AP3045_PS_PERSIS_MAX			(63)
#define AP3045_PS_PERSIS					0x00 	/*0x00~0x3f*/
#define AP3045_PS_PERSIS_1				0x01

/* PS Time */
#define AP3045_PS_TIME_0         		0x00
#define AP3045_PS_TIME_1         		0x01
#define AP3045_PS_TIME_2         		0x02
#define AP3045_PS_TIME_3         		0x03

/* PS Threshold */
#define AP3045_REG_PS_THDL_L_SHIFT	(0)
#define AP3045_REG_PS_THDL_L_MASK	0xFF
#define AP3045_REG_PS_THDL_H_SHIFT	(2)
#define AP3045_REG_PS_THDL_H_MASK	0x03
#define AP3045_REG_PS_THDH_L_SHIFT	(0)
#define AP3045_REG_PS_THDH_L_MASK	0xFF
#define AP3045_REG_PS_THDH_H_SHIFT	(2)
#define AP3045_REG_PS_THDH_H_MASK	0x03

#define AP3045_REG_SYS_CONF_SHIFT	(0)
#define AP3045_REG_SYS_CONF_MASK	0x07

/* ALS Persistence */
#define AP3045_ALS_PERSIS_MAX		(63)
#define AP3045_ALS_PERSIS     			0x00 	/*0x00~0x3f*/

/* ALS Gain (Integration) */
#define AP3045_ALS_GAIN_MAX		(3)
#define AP3045_ALS_GAIN_MASK	(0xCF)
#define AP3045_ALS_GAIN_SHIFT	(4)
#define AP3045_ALS_GAIN_34304	(0)
#define AP3045_ALS_GAIN_8576		(1) 	
#define AP3045_ALS_GAIN_2144		(2) 	
#define AP3045_ALS_GAIN_536		(3) 	

#define AP3045_OBJ_MASK                  (0x10)
/*----------------------------------------------------------------------------*/

#define DISABLE                     0x00

#define ENABLE                      0x01

/*============================================================================*/

#endif
