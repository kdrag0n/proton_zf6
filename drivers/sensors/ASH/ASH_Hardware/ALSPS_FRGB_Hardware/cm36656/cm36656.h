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
#ifndef __LINUX_ALSPSFRGB_CM36686_H
#define __LINUX_ALSPSFRGB_CM36686_H

#define CM36656_PROXIMITY_THDL_DEFAULT  	(91)
#define CM36656_PROXIMITY_THDH_DEFAULT  	(121)
#define CM36656_PROXIMITY_INF_DEFAULT  		(77)
#define CM36656_PROXIMITY_AUTOK_MIN  		(3)
#define CM36656_PROXIMITY_AUTOK_MAX  		(250)
#define CM36656_LIGHT_CALIBRATION_DEFAULT   (1588)
#define CM36656_LIGHT_MAX_THRESHOLD		(65534)
#define CM36656_FRGB_RED_RATIO_DEFAULT	(1)
#define CM36656_FRGB_GREEN_RATIO_DEFAULT	(1)
#define CM36656_FRGB_BLUE_RATIO_DEFAULT	(1)
#define CM36656_NUM_REGS					(15)

/*Define Command Code*/
#define		CS_CONF	0x00
#define		CS_THDH		0x01
#define		CS_THDL		0x02
#define		PS_CONF1	0x03
#define		PS_CONF2	0x03
#define		PS_CONF3	0x04
#define		PS_CONF4	0x04
#define		PS_THDL		0x05
#define		PS_THDH		0x06
#define		PS_CANC	0x07
#define		CS_R_DATA	0xF0
#define		CS_G_DATA	0xF1
#define		CS_B_DATA	0xF2
#define		CS_IR_DATA	0xF3
#define		PS_DATA	0xF4
#define		INT_FLAG	0xF5
#define		ID_REG		0xF6

/*for ALS CONF command*/
//Integration
#define CM36656_CS_IT_MAX		(3)
#define CM36656_CS_IT_MASK		0xF3
#define CM36656_CS_IT_SHIFT		(2)
#define CM36656_CS_IT_50MS		(0)
#define CM36656_CS_IT_100MS		(1)
#define CM36656_CS_IT_200MS		(2)
#define CM36656_CS_IT_400MS		(3)

//Persistence
#define CM36656_CS_PERS_MAX	(3)
#define CM36656_CS_PERS_MASK	0xF3
#define CM36656_CS_PERS_SHIFT	(2)
#define CM36656_CS_PERS_1 		(0)
#define CM36656_CS_PERS_2 		(1)
#define CM36656_CS_PERS_4 		(2)
#define CM36656_CS_PERS_8 		(3)

#define CM36656_CS_INT_EN	 	(1 << 0) /*enable/disable Interrupt*/
#define CM36656_CS_INT_MASK		0xFE
#define CM36656_CS_SD			(1 << 0) /*enable/disable ALS func, 1:disable , 0: enable*/
#define CM36656_CS_SD_MASK		0xFE

/*for PS CONF1 command*/
#define CM36656_PS_INT_IN_AND_OUT	(8)
#define CM36656_PS_INT_MASK			0xF7

//LED Duty Ratio
#define CM36656_PS_DR_MAX		(3)
#define CM36656_PS_DR_MASK		0x3F
#define CM36656_PS_DR_SHIFT		(6)
#define CM36656_PS_PERIOD_8   	(0)
#define CM36656_PS_PERIOD_16	(1)
#define CM36656_PS_PERIOD_32	(2)
#define CM36656_PS_PERIOD_64	(3)

//Persistence
#define CM36656_PS_PERS_MAX	(3)
#define CM36656_PS_PERS_MASK	0xCF
#define CM36656_PS_PERS_SHIFT	(4)
#define CM36656_PS_PERS_1		(0)
#define CM36656_PS_PERS_2		(1)
#define CM36656_PS_PERS_3		(2)
#define CM36656_PS_PERS_4		(3)

//Integration
#define CM36656_PS_IT_MAX		(3)
#define CM36656_PS_IT_MASK		0x3F
#define CM36656_PS_IT_SHIFT		(6)
#define CM36656_PS_IT_1T 	   		(0)
#define CM36656_PS_IT_2T 	 		(1)
#define CM36656_PS_IT_4T 		 	(2)
#define CM36656_PS_IT_8T 		 	(3)

//PS start
#define CM36656_PS_START_MASK		0xF7
#define CM36656_PS_START_SHIFT		(3)
#define CM36656_PS_START			(1)

#define CM36656_PS_SD	       (1 << 0)/*enable/disable PS func, 1:disable , 0: enable*/
#define CM36656_PS_SD_MASK	 0xFE

/*for PS CONF3 command*/
#define CM36656_PS_MS_NORMAL        (0 << 6)
#define CM36656_PS_MS_LOGIC_ENABLE  (1 << 6)

//CS CONF start bit
#define CM36656_CS_START_MASK	0x7F
#define CM36656_CS_START_SHIFT	(7)
#define CM36656_CS_START		(1)

//LED Current
#define CM36656_LED_I_MAX		(7)
#define CM36656_LED_I_MASK		0xF8
#define CM36656_LED_I_SHIFT		(0)
#define CM36656_LED_I_70            	(0)
#define CM36656_LED_I_95            	(1)
#define CM36656_LED_I_110            	(2)
#define CM36656_LED_I_130            	(3)
#define CM36656_LED_I_170            	(4)
#define CM36656_LED_I_200            	(5)
#define CM36656_LED_I_220            	(6)
#define CM36656_LED_I_240            	(7)

#define CM36656_PS_SMART_PERS_ENABLE  (1 << 4)
#define CM36656_PS_ACTIVE_FORCE_MODE  (1 << 3)
#define CM36656_PS_ACTIVE_FORCE_TRIG  (1 << 2)

/*for INT FLAG*/
#define INT_FLAG_PS_SPFLAG		(1<<6)
#define INT_FLAG_CS_IF_L			(1<<3)
#define INT_FLAG_CS_IF_H			(1<<2)
#define INT_FLAG_PS_IF_CLOSE		(1<<1)
#define INT_FLAG_PS_IF_AWAY		(1<<0)  

#endif
