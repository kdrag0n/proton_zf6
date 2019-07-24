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
/* ALSPS Sensor Hardware Module */
/******************************/
#ifndef __LINUX_ALSPS_CM36686_H
#define __LINUX_ALSPS_CM36686_H

#define CM36686_PROXIMITY_THDL_DEFAULT  	(12)
#define CM36686_PROXIMITY_THDH_DEFAULT  	(30)
#define CM36686_PROXIMITY_INF_DEFAULT  		(2)
#define CM36686_PROXIMITY_AUTOK_MIN  		(3)
#define CM36686_PROXIMITY_AUTOK_MAX  		(50)
#define CM36656_LIGHT_CALIBRATION_DEFAULT (2701)
#define CM36686_LIGHT_MAX_THRESHOLD		(65534)
#define CM36686_NUM_REGS			(13)

/*Define Command Code*/
#define		ALS_CONF	0x00
#define		ALS_THDH  	0x01
#define		ALS_THDL		0x02
#define		PS_CONF1    	0x03
#define		PS_CONF3    	0x04
#define		PS_CANC      	0x05
#define		PS_THDL      	0x06
#define		PS_THDH      	0x07
#define		PS_DATA      	0x08
#define		ALS_DATA    	0x09
#define		INT_FLAG    	0x0B
#define		ID_REG        	0x0C

/*for ALS CONF command*/
//Integration
#define CM36686_ALS_IT_MAX	(3)
#define CM36686_ALS_IT_MASK	0x3F
#define CM36686_ALS_IT_SHIFT	(6)
#define CM36686_ALS_IT_80MS 	(0)
#define CM36686_ALS_IT_160MS 	(1)
#define CM36686_ALS_IT_320MS 	(2)
#define CM36686_ALS_IT_640MS 	(3)

//Persistence
#define CM36686_ALS_PERS_MAX		(3)
#define CM36686_ALS_PERS_MASK		0xF3
#define CM36686_ALS_PERS_SHIFT		(2)
#define CM36686_ALS_PERS_1 		(0)
#define CM36686_ALS_PERS_2 		(1)
#define CM36686_ALS_PERS_4 		(2)
#define CM36686_ALS_PERS_8 		(3)

#define CM36686_ALS_INT_EN	 	(1 << 1) /*enable/disable Interrupt*/
#define CM36686_ALS_INT_MASK		0xFD
#define CM36686_ALS_SD			(1 << 0) /*enable/disable ALS func, 1:disable , 0: enable*/
#define CM36686_ALS_SD_MASK		0xFE

/*for PS CONF1 command*/
#define CM36686_PS_12BITS	 (0 << 11)
#define CM36686_PS_16BITS    (1 << 11)
#define CM36686_PS_INT_OFF	       (0 << 8) /*enable/disable Interrupt*/
#define CM36686_PS_INT_IN          (1 << 8)
#define CM36686_PS_INT_OUT         (2 << 8)
#define CM36686_PS_INT_IN_AND_OUT  (3)

#define CM36686_PS_INT_MASK   0xFC

//LED Duty Ratio
#define CM36686_PS_DR_MAX		(3)
#define CM36686_PS_DR_MASK		0x3F
#define CM36686_PS_DR_SHIFT		(6)
#define CM36686_PS_DR_1_40   		(0)
#define CM36686_PS_DR_1_80   		(1)
#define CM36686_PS_DR_1_160  		(2)
#define CM36686_PS_DR_1_320  		(3)

//Persistence
#define CM36686_PS_PERS_MAX		(3)
#define CM36686_PS_PERS_MASK		0xCF
#define CM36686_PS_PERS_SHIFT		(4)
#define CM36686_PS_PERS_1 	 		(0)
#define CM36686_PS_PERS_2 	 		(1)
#define CM36686_PS_PERS_3 	 		(2)
#define CM36686_PS_PERS_4 	 		(3)

//Integration
#define CM36686_PS_IT_MAX			(7)
#define CM36686_PS_IT_MASK		0xF1
#define CM36686_PS_IT_SHIFT		(1)
#define CM36686_PS_IT_1T 	   		(0)
#define CM36686_PS_IT_1_5T   		(1)
#define CM36686_PS_IT_2T 	 		(2)
#define CM36686_PS_IT_2_5T 		(3)
#define CM36686_PS_IT_3T 		 	(4)
#define CM36686_PS_IT_3_5T 		(5)
#define CM36686_PS_IT_4T 		 	(6)
#define CM36686_PS_IT_8T 		 	(7)

#define CM36686_PS_SD			(1 << 0)/*enable/disable PS func, 1:disable , 0: enable*/
#define CM36686_PS_SD_MASK	 0xFE

/*for PS CONF3 command*/
#define CM36686_PS_MS_NORMAL		(0 << 6)
#define CM36686_PS_MS_LOGIC_ENABLE	(1 << 6)

//LED Current
#define CM36686_LED_I_MAX			(7)
#define CM36686_LED_I_MASK		0xF8
#define CM36686_LED_I_SHIFT		(0)
#define CM36686_LED_I_50            		(0)
#define CM36686_LED_I_75            		(1)
#define CM36686_LED_I_100            	(2)
#define CM36686_LED_I_120            	(3)
#define CM36686_LED_I_140            	(4)
#define CM36686_LED_I_160            	(5)
#define CM36686_LED_I_180            	(6)
#define CM36686_LED_I_200            	(7)

#define CM36686_PS_SMART_PERS_ENABLE	(1 << 4)
#define CM36686_PS_ACTIVE_FORCE_MODE	(1 << 3)
#define CM36686_PS_ACTIVE_FORCE_TRIG	(1 << 2)

/*for INT FLAG*/
#define INT_FLAG_PS_SPFLAG		(1<<6)
#define INT_FLAG_ALS_IF_L		(1<<5)
#define INT_FLAG_ALS_IF_H		(1<<4)
#define INT_FLAG_PS_IF_CLOSE	(1<<1)
#define INT_FLAG_PS_IF_AWAY	(1<<0)  

#endif
