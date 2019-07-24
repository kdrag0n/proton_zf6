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
#ifndef __LINUX_IRSENSOR_CM36675_H
#define __LINUX_IRSENSOR_CM36675_H

#define CM36675_PROXIMITY_THDL_DEFAULT  	(12)
#define CM36675_PROXIMITY_THDH_DEFAULT  	(30)
#define CM36675_PROXIMITY_INF_DEFAULT  		(2)
#define CM36675_PROXIMITY_AUTOK_MIN  		(3)
#define CM36675_PROXIMITY_AUTOK_MAX  		(50)
#define CM36675_NUM_REGS					(13)

/*Define Command Code*/
#define		PS_CONF1    	0x03
#define		PS_CONF3    	0x04
#define		PS_CANC      	0x05
#define		PS_THDL      	0x06
#define		PS_THDH      	0x07
#define		PS_DATA      	0x08
#define		INT_FLAG    	0x0B
#define		ID_REG        	0x0C

/*for PS CONF1 command*/
#define CM36675_PS_INT_OFF			(0 << 8) /*enable/disable Interrupt*/
#define CM36675_PS_INT_IN			(1 << 8)
#define CM36675_PS_INT_OUT			(2 << 8)
#define CM36675_PS_INT_IN_AND_OUT	(3)
#define CM36675_PS_INT_MASK			0xFC

//LED Duty Ratio
#define CM36675_PS_DR_MAX		(3)
#define CM36675_PS_DR_MASK		0x3F
#define CM36675_PS_DR_SHIFT		(6)
#define CM36675_PS_PERIOD_16   	(0)
#define CM36675_PS_PERIOD_32  	(1)
#define CM36675_PS_PERIOD_64  	(2)
#define CM36675_PS_PERIOD_128  	(3)

//Persistence
#define CM36675_PS_PERS_MAX	(3)
#define CM36675_PS_PERS_MASK	0xCF
#define CM36675_PS_PERS_SHIFT	(4)
#define CM36675_PS_PERS_1 	 	(0)
#define CM36675_PS_PERS_2 	 	(1)
#define CM36675_PS_PERS_3 	 	(2)
#define CM36675_PS_PERS_4 	 	(3)

//Integration
#define CM36675_PS_IT_MAX		(7)
#define CM36675_PS_IT_MASK		0xF1
#define CM36675_PS_IT_SHIFT		(1)
#define CM36675_PS_IT_1T 	   		(0)
#define CM36675_PS_IT_2T 	 		(1)
#define CM36675_PS_IT_4T 		 	(2)
#define CM36675_PS_IT_8T 		 	(3)

#define CM36675_PS_SD	       (1 << 0)/*enable/disable PS func, 1:disable , 0: enable*/
#define CM36675_PS_SD_MASK	 0xFE

/*for PS CONF3 command*/
#define CM36675_PS_MS_NORMAL        (0 << 6)
#define CM36675_PS_MS_LOGIC_ENABLE  (1 << 6)

//LED Current
#define CM36675_LED_I_MAX		(7)
#define CM36675_LED_I_MASK		0xF8
#define CM36675_LED_I_SHIFT		(0)
#define CM36675_LED_I_60            	(0)
#define CM36675_LED_I_80            	(1)
#define CM36675_LED_I_110            	(2)
#define CM36675_LED_I_130            	(3)
#define CM36675_LED_I_150            	(4)
#define CM36675_LED_I_180            	(5)
#define CM36675_LED_I_200            	(6)
#define CM36675_LED_I_230            	(7)

#define CM36675_PS_SMART_PERS_ENABLE	(1 << 4)
#define CM36675_PS_ACTIVE_FORCE_MODE	(1 << 3)
#define CM36675_PS_ACTIVE_FORCE_TRIG	(1 << 2)

/*for INT FLAG*/
#define INT_FLAG_PS_SPFLAG		(1<<6)
#define INT_FLAG_PS_IF_CLOSE		(1<<1)
#define INT_FLAG_PS_IF_AWAY		(1<<0)  

#endif
