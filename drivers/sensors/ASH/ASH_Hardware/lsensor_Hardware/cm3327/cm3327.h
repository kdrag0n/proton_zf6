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
#ifndef __LINUX_IRSENSOR_CM36686_H
#define __LINUX_IRSENSOR_CM36686_H

#define CM3327_LIGHT_200LUX_DEFAULT		(550)
#define CM3327_LIGHT_1000LUX_DEFAULT	(2701)
#define CM3327_LIGHT_MAX_THRESHOLD		(65534)
#define CM3327_NUM_REGS				(16)

/*Define Command Code*/
#define		ALS_CONF	0x00
#define		ALS_THDH  	0x02
#define		ALS_THDL	0x03
#define		R_DATA		0x08
#define		G_DATA		0x09
#define		B_DATA		0x0A
#define		IR_DATA    	0x0B
#define		INT_FLAG    	0x0D
#define		ID_REG        	0x0C

/*for ALS CONF command*/
//Integration
//#define CM3327_RGBIR_IT_MAX		(3)
#define CM3327_RGBIR_IT_MASK	0xE7
#define CM3327_RGBIR_IT_SHIFT	(3)
#define CM3327_RGBIR_IT_60MS 	(0)
#define CM3327_RGBIR_IT_120MS 	(1)
#define CM3327_RGBIR_IT_240MS 	(2)
#define CM3327_RGBIR_IT_480MS 	(3)

//Persistence
//#define CM3327_ALS_PERS_MAX	(3)
#define CM3327_ALS_PERS_MASK	0xF3
#define CM3327_ALS_PERS_SHIFT	(2)
#define CM3327_ALS_PERS_1 		(0)
#define CM3327_ALS_PERS_2 		(1)
#define CM3327_ALS_PERS_4 		(2)
#define CM3327_ALS_PERS_8 		(3)

#define CM3327_ALS_INT_EN	 	(1 << 0) /*enable/disable Interrupt*/
#define CM3327_ALS_INT_MASK		0xFE
#define CM3327_ALS_SD			(1 << 0) /*enable/disable ALS func, 1:disable , 0: enable*/
#define CM3327_ALS_SD_MASK		0xFE

/*for INT FLAG*/
#define INT_FLAG_ALS_IF_L            (1<<2)
#define INT_FLAG_ALS_IF_H            (1<<1)

#endif
