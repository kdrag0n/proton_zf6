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
#ifndef __LINUX_AP3425_H
#define __LINUX_AP3425_H

#define AP3425_PROXIMITY_THDL_DEFAULT  (50)
#define AP3425_PROXIMITY_THDH_DEFAULT  (100)
#define AP3425_LIGHT_200LUX_DEFAULT		(300)
#define AP3425_LIGHT_1000LUX_DEFAULT		(1400)
#define AP3425_LIGHT_MAX_THRESHOLD		(65535)

/*****************/
/*Command Code*/
/****************/
#define AP3425_NUM_CACHABLE_REGS	29

/* ap3425 System Register set */
#define AP3425_REG_SYS_CONF        		0x00
#define AP3425_REG_SYS_INTSTATUS  0x01
#define AP3425_REG_SYS_INTCTRL     	0x02
#define AP3425_REG_SYS_PID     			0x04
#define AP3425_REG_SYS_WAITTIME    0x06

#define AP3425_REG_IR_DATA_LOW     	0x0A
#define AP3425_REG_IR_DATA_HIGH    	0x0B
#define AP3425_REG_ALS_DATA_LOW   	0x0C
#define AP3425_REG_ALS_DATA_HIGH  	0x0D
#define AP3425_REG_PS_DATA_LOW     	0x0E
#define AP3425_REG_PS_DATA_HIGH    	0x0F

/* ap3425 ALS Register set */
#define AP3425_REG_ALS_GAIN        		0x10 
#define AP3425_REG_ALS_PERSIS      	0x14
#define AP3425_REG_ALS_THDL_L      	0x1A
#define AP3425_REG_ALS_THDL_H      	0x1B
#define AP3425_REG_ALS_THDH_L      	0x1C
#define AP3425_REG_ALS_THDH_H      	0x1D

/* ap3425 PS Register set */
#define AP3425_REG_PS_LEDG         		0x20 /*PS GAIN*/
#define AP3425_REG_PS_LEDD         		0x21 /*PS LED DRIVER*/
#define AP3425_REG_PS_IFORM        		0x22 /* PS INT Mode*/
#define AP3425_REG_PS_MEAN         		0x23
#define AP3425_REG_PS_SMARTINT     	0x24 /* PS Smart INT for low power */
#define AP3425_REG_PS_INTEGR       		0x25
#define AP3425_REG_PS_PERSIS       		0x26
#define AP3425_REG_PS_CAL_L        		0x28
#define AP3425_REG_PS_CAL_H        		0x29
#define AP3425_REG_PS_THDL_L       		0x2A
#define AP3425_REG_PS_THDL_H       	0x2B
#define AP3425_REG_PS_THDH_L       	0x2C
#define AP3425_REG_PS_THDH_H       	0x2D

/***************/
/*Control Data*/
/**************/
/* SYS Interrupt Control */
#define AP3425_SYS_ICLEAN_AUTO 	0x00
#define AP3425_SYS_ICLEAN_MANUAL	0x01

/* SYS LED Control */
#define AP3425_SYS_LED_GAIN_MASK		0xF3
#define AP3425_SYS_LED_GAIN_SHIFT		(2)
#define AP3425_SYS_LED_G1        		0x00	/* 0 puls */
#define AP3425_SYS_LED_G2         		0x01	/* 1 puls (default)*/
#define AP3425_SYS_LED_G4        		0x02	/* 2 puls  */
#define AP3425_SYS_LED_G8         		0x03	/* 3 puls  */

#define AP3425_SYS_LED_MAX			(3)
#define AP3425_SYS_LED_167        		0x00	/* 16.7% */
#define AP3425_SYS_LED_333         		0x01	/* 33.3% */
#define AP3425_SYS_LED_667         		0x02	/* 66.7% */
#define AP3425_SYS_LED_1000         		0x03	/* 100% (default)*/

/* SYS Turn ON/OFF */
#define AP3425_SYS_DEV_DOWN        	0x00
#define AP3425_SYS_ALS_ENABLE      	0x01
#define AP3425_SYS_PS_ENABLE       	0x02
#define AP3425_SYS_ALS_PS_ENABLE   	0x03
#define AP3425_SYS_DEV_RESET       	0x04

/* SYS interrupt State */
#define AP3425_REG_SYS_INT_SHIFT   		(0)
#define AP3425_REG_SYS_INT_PS_CLR		(2)
#define AP3425_REG_SYS_INT_LS_CLR		(1)
#define AP3425_REG_SYS_INT_MASK		0x03
#define AP3425_REG_SYS_INT_PMASK		0x02
#define AP3425_REG_SYS_INT_AMASK		0x01
#define AP3426_OBJ_MASK					0x10
#define AP3426_OBJ_SHIFT					(4)

/* PS Integration */
#define AP3425_PS_INTEG_MAX			(63)
#define AP3425_PS_INTEG					0x00 	/*0x00~0x3f*/
#define AP3425_PS_INTEG_3				0x03
#define AP3425_PS_INTEG_5				0x04
#define AP3425_PS_INTEG_17			0x10
#define AP3425_PS_INTEG_38			0x25

/* PS Persistence */
#define AP3425_PS_PERSIS_MAX			(63)
#define AP3425_PS_PERSIS				0x00 	/*0x00~0x3f*/
#define AP3425_PS_PERSIS_1				0x01

/* PS MEAN */
#define AP3425_PS_MEAN_0         		0x00	/* 5ms @2T*/
#define AP3425_PS_MEAN_1         		0x01	/* 9.6ms @2T*/
#define AP3425_PS_MEAN_2         		0x02	/* 14.1ms @2T*/
#define AP3425_PS_MEAN_3         		0x03	/* 18.7ms @2T*/

/* PS Threshold */
#define AP3425_REG_PS_THDL_L_SHIFT	(0)
#define AP3425_REG_PS_THDL_L_MASK		0xFF
#define AP3425_REG_PS_THDL_H_SHIFT	(2)
#define AP3425_REG_PS_THDL_H_MASK	0x03
#define AP3425_REG_PS_THDH_L_SHIFT	(0)
#define AP3425_REG_PS_THDH_L_MASK	0xFF
#define AP3425_REG_PS_THDH_H_SHIFT	(2)
#define AP3425_REG_PS_THDH_H_MASK	0x03

#define AP3425_REG_SYS_CONF_SHIFT	(0)
#define AP3425_REG_SYS_CONF_MASK	0x07

/* ALS Persistence */
#define AP3425_ALS_PERSIS_MAX		(63)
#define AP3425_ALS_PERSIS     			0x00 	/*0x00~0x3f*/

/* ALS Gain (Integration) */
#define AP3425_ALS_GAIN_MAX		(3)
#define AP3425_ALS_GAIN_MASK	(0xCF)
#define AP3425_ALS_GAIN_SHIFT	(4)
#define AP3425_ALS_GAIN_34304	(0)
#define AP3425_ALS_GAIN_8576		(1) 	
#define AP3425_ALS_GAIN_2144		(2) 	
#define AP3425_ALS_GAIN_536		(3) 	

#define AP3426_OBJ_COMMAND	0x01


/* ap3426 data registers */
#define AP3426_REG_IR_DATA_LOW_SHIFT     (0)
#define AP3426_REG_IR_DATA_LOW_MASK 0xFF     
#define AP3426_REG_IR_DATA_HIGH_SHIFT    (0)
#define AP3426_REG_IR_DATA_HIGH_MASK    0x03
#define AP3426_REG_PS_DATA_LOW_SHIFT     (0)
#define	AL3426_REG_PS_DATA_LOW_MASK	   0xFF
#define AP3426_REG_PS_DATA_HIGH_SHIFT    (0)
#define	AL3426_REG_PS_DATA_HIGH_MASK	   0x03

/*----------------------------------------------------------------------------*/

//#define AP3426_REG_ALS_CAL         0x19
#define AP3426_REG_ALS_THDL_L_SHIFT	(0)
#define AP3426_REG_ALS_THDL_L_MASK	0xFF
#define AP3426_REG_ALS_THDL_H_SHIFT	(0)
#define AP3426_REG_ALS_THDL_H_MASK	0xFF
#define AP3426_REG_ALS_THDH_L_SHIFT	(0)
#define AP3426_REG_ALS_THDH_L_MASK	0xFF

#define AP3426_REG_ALS_THDH_H_SHIFT	(0)
#define AP3426_REG_ALS_THDH_H_MASK	0xFF
/*----------------------------------------------------------------------------*/

/* ap3426 PS CONFIG registers */
#define AP3426_REG_PS_CONF_SHIFT         (2) 
#define AP3426_REG_PS_CONF_MASK         0x0C 
#define AP3426_REG_PS_LEDD_SHIFT         (0) 
#define AP3426_REG_PS_LEDD_MASK         0x03
#define AP3426_REG_PS_MEAN_SHIFT         (0)
#define AP3426_REG_PS_MEAN_MASK         0x03




/*============================================================================*/

//SYSTEM MODE (AP3426_REG_SYS_CONF)


/*----------------------------------------------------------------------------*/

//INT FLAG BIT MASK
#define	AP3426_SYS_ALS_INT_TRI     0x01
#define	AP3426_SYS_PS_INT_TRI      0x02
#define	AP3426_SYS_PS_INT_OBJ      0x10
#define	AP3426_SYS_PS_INT_IROV     0x20

/*----------------------------------------------------------------------------*/



/*----------------------------------------------------------------------------*/

//ALS CONFIG

#define AP3426_ALS_RANGE_0         0x00	/* Full range 32768 lux (0.5lux/count) */

#define AP3426_ALS_RANGE_1         0x01	/* Full range 8192 lux */

#define AP3426_ALS_RANGE_2         0x02	/* Full range 2048 lux */

#define AP3426_ALS_RANGE_3         0x03	/* Full range 512 lux */

#define AP3426_ALS_RANGE_MASK		0x30

#define AP3426_ALS_RANGE_SHIFT	(4)

#define AP3426_ALS_PERSIST_MASK	0x0F


/*----------------------------------------------------------------------------*/

#define DISABLE                     0x00

#define ENABLE                      0x01

/*============================================================================*/







/*----------------------------------------------------------------------------*/

//PS Engineering Registers

#define AP3426_REG_PS_DC_1         0x30 /*Only in Engineering chip, couldn't find in datasheet*/

#define AP3426_REG_PS_DC_1_SHIFT         (0) 

#define AP3426_REG_PS_DC_1_MASK         0xFF 

#define AP3426_REG_PS_DC_2         0x32 /*Only in Engineering chip, couldn't find in datasheet*/

#define AP3426_REG_PS_DC_2_SHIFT         (0) 

#define AP3426_REG_PS_DC_2_MASK         0xFF 



/*----------------------------------------------------------------------------*/


#endif
