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
#ifndef __LINUX_ALSPSFRGB_VCNL36863_H
#define __LINUX_ALSPSFRGB_VCNL36863_H

#define VCNL36863_PROXIMITY_THDL_DEFAULT    (124)
#define VCNL36863_PROXIMITY_THDH_DEFAULT    (197)
#define VCNL36863_PROXIMITY_INF_DEFAULT     (90)
#define VCNL36863_PROXIMITY_AUTOK_MIN       (3)
#define VCNL36863_PROXIMITY_AUTOK_MAX       (250)
#define VCNL36863_LIGHT_CALIBRATION_DEFAULT (106)
#define VCNL36863_LIGHT_MAX_THRESHOLD       (65534)
#define VCNL36863_FRGB_RED_RATIO_DEFAULT    (1)
#define VCNL36863_FRGB_GREEN_RATIO_DEFAULT  (1)
#define VCNL36863_FRGB_BLUE_RATIO_DEFAULT   (1)
#define VCNL36863_NUM_REGS                  (17)

#define VCNL36863_I2C_NAME "vcnl36863"

/* Define Slave Address*/
#define	VCNL36863_SLAVE_ADDR 0x60

/* Define Chip ID */
#define VCNL36863_ID 0x63

/*Define Command Code*/
#define		CS_CONF1   0x00
#define		CS_CONF2   0x00
#define		CS_THDH    0x01
#define		CS_THDL    0x02
#define		PS_CONF1   0x03
#define		PS_CONF2   0x03
#define		PS_CONF3   0x04
#define		PS_CONF4   0x04

#define		PS_THDL    0x05
#define		PS_THDH    0x06
#define		PS_CANC    0x07

#define 	PS_AC      0x08

#define		CS_R_DATA  0xF0
#define		CS_G_DATA  0xF1
#define		CS_B_DATA  0xF2
#define		CS_IR_DATA 0xF3
#define		PS_DATA    0xF4

#define		INT_FLAG   0xF5
#define		ID_REG     0xF6

#define		PS_AC_DATA 0xF7

struct vcnl36863_reg {
	uint8_t reg;
};

static struct vcnl36863_reg vcnl36863_regs[] = {
	{.reg = CS_CONF1,  },  // 0x00 
	{.reg = CS_THDH,   },  // 0x01
	{.reg = CS_THDL,   },  // 0x02
	{.reg = PS_CONF1,  },  // 0x03
	{.reg = PS_CONF3,  },  // 0x04
	{.reg = PS_THDL,   },  // 0x05
	{.reg = PS_THDH,   },  // 0x06
	{.reg = PS_CANC,   },  // 0x07
	{.reg = PS_AC,     },  // 0x08
	{.reg = CS_R_DATA, },  // 0xF0
	{.reg = CS_G_DATA, },  // 0xF1
	{.reg = CS_B_DATA, },  // 0xF2
	{.reg = CS_IR_DATA,},  // 0xF3
	{.reg = PS_DATA,   },  // 0xF4
	{.reg = INT_FLAG,  },  // 0xF5
	{.reg = ID_REG,    },  // 0xF6
	{.reg = PS_AC_DATA,},  // 0xF7
};

/**
 * for ALS CONF command
 **/
 
/*** CS CONF1 ***/
//PS start
#define VCNL36863_CS_START_MASK  0x7F
#define VCNL36863_CS_START_SHIFT (7)
#define VCNL36863_CS_START       (1)

//Integration
#define VCNL36863_CS_IT_MAX      (3)
#define VCNL36863_CS_IT_MASK     0xF3
#define VCNL36863_CS_IT_NOT_MASK 0x0C
#define VCNL36863_CS_IT_SHIFT    (2)
#define VCNL36863_CS_IT_50MS     (0)
#define VCNL36863_CS_IT_100MS    (1)
#define VCNL36863_CS_IT_200MS    (2)
#define VCNL36863_CS_IT_400MS    (3)

#define VCNL36863_CS_STANDBY      (1 << 1)
#define VCNL36863_CS_STANDBY_MASK 0xFD

#define VCNL36863_CS_SD      (1 << 0) /*enable/disable ALS func, 1:disable , 0: enable*/
#define VCNL36863_CS_SD_MASK 0xFE

/*** CS CONF2 ***/
//Persistence
#define VCNL36863_CS_PERS_MAX   (3)
#define VCNL36863_CS_PERS_MASK  0xF3
#define VCNL36863_CS_PERS_SHIFT (2)
#define VCNL36863_CS_PERS_1     (0)
#define VCNL36863_CS_PERS_2     (1)
#define VCNL36863_CS_PERS_4     (2)
#define VCNL36863_CS_PERS_8     (3)

#define VCNL36863_CS_START2_MASK  0xFD
#define VCNL36863_CS_START2_SHIFT (1)
#define VCNL36863_CS_START2       (1)

#define VCNL36863_CS_INT_EN   (1 << 0) /*enable/disable Interrupt*/
#define VCNL36863_CS_INT_MASK 0xFE

/**
 * for PS CONF command
 **/
 
/*** PS CONF1 ***/
//LED Duty Ratio
#define VCNL36863_PS_DR_MAX    (3)
#define VCNL36863_PS_DR_MASK   0x3F
#define VCNL36863_PS_DR_SHIFT  (6)
#define VCNL36863_PS_PERIOD_10 (0)
#define VCNL36863_PS_PERIOD_20 (1)
#define VCNL36863_PS_PERIOD_40 (2)
#define VCNL36863_PS_PERIOD_80 (3)

//Persistence
#define VCNL36863_PS_PERS_MAX   (3)
#define VCNL36863_PS_PERS_MASK  0xCF
#define VCNL36863_PS_PERS_SHIFT (4)
#define VCNL36863_PS_PERS_1     (0)
#define VCNL36863_PS_PERS_2     (1)
#define VCNL36863_PS_PERS_3     (2)
#define VCNL36863_PS_PERS_4     (3)

#define VCNL36863_PS_INT_IN_AND_OUT (2 << 2) /*enable/disable Interrupt*/
#define VCNL36863_PS_INT_MASK       0xF3  /*enable/disable Interrupt*/

#define VCNL36863_PS_SMART_PERS      (1 << 1)
#define VCNL36863_PS_SMART_PERS_MASK 0xFD

#define VCNL36863_PS_SD      (1 << 0)/*enable/disable PS func, 1:disable , 0: enable*/
#define VCNL36863_PS_SD_MASK 0xFE

/*** PS CONF2 ***/
//Integration
#define VCNL36863_PS_IT_MAX   (3)
#define VCNL36863_PS_IT_MASK  0x3F
#define VCNL36863_PS_IT_SHIFT (6)
#define VCNL36863_PS_IT_1T    (0)
#define VCNL36863_PS_IT_2T    (1)
#define VCNL36863_PS_IT_4T    (2)
#define VCNL36863_PS_IT_8T    (3)

//Multi-Pulse setting
#define VCNL36863_PS_MPS_MAX   (3)
#define VCNL36863_PS_MPS_MASK  0xCF
#define VCNL36863_PS_MPS_SHIFT (4)
#define VCNL36863_PS_MPS_1     (0)
#define VCNL36863_PS_MPS_2     (1)
#define VCNL36863_PS_MPS_4     (2)
#define VCNL36863_PS_MPS_8     (3)

//PS start
#define VCNL36863_PS_START_MASK  0xF7
#define VCNL36863_PS_START_SHIFT (3)
#define VCNL36863_PS_START       (1)

//PS high gain mode enable
#define VCNL36863_PS_HG_ENABLE_MASK  0xFB
#define VCNL36863_PS_HG_ENABLE_SHIFT (2)
#define VCNL36863_PS_HG_ENABLE       (1)

#define VCNL36863_PS_INT_SEL      (1 << 0)
#define VCNL36863_PS_INT_SEL_MASK 0xFE

/*** PS CONF3 ***/
#define VCNL36863_PS_ACTIVE_FORCE_MODE_MASK  0xBF
#define VCNL36863_PS_ACTIVE_FORCE_MODE_SHIFT (6)
#define VCNL36863_PS_ACTIVE_FORCE_MODE       (1)

#define VCNL36863_PS_ACTIVE_FORCE_TRIG_MASK  0xDF
#define VCNL36863_PS_ACTIVE_FORCE_TRIG_SHIFT (5)
#define VCNL36863_PS_ACTIVE_FORCE_TRIG       (1)

#define VCNL36863_PS_START2_MASK   0xF7
#define VCNL36863_PS_START2_SHIFT  (3)
#define VCNL36863_PS_START2        (1)

/*** PS CONF4 ***/
#define VCNL36863_PS_SUNLIGHT_DEFAULT_MASK 0x1F
#define VCNL36863_LED_I_SHIFT              (5)
#define VCNL36863_PS_SUNLIGHT_DEFAULT      (7)

//LED Current
#define VCNL36863_VCSEL_I_MAX   (4)
#define VCNL36863_VCSEL_I_MASK  0xFC
#define VCNL36863_VCSEL_I_SHIFT (0)
#define VCNL36863_VCSEL_I_7mA   (0)
#define VCNL36863_VCSEL_I_11mA  (1)
#define VCNL36863_VCSEL_I_14mA  (2)
#define VCNL36863_VCSEL_I_17mA  (3)
#define VCNL36863_VCSEL_I_20mA  (4)

/*** INT FLAG ***/
#define INT_FLAG_PS_ACFLAG   (1 << 5)  //After PS finishing auto calibration, INT raise.
#define INT_FLAG_PS_SPFLAG   (1 << 4)  //PS entering protection mode
#define INT_FLAG_CS_IF_L     (1 << 3)
#define INT_FLAG_CS_IF_H     (1 << 2)
#define INT_FLAG_PS_IF_CLOSE (1 << 1)  //PS rises above PS_THDH INT trigger event
#define INT_FLAG_PS_IF_AWAY  (1 << 0)  //PS drops below PS_THDL INT trigger event

#endif
