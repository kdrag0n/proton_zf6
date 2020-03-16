/* 
 * Copyright (C) 2015 ASUSTek Inc.
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
#ifndef __LINUX_IRSENSOR_HARDWARE_H
#define __LINUX_IRSENSOR_HARDWARE_H

/****************************/
/* IR Sensor Configuration */
/**************************/
enum hardware_source {
	IRsensor_hw_source_cm36656=0,
	IRsensor_hw_source_max,
};

#include <linux/input/ASH.h>
extern IRsensor_hw* IRsensor_hw_cm36686_getHardware(void);
extern IRsensor_hw* IRsensor_hw_cm36656_getHardware(void);
extern IRsensor_hw* IRsensor_hw_ap3045_getHardware(void);
extern IRsensor_hw* Psensor_hw_cm36675_getHardware(void);
extern IRsensor_hw* Lsensor_hw_cm3327_getHardware(void);
#endif

