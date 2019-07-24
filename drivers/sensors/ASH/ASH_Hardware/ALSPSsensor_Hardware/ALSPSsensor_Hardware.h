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
/* ALSPS Sensor Hardware Module */
/******************************/
#ifndef __LINUX_ALSPS_HARDWARE_H
#define __LINUX_ALSPS_HARDWARE_H

/****************************/
/* ALSPS Sensor Configuration */
/**************************/
enum hardware_source {
	ALSPS_hw_source_cm36686=0,
	ALSPS_hw_source_max,
};

#include <linux/input/ASH.h>
extern ALSPS_hw* ALSPS_hw_cm36686_getHardware(void);
extern ALSPS_hw* ALSPS_hw_ap3045_getHardware(void);
#endif

