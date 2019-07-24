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

/**************************************************/
/* SAR Sensor Hardware Module */
/************************************************/
#ifndef __LINUX_SAR_SENSOR_HARDWARE_H
#define __LINUX_SAR_SENSOR_HARDWARE_H

/*********************************************/
/* SAR Sensor Configuration */
/*******************************************/
enum hardware_source {
	//SAR_sensor_hw_source_sx9310 = 0,
	SAR_sensor_hw_source_sx9325 = 0,
	SAR_sensor_hw_source_max,
};

#include <linux/input/ASH.h>
#ifdef USE_SX9310
extern SAR_sensor_hw* SAR_sensor_hw_sx9310_getHardware(void);
#endif
extern SAR_sensor_hw* SAR_sensor_hw_sx9325_getHardware(void);

#endif

