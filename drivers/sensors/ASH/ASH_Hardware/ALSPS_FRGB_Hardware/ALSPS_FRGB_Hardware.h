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
/* ALSPS FRGB Sensor Hardware Module */
/************************************************/
#ifndef __LINUX_ALSPSFRGB_HARDWARE_H
#define __LINUX_ALSPSFRGB_HARDWARE_H

/*********************************************/
/* ALSPS FRGB Sensor Configuration */
/*******************************************/
enum hardware_source {
	ALSPS_FRGB_hw_source_vcnl36863=0,
	ALSPS_FRGB_hw_source_max,
};

#include <linux/input/ASH.h>
extern ALSPS_FRGB_hw* ALSPS_FRGB_hw_vcnl36863_getHardware(void);

#endif

