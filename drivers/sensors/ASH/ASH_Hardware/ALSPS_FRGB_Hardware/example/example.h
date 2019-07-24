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
#ifndef __LINUX_IRSENSOR_EXAMPLE_H
#define __LINUX_IRSENSOR_EXAMPLE_H

#define EXAMPLE_PROXIMITY_THDL_DEFAULT  	(20)
#define EXAMPLE_PROXIMITY_THDH_DEFAULT  	(50)
#define EXAMPLE_PROXIMITY_INF_DEFAULT  		(2)
#define EXAMPLE_PROXIMITY_AUTOK_MIN  		(3)
#define EXAMPLE_PROXIMITY_AUTOK_MAX  		(50)

#define EXAMPLE_LIGHT_MAX_THRESHOLD		(65535)
#define EXAMPLE_LIGHT_200LUX_DEFAULT		(200)
#define EXAMPLE_LIGHT_1000LUX_DEFAULT		(1000)

#define EXAMPLE_FRGB_RED_RATIO_DEFAULT	(1)
#define EXAMPLE_FRGB_GREEN_RATIO_DEFAULT	(1)
#define EXAMPLE_FRGB_BLUE_RATIO_DEFAULT	(1)

#define EXAMPLE_NUM_REGS							(13)

#endif
