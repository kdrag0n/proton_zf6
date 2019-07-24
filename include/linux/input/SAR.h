/* 
 * Copyright (C) 2019 ASUSTek Inc.
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

#ifndef __LINUX_SAR_H
#define __LINUX_SAR_H

#define NAME_SIZE	(10)

/**
 * Define the SAR report event values.
 * @SAR_SENSOR_REPORT_PS_CLOSE : report close event.
 * @SAE_SENSOR_REPORT_PS_AWAY : report away event.
 */
#define SAR_SENSOR_REPORT_CLOSE 			(1)
#define SAR_SENSOR_REPORT_AWAY     			(0)

#define OBJECT_NEAR			 (1)
#define OBJECT_FAR			 (0)

/**
 * Define the proxstat bit of SAR sensor, 
 * which indicates if proximity is currently detected by CSx.
 * @PROXSTAT0 : CS0 detect
 * @RROXSTAT1 : CS1 detect
 * @PROXSTAT2 : CS2 detect
 * @PROXSTAT3 : CS3 detect
 */
#define PROXSTAT0		(0x00)
#define PROXSTAT1		(0x02)
#define PROXSTAT2		(0x04)
#define PROXSTAT3		(0x08)	

/**
 * SAR_sensor_report_register - before report SAR sensor event
 * you need to register first. This will create input device for SAR sensor.
 */
extern int SAR_sensor_report_register(void);

/**
 * SAR_sensor_report_unregister - Remove the input device for SAR sensor.
 */
extern void SAR_sensor_report_unregister(void);

/**
 * SAR_sensor_report_abs - report the SAR abs.
 * @ abs=SAR_SENSOR_REPORT_CLOSE : report close event.
 * @ abs=SAR_SENSOR_REPORT_AWAY : report away event.
 */
extern void SAR_sensor_report_abs(int abs);

/**
 * SAR_sensor_GPIO
 */
 typedef struct SAR_sensor_GPIO {
	void (*SAR_sensor_isr)(void);
}SAR_sensor_GPIO;

/**
 * SAR_sensor_gpio_register - register the GPIO setting and set the IRQ handler.
 * Return the IRQ.
 */
#include <linux/i2c.h>
extern int SAR_sensor_gpio_register(struct i2c_client *client, SAR_sensor_GPIO *gpio_ist);

/**
 * SAR_sensor_gpio_unregister - unregister the GPIO setting.
 */
extern int SAR_sensor_gpio_unregister(int irq);

/**
 * SAR_sensor_gpio_value - return the gpio value.
 */
extern int SAR_sensor_gpio_value(void);

/**
SAR_sensor_I2C - We define functions for i2c driver.
 * @SAR_sensor_probe : It will be run when "i2c_add_driver" has been called.
 * @SAR_sensor_remove : Remove all Memory.
 * @SAR_sensor_shutdown : Turn off all sensors.
 * @SAR_sensor_suspend : Do what suspend should do.
 * @SAR_sensor_resume : Do what resume should do.
 */
typedef struct SAR_sensor_I2C {
	void (*SAR_sensor_probe)(struct i2c_client *client);
	void (*SAR_sensor_remove)(void);
	void (*SAR_sensor_shutdown)(void);
	void (*SAR_sensor_suspend)(void);
	void (*SAR_sensor_resume)(void);	
}SAR_sensor_I2C;

/**
 * SAR_sensor_hw - the i2c control functions for SAR sensor
 */
typedef struct SAR_sensor_hw {	
	char vendor[NAME_SIZE];
	char module_number[NAME_SIZE];

	int (*SAR_sensor_hw_check_ID)(void);	
	int (*SAR_sensor_hw_init)(struct i2c_client* client);
	int (*SAR_sensor_hw_read_regStat)(void);	
	int (*SAR_sensor_hw_get_interrupt)(void);	
	int (*SAR_sensor_hw_show_allreg)(void);	
	int (*SAR_sensor_hw_set_register)(uint8_t reg, int value);
	int (*SAR_sensor_hw_get_register)(uint8_t reg);
	int (*SAR_sensor_hw_turn_onoff)(bool bOn);
	int (*SAR_sensor_hw__get_proxuserful)(uint8_t cs);
	int (*SAR_sensor_hw_read_rawData)(void);
	int (*SAR_sensor_hw_get_manual_offset_cal)(void);
	int (*SAR_sensor_hw_manual_offset_cal)(int value);
}SAR_sensor_hw;

/**
 * SAR_sensor_ATTR_BMMI - attributes for SAR_sensor BMMI.
 */
typedef struct{ 
	bool (*SAR_sensor_show_atd_test)(void);
	int (*SAR_sensor_show_raw_data)(void);
}SAR_sensor_ATTR_BMMI;

/**
 * SAR_sensor_info_type - define the SAR_sensor information.
 * @vendor : ASUS.
 * @version : driver version.
 * @module_number : hardware chip serial number.
 */
#define NAME_SIZE	(10)
typedef struct{
	char vendor[NAME_SIZE];
	char module_number[NAME_SIZE];
}SAR_sensor_info_type;

/**
 * SAR_sensor_ATTR_HAL - attributes for SAR_sensor HAL function.
 */
typedef struct{
	bool (*SAR_sensor_show_switch_onoff)(void);
	int (*SAR_sensor_store_switch_onoff)(bool bOn);
	int (*SAR_sensor_show_Interrupt_detect_status)(void);
	bool (*SAR_sensor_show_sar_status)(void);
}SAR_sensor_ATTR_HAL;

/**
 * SAR_sensor_ATTR_Hardware - attributes for SAR_sensor hardware read/write.
 */
typedef struct{
	uint8_t show_reg_addr;
	int (*SAR_show_reg)(uint8_t addr);
	int (*SAR_store_reg)(uint8_t addr, int value);
	int (*SAR_sensor_show_manual_offset_cal)(void);
	int (*SAR_sensor_store_manual_offset_cal)(int value);
}SAR_sensor_ATTR_Hardware;

/**
 * SAR_sensor_ATTR_Extension - attributes for SAR_sensor extensive functions.
 */
typedef struct{	
	bool (*SAR_sensor_show_allreg)(void);
}SAR_sensor_ATTR_Extension;

/**
 * SAR_sensor_ATTR - attributes for lsensor.
 */
typedef struct{
	SAR_sensor_info_type 			*info_type;
	SAR_sensor_ATTR_BMMI 		*ATTR_BMMI;
	SAR_sensor_ATTR_HAL		*ATTR_HAL;
	SAR_sensor_ATTR_Hardware	*ATTR_Hardware;
	SAR_sensor_ATTR_Extension	*ATTR_Extension;
}SAR_sensor_ATTR;

/**
 * SAR_sensor_ATTR_register - assign a psensor only file node and create the attributes.
 *
 * The attributes will be created at /sys/class/sensors/SAR_sensor.
  */
extern int SAR_sensor_ATTR_register(SAR_sensor_ATTR *mATTR);

/**
 * SAR_sensor_ATTR_unregister - remove the SAR_sensor attributes.
 */
extern int SAR_sensor_ATTR_unregister(void);

/**
 * SAR_sensor_ATTR_create - create SAR_sensor customize attributes.
 * @mSAR_sensor_attr : the pointer of device_attribute, which you want to create attribute.
 */
extern int SAR_sensor_ATTR_create(struct device_attribute *mSAR_sensor_attr);


/**
 * SAR_sensor_i2c_register - struct SAR_sensor_I2C wrapped by i2c driver.
 */
extern int SAR_sensor_i2c_register(SAR_sensor_I2C *ir_i2c);

/**
 * SAR_sensor_hw_getHardware - Before this function, you SHOULD execute SAR_sensor_i2c_register first.
 */
extern SAR_sensor_hw* SAR_sensor_hw_getHardware(void);

/**
 * SAR_sensor_i2c_unregister - i2c_del_driver.
 */
extern int SAR_sensor_i2c_unregister(void);

#endif
