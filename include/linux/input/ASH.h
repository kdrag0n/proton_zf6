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
 
#ifndef __LINUX_ASH_H
#define __LINUX_ASH_H

#include <linux/input/SAR.h>

/**
 * ASH_type - define the sensor types ASH supports.
 */
typedef enum{
	psensor = 0,
	lsensor,
	frgbsensor,
	hallsensor,
	sarsensor,
}ASH_type;

/**
 * ASH_ATTR_device_create - create the sensor type attributes and return device struct,
 * 			which can used to create attributes. (/sys/class/sensors/)
 */
#include <linux/device.h>
#include <linux/fs.h>
extern struct device *ASH_ATTR_device_create(ASH_type type);

/**
 * ASH_ATTR_device_remove - remove the sensor type.
 */
extern void ASH_ATTR_device_remove(ASH_type type);

/**
 * HALLsensor_ATTR - for attributes  
 * @show_action_status : show the hall sensor status.
 * @show_hall_sensor_enable : true - HW on ; false - HW off.
 * @store_hall_sensor_enable : true - HW turn on ; false - HW turn off.
 * @show_hall_sensor_debounce : time in mini second(ms).
 * @store_hall_sensor_debounce : time in mini second(ms).
 */
typedef struct{
	bool (*show_action_status)(void);
	bool (*show_hall_sensor_enable)(void);
	int (*store_hall_sensor_enable)(bool bOn);
	int (*show_hall_sensor_debounce)(void);
	int (*store_hall_sensor_debounce)(int debounce);
	int (*show_hall_sensor_int_count)(void);
	int (*show_hall_sensor_event_count)(void);
	int (*show_hall_sensor_error_mesg)(char *error_mesg);
}HALLsensor_ATTR;

/**
 * HALLsensor_ATTR_register - assign a hall sensor file node and create the attributes.
 * @status : 1 - No magnetic ; 0 - Magnetic
 * @switch : on - HW on ; off - HW off
 * @debounce : time in mini second(ms)
 *
 * The attributes will be created at /sys/class/sensors/hallsensor.
 */
extern int HALLsensor_ATTR_register(HALLsensor_ATTR *mATTR);

/**
 * HALLsensor_ATTR_unregister - remove the hall sensor attributes.
 */
extern int HALLsensor_ATTR_unregister(void);

/**
 * HALLsensor_ATTR_create - create hall sensor customize attributes.
 * @mpsensor_attr : the pointer of device_attribute, which you want to create attribute.
 */
#include <linux/device.h>
extern int HALLsensor_ATTR_create(struct device_attribute *mhallsensor_attr);

/**
 * psensor_info_type - define the psensor information.
 * @vendor : ASUS.
 * @version : driver version.
 * @module_number : hardware chip serial number.
 */
#define NAME_SIZE	(10)
typedef struct{
	char vendor[NAME_SIZE];
	char module_number[NAME_SIZE];
}psensor_info_type;

/**
 * psensor_ATTR_Calibration - attributes for psensor calibration.
 */
typedef struct{	
	int (*proximity_show_calibration_hi)(void);
	int (*proximity_store_calibration_hi)(int calvalue);
	int (*proximity_show_calibration_lo)(void);
	int (*proximity_store_calibration_lo)(int calvalue);
	int (*proximity_show_calibration_inf)(void);
	int (*proximity_store_calibration_inf)(int calvalue); 
	int (*proximity_show_adc)(void);
}psensor_ATTR_Calibration;

/**
 * psensor_ATTR_BMMI - attributes for psensor BMMI.
 */
typedef struct{	
	bool (*proximity_show_atd_test)(void); 
}psensor_ATTR_BMMI;

/**
 * psensor_ATTR_Hardware - attributes for psensor hardware read/write.
 */
typedef struct{
	uint8_t show_reg_addr;
	int (*proximity_show_reg)(uint8_t addr);
	int (*proximity_store_reg)(uint8_t addr, int value);	
}psensor_ATTR_Hardware;

/**
 * psensor_ATTR_HAL - attributes for psensor HAL function.
 */
typedef struct{	
	bool (*proximity_show_switch_onoff)(void);
	int (*proximity_store_switch_onoff)(bool bOn);
	bool (*proximity_show_status)(void);
}psensor_ATTR_HAL;

#define ERROR_MESG_SIZE	(99)

/**
 * psensor_ATTR_Extension - attributes for psensor extensive functions.
 */
typedef struct{	
	bool (*proximity_show_allreg)(void);

	/*switch ON/OFF proximity polling adc*/
	bool (*proximity_show_polling_mode)(void);
	int (*proximity_store_polling_mode)(bool bOn);
	
	/*Switch ON/OFF proximity auto calibration*/
	bool (*proximity_show_autok)(void);
	int (*proximity_store_autok)(bool bOn);

	/*modify the auto calibration range*/
	int (*proximity_show_autokmin)(void);
	int (*proximity_store_autokmin)(int autokmin);
	int (*proximity_show_autokmax)(void);
	int (*proximity_store_autokmax)(int autokmax);

	/*Get the number of interrupts and events*/
	int (*proximity_show_int_count)(void);
	int (*proximity_show_event_count)(void);

	/*For unit/stress test*/
	int (*proximity_show_error_mesg)(char *error_mesg);

	/*For transition period from 3/5 to 2/4*/
	int (*proximity_show_selection)(void);
	int (*proximity_store_selection)(int selection);
	
	/*For power key turn on screen and enable touch*/
	int (*proximity_show_touch_enable)(void);
	int (*proximity_store_touch_enable)(bool enable);
	
	/*For load calibration data*/
	int (*proximity_store_load_calibration_data)(void);
	
	/*For enable anti-oil workaround*/
	int (*proximity_show_anti_oil_enable)(void);
	int (*proximity_store_anti_oil_enable)(bool enable);
}psensor_ATTR_Extension;

/**
 * psensor_ATTR - attributes for ALSPSsensor.
 */
typedef struct{
	psensor_info_type 			*info_type;	
	psensor_ATTR_Calibration 	*ATTR_Calibration;
	psensor_ATTR_BMMI 		*ATTR_BMMI;
	psensor_ATTR_Hardware 	*ATTR_Hardware;
	psensor_ATTR_HAL 		*ATTR_HAL;
	psensor_ATTR_Extension 	*ATTR_Extension;
}psensor_ATTR;

/**
 * psensor_ATTR_register - assign a psensor only file node and create the attributes.
 *
 * The attributes will be created at /sys/class/sensors/psensor.
  */
extern int psensor_ATTR_register(psensor_ATTR *mATTR);

/**
 * psensor_ATTR_unregister - remove the psensor attributes.
 */
extern int psensor_ATTR_unregister(void);

/**
 * psensor_ATTR_create - create psensor customize attributes.
 * @mpsensor_attr : the pointer of device_attribute, which you want to create attribute.
 */
#include <linux/device.h>
extern int psensor_ATTR_create(struct device_attribute *mpsensor_attr); 

/**
 * lsensor_info_type - define the lsensor information.
 * @vendor : ASUS.
 * @version : driver version.
 * @module_number : hardware chip serial number.
 */
#define NAME_SIZE	(10)
typedef struct{
	char vendor[NAME_SIZE];
	char module_number[NAME_SIZE];
}lsensor_info_type;

/**
 * @LIGHT_GAIN_ACCURACY_CALVALUE : the accuracy of light sensor gain value.
 */
#define LIGHT_GAIN_ACCURACY_CALVALUE	(10000)

/**
 * lsensor_ATTR_Calibration - attributes for lsensor calibration.
 */
typedef struct{	
	int (*light_show_calibration)(void);
	int (*light_store_calibration)(int calvalue);
	int (*light_show_gain)(void);
	int (*light_show_adc)(void); 
}lsensor_ATTR_Calibration;

/**
 * lsensor_ATTR_BMMI - attributes for lsensor BMMI.
 */
typedef struct{ 
	bool (*light_show_atd_test)(void);
}lsensor_ATTR_BMMI;

/**
 * lsensor_ATTR_Hardware - attributes for lsensor hardware read/write.
 */
typedef struct{
	uint8_t show_reg_addr;
	int (*light_show_reg)(uint8_t addr);
	int (*light_store_reg)(uint8_t addr, int value);	
}lsensor_ATTR_Hardware;

/**
 * lsensor_ATTR_HAL - attributes for lsensor HAL function.
 */
typedef struct{
	bool (*light_show_switch_onoff)(void);
	int (*light_store_switch_onoff)(bool bOn);
	int (*light_show_lux)(void);		
}lsensor_ATTR_HAL;

/**
 * lsensor_ATTR_Extension - attributes for lsensor extensive functions.
 */
typedef struct{	
	bool (*light_show_allreg)(void);

	/*modify the sensitivity of light sensor*/
	int (*light_show_sensitivity)(void);
	int (*light_store_sensitivity)(int sensitivity);

	/*change the frequence of log for the light sensor*/
	int (*light_show_log_threshold)(void);
	int (*light_store_log_threshold)(int log_threshold);

	/*Get the number of interrupts and events*/
	int (*light_show_int_count)(void);
	int (*light_show_event_count)(void);

	/*For unit/stress test*/
	int (*light_show_error_mesg)(char *error_mesg);

	/*For transition period from 3/5 to 2/4*/
	int (*light_show_selection)(void);
	int (*light_store_selection)(int selection);
}lsensor_ATTR_Extension;

/**
 * lsensor_ATTR - attributes for lsensor.
 */
typedef struct{
	lsensor_info_type 			*info_type;	
	lsensor_ATTR_Calibration 	*ATTR_Calibration;
	lsensor_ATTR_BMMI 		*ATTR_BMMI;
	lsensor_ATTR_Hardware   	*ATTR_Hardware;
	lsensor_ATTR_HAL 		         *ATTR_HAL;
	lsensor_ATTR_Extension 	         *ATTR_Extension;
}lsensor_ATTR;

/**
 * lsensor_ATTR_register - assign a psensor only file node and create the attributes.
 *
 * The attributes will be created at /sys/class/sensors/lsensor.
  */
extern int lsensor_ATTR_register(lsensor_ATTR *mATTR);

/**
 *lsensor_ATTR_unregister - remove the lsensor attributes.
 */
extern int lsensor_ATTR_unregister(void);

/**
 * lsensor_ATTR_create - create lsensor customize attributes.
 * @mlsensor_attr : the pointer of device_attribute, which you want to create attribute.
 */
extern int lsensor_ATTR_create(struct device_attribute *mlsensor_attr);

/**
 * FRGBsensor_info_type - define the FRGBsensor information.
 * @vendor : ASUS.
 * @version : driver version.
 * @module_number : hardware chip serial number.
 */
#define NAME_SIZE	(10)
typedef struct{
	char vendor[NAME_SIZE];
	char module_number[NAME_SIZE];
}FRGBsensor_info_type;

/**
 * FRGBsensor_ATTR_Calibration - attributes for FRGBsensor calibration.
 */
typedef struct{
	int (*FRGB_show_red)(void); 
	int (*FRGB_show_green)(void); 
	int (*FRGB_show_blue)(void); 
	int (*FRGB_show_ir)(void); 
}FRGBsensor_ATTR_Calibration;

/**
 * FRGBsensor_ATTR_BMMI - attributes for FRGBsensor BMMI.
 */
typedef struct{ 
	bool (*FRGB_show_atd_test)(void);
}FRGBsensor_ATTR_BMMI;

/**
 * FRGBsensor_ATTR_Hardware - attributes for FRGBsensor hardware read/write.
 */
typedef struct{
	uint8_t show_reg_addr;
	int (*FRGB_show_reg)(uint8_t addr);
	int (*FRGB_store_reg)(uint8_t addr, int value);	
}FRGBsensor_ATTR_Hardware;

/**
 * FRGBsensor_ATTR_HAL - attributes for FRGBsensor HAL function.
 */
typedef struct{
	bool (*FRGB_show_switch_onoff)(void);
	int (*FRGB_store_switch_onoff)(bool bOn);	
}FRGBsensor_ATTR_HAL;

/**
 * FRGBsensor_ATTR_Extension - attributes for FRGBsensor extensive functions.
 */
typedef struct{	
	bool (*FRGB_show_allreg)(void);

	/*change the frequence of log for the light sensor*/
	int (*FRGB_show_log_threshold)(void);
	int (*FRGB_store_log_threshold)(int log_threshold);

	/*For unit/stress test*/
	int (*FRGB_show_error_mesg)(char *error_mesg);
}FRGBsensor_ATTR_Extension;

/**
 * FRGBsensor_ATTR - attributes for lsensor.
 */
typedef struct{
	FRGBsensor_info_type 			*info_type;	
	FRGBsensor_ATTR_Calibration 	*ATTR_Calibration;
	FRGBsensor_ATTR_BMMI 		*ATTR_BMMI;
	FRGBsensor_ATTR_Hardware   	*ATTR_Hardware;
	FRGBsensor_ATTR_HAL 		         *ATTR_HAL;
	FRGBsensor_ATTR_Extension 	         *ATTR_Extension;
}FRGBsensor_ATTR;

/**
 * FRGBsensor_ATTR_register - assign a psensor only file node and create the attributes.
 *
 * The attributes will be created at /sys/class/sensors/FRGBsensor.
  */
extern int FRGBsensor_ATTR_register(FRGBsensor_ATTR *mATTR);

/**
 *FRGBsensor_ATTR_unregister - remove the FRGBsensor attributes.
 */
extern int FRGBsensor_ATTR_unregister(void);

/**
 * FRGBsensor_ATTR_create - create FRGBsensor customize attributes.
 * @mFRGBsensor_attr : the pointer of device_attribute, which you want to create attribute.
 */
extern int FRGBsensor_ATTR_create(struct device_attribute *mFRGBsensor_attr);

/**
 * Define the proximity report event values.
 * @PSENSOR_REPORT_PS_CLOSE : report close event.
 * @PSENSOR_REPORT_PS_AWAY : report away event.
 */
#define PSENSOR_REPORT_PS_POCKET            (0)
#define PSENSOR_REPORT_PS_CLOSE 			(2)
#define PSENSOR_REPORT_PS_AWAY     			(10) 

/**
 * psensor_report_register - before report psensor event
 * you need to register first. This will create input device for psensor.
 */
extern int psensor_report_register(void);

/**
 * psensor_report_unregister - Remove the input device for psensor.
 */
extern void psensor_report_unregister(void);

/**
 * psensor_report_abs - report the proximity abs.
 * @ abs=IRSENSOR_REPORT_PS_CLOSE : report close event.
 * @ abs=IRSENSOR_REPORT_PS_AWAY : report away event.
 */
extern void psensor_report_abs(int abs);

/**
 * lsensor_report_register - before report lsensor event
 * you need to register first. This will create input device for lsensor.
 */
extern int lsensor_report_register(void);

/**
 * lsensor_report_unregister - Remove the input device for lsensor.
 */
extern void lsensor_report_unregister(void);

/**
 * lsensor_report_lux - report the light sensor lux.
 */
extern void lsensor_report_lux(int lux);

/**
 * FRGBsensor_report_register - before report FRGB sensor event
 * you need to register first. This will create input device for FRGB sensor.
 */
extern int FRGBsensor_report_register(void);

/**
 * FRGBsensor_report_unregister - Remove the input device for FRGB sensor.
 */
extern void FRGBsensor_report_unregister(void);

/**
 * FRGBsensor_report_raw - report the FRGB sensor raw data.
 */
extern void FRGBsensor_report_raw(int *data, int size); 

/**
 * Define the hall sensor report event values.
 * @HALLSENSOR_REPORT_LID_OPEN : report open event.
 * @HALLSENSOR_REPORT_LID_CLOSE : report close event.
 */
#define HALLSENSOR_REPORT_LID_OPEN 			(0)
#define HALLSENSOR_REPORT_LID_CLOSE    		(1) 

/**
 * HALLsensor_report_register - before report hall sensor event
 * you need to register first. This will create input device for Hall sensor.
 */
extern int HALLsensor_report_register(void);

/**
 * HALLsensor_report_unregister - Remove the input device for Hall sensor.
 */
extern void HALLsensor_report_unregister(void);

/**
 * hallsensor_report_lid - report the hall sensor lid.
 */
extern void hallsensor_report_lid(int lid);

/**
 * Define the path of Light Sensor Calibration file.
 * @LSENSOR_200LUX_CALIBRATION_FILE : 200 Lux Calibration file.
 * @LSENSOR_1000LUX_CALIBRATION_FILE : 1000 Lux Calibration file.
 */
#define LSENSOR_200LUX_CALIBRATION_FILE	    "/vendor/factory/lsensor_200lux.nv"
#define LSENSOR_1000LUX_CALIBRATION_FILE	"/vendor/factory/lsensor_1000lux.nv"
#define LSENSOR_CALIBRATION_FILE			"/vendor/factory/sensors/als_cal_50ms" //"/vendor/factory/lsensor.nv"

/*For transition period from 100ms to 50ms +++*/
#define LSENSOR_50MS_CALIBRATION_FILE		"/vendor/factory/sensors/als_cal_50ms" //"/vendor/factory/lsensor_50ms.nv"
#define LSENSOR_100MS_CALIBRATION_FILE		"/vendor/factory/sensors/als_cal_100ms" //"/vendor/factory/lsensor_100ms.nv"
/*For transition period from 100ms to 50ms ---*/

/**
 * Define the path of Proximity Sensor Calibration file.
 * @PSENSOR_HI_CALIBRATION_FILE : high calibration file.
 * @PSENSOR_LOW_CALIBRATION_FILE : low calibration file.
 * @PSENSOR_INF_CALIBRATION_FILE : cross talk calibration file.
 */
#define PSENSOR_HI_CALIBRATION_FILE  	"/vendor/factory/sensors/ps_cal_2cm" //"/vendor/factory/psensor_hi.nv"
#define PSENSOR_LOW_CALIBRATION_FILE  	"/vendor/factory/sensors/ps_cal_4cm" //"/vendor/factory/psensor_low.nv"
#define PSENSOR_INF_CALIBRATION_FILE  	"/vendor/factory/sensors/ps_cal_inf" //"/vendor/factory/psensor_inf.nv"

/*For transition period from 3/5 to 2/4 +++*/
#define PSENSOR_1CM_CALIBRATION_FILE    "/vendor/factory/sensors/ps_cal_1cm" //"/vendor/factory/psensor_1cm.nv"
#define PSENSOR_2CM_CALIBRATION_FILE  	"/vendor/factory/sensors/ps_cal_2cm" //"/vendor/factory/psensor_2cm.nv"
#define PSENSOR_4CM_CALIBRATION_FILE  	"/vendor/factory/sensors/ps_cal_4cm" //"/vendor/factory/psensor_4cm.nv"
#define PSENSOR_3CM_CALIBRATION_FILE  	"/vendor/factory/sensors/ps_cal_3cm" //"/vendor/factory/psensor_3cm.nv"
#define PSENSOR_5CM_CALIBRATION_FILE  	"/vendor/factory/sensors/ps_cal_5cm" //"/vendor/factory/psensor_5cm.nv"
/*For transition period from 3/5 to 2/4 ---*/

/**
 * Define the path of Front RGB Calibration file.
 * @FRGB_LIGHT1_CALIBRATION_FILE : light1 calibration file.
 * @FRGB_LIGHT2_CALIBRATION_FILE : light2 calibration file.
 * @FRGB_LIGHT3_CALIBRATION_FILE : light3 calibration file.
 */
#define FRGB_LIGHT1_CALIBRATION_FILE  	"/vendor/factory/sensors/frgb_light1.nv"
#define FRGB_LIGHT2_CALIBRATION_FILE  	"/vendor/factory/sensors/frgb_light2.nv"
#define FRGB_LIGHT3_CALIBRATION_FILE  	"/vendor/factory/sensors/frgb_light3.nv"

/**
 * psensor_factory_read_high
 * psensor_factory_read_low - kernel space read high/low calibration data.
 * 
 * Return value is NOT negative (>=0) if it is success reading from file(.nv)
 */
extern int 	psensor_factory_read_high(void);
extern int 	psensor_factory_read_low(void);

/**
 * psensor_factory_write_high
 * psensor_factory_write_low - kernel space write high/low calibration data.
 *
 * Return value is TRUE if it is success writing value to file(.nv)
 */
extern bool psensor_factory_write_high(int calvalue);
extern bool psensor_factory_write_low(int calvalue);

/**
 * psensor_factory_read_inf
 * psensor_factory_write_inf - kernel space read/write inf calibration data.
 *
 * Return value is TRUE if it is success writing value to file(.nv)
 */
extern int psensor_factory_read_inf(void);
extern bool psensor_factory_write_inf(int calvalue);

/*For transition period from 3/5 to 2/4 +++*/
extern int 	psensor_factory_read_2cm(void);
extern bool psensor_factory_write_2cm(int calvalue);
extern int 	psensor_factory_read_4cm(void);
extern bool psensor_factory_write_4cm(int calvalue);

extern int 	psensor_factory_read_3cm(void);
extern bool psensor_factory_write_3cm(int calvalue);
extern int 	psensor_factory_read_5cm(void);
extern bool psensor_factory_write_5cm(int calvalue);

extern int psensor_factory_read_1cm(void);
extern bool psensor_factory_write_1cm(int calvalue);
/*For transition period from 3/5 to 2/4 ---*/

/**
 * lsensor_factory_read_200lux
 * lsensor_factory_write_200lux - kernel space read/write 200lux calibration data. 
 *
 * Return value is NOT negative (>=0) if it is success reading from file(.nv)
 * Return value is TRUE if it is success writing value to file(.nv)
 */
extern int 	lsensor_factory_read_200lux(void);
extern bool lsensor_factory_write_200lux(int calvalue);

/**
 * lsensor_factory_read_1000lux
 * lsensor_factory_write_1000lux - kernel space read/write 1000lux calibration data.
 *
 * Return value is NOT negative (>=0) if it is success reading from file(.nv)
 * Return value is TRUE if it is success writing value to file(.nv)
 */
extern int 	lsensor_factory_read_1000lux(void);
extern bool lsensor_factory_write_1000lux(int calvalue);

/**
 * lsensor_factory_read
 * lsensor_factory_write - kernel space read/write calibration data. 
 *
 * Return value is NOT negative (>=0) if it is success reading from file(.nv)
 * Return value is TRUE if it is success writing value to file(.nv)
 */
extern int 	lsensor_factory_read(void);
extern bool lsensor_factory_write(int calvalue);

/*For transition period from 100ms to 50ms +++*/
extern int 	lsensor_factory_read_50ms(void);
extern bool lsensor_factory_write_50ms(int calvalue);

extern int 	lsensor_factory_read_100ms(void);
extern bool lsensor_factory_write_100ms(int calvalue);
/*For transition period from 100ms to 50ms ---*/

/**
 * FRGBsensor_factory_read_light1
 * FRGBsensor_factory_write_light1 - kernel space read/write light1 calibration data. 
 *
 * Return value is NOT negative (>=0) if it is success reading from file(.nv)
 * Return value is TRUE if it is success writing value to file(.nv)
 */
extern int 	FRGBsensor_factory_read_light1(void);
extern bool FRGBsensor_factory_write_light1(int calvalue);

/**
 * FRGBsensor_factory_read_light2
 * FRGBsensor_factory_write_light2 - kernel space read/write light2 calibration data. 
 *
 * Return value is NOT negative (>=0) if it is success reading from file(.nv)
 * Return value is TRUE if it is success writing value to file(.nv)
 */
extern int 	FRGBsensor_factory_read_light2(void);
extern bool FRGBsensor_factory_write_light2(int calvalue);

/**
 * FRGBsensor_factory_read_light3
 * FRGBsensor_factory_write_light3 - kernel space read/write light3 calibration data. 
 *
 * Return value is NOT negative (>=0) if it is success reading from file(.nv)
 * Return value is TRUE if it is success writing value to file(.nv)
 */
extern int 	FRGBsensor_factory_read_light3(void);
extern bool FRGBsensor_factory_write_light3(int calvalue);

/**
 * psensor_GPIO
 */
 typedef struct psensor_GPIO {
	void (*psensor_isr)(void);
}psensor_GPIO;

/**
 * lsensor_GPIO
 */
 typedef struct lsensor_GPIO {
	void (*lsensor_isr)(void);
}lsensor_GPIO;

/**
 * ALSPSsensor_GPIO
 */
 typedef struct ALSPSsensor_GPIO {
	void (*ALSPSsensor_isr)(void);
}ALSPSsensor_GPIO;

/**
 * HALLsensor_GPIO
 */
 typedef struct HALLsensor_GPIO {
	void (*HALLsensor_isr)(void);
}HALLsensor_GPIO;

/**
 * ALSPSsensor_gpio_register - register the GPIO setting and set the IRQ handler.
 * Return the IRQ.
 */
#include <linux/i2c.h>
extern int ALSPSsensor_gpio_register(struct i2c_client *client, ALSPSsensor_GPIO *gpio_ist);

/**
 * ALSPSsensor_gpio_unregister - unregister the GPIO setting.
 */
extern int ALSPSsensor_gpio_unregister(int irq);

/**
 * ALSPSsensor_enabe_gpio_register - register the enable ldo GPIO setting and set the IRQ handler.
 */
extern int ALSPSsensor_enabe_gpio_register(struct i2c_client *client);

/**
 * ALSPSsensor_enabe_gpio_unregister - unregister the enable ldo GPIO setting.
 */
extern int ALSPSsensor_enabe_gpio_unregister(void);


/**
 * psensor_gpio_register - register the GPIO setting and set the IRQ handler.
 * Return the IRQ.
 */
#include <linux/i2c.h>
extern int psensor_gpio_register(struct i2c_client *client, psensor_GPIO *gpio_ist);

/**
 * psensor_gpio_unregister - unregister the GPIO setting.
 */
extern int psensor_gpio_unregister(int irq);

/**
 * lsensor_gpio_register - register the GPIO setting and set the IRQ handler.
 * Return the IRQ.
 */
extern int lsensor_gpio_register(struct i2c_client *client, lsensor_GPIO *gpio_ist);

/**
 * lsensor_gpio_unregister - unregister the GPIO setting.
 */
extern int lsensor_gpio_unregister(int irq);

/**
 * HALLsensor_gpio_register - register the GPIO setting and set the IRQ handler.
 * Return the IRQ.
 */
#include <linux/platform_device.h>
extern int HALLsensor_gpio_register(struct platform_device *pdev, HALLsensor_GPIO *gpio_ist);

/**
 * HALLsensor_gpio_unregister - unregister the GPIO setting.
 */
extern int HALLsensor_gpio_unregister(int irq);

/**
 * ALSPSsensor_gpio_setI2cClient - set the i2c_client for of_get_named_gpio.
 */
 #include <linux/i2c.h>
 extern int ALSPSsensor_gpio_setI2cClient(struct i2c_client *client);

/**
 * HALLsensor_gpio_value - return the gpio value.
 */
extern int HALLsensor_gpio_value(void);

/**
 * i2c_read_reg_u8 - 
 * i2c_write_reg_u8 - read/write i2c for 1 Byte (8bits). These are functions for i2c read/write.
 */
extern uint8_t i2c_read_reg_u8(struct i2c_client* client, u8 reg);
extern int 		i2c_write_reg_u8(struct i2c_client* client, u8 reg, uint8_t data);

/**
 * i2c_read_reg_u16 - 
 * i2c_write_reg_u16 - read/write i2c for 2 Byte (16bits). These are functions for i2c read/write.
 */
extern int i2c_read_reg_u16(struct i2c_client* client, u8 reg, uint8_t* data);
extern int i2c_write_reg_u16(struct i2c_client* client, u8 reg, uint8_t* data);

/**
 * ALSPS_I2C - We define functions for i2c driver.
 * @ALSPS_probe : It will be run when "i2c_add_driver" has been called.
 * @ALSPS_remove : Remove all Memory.
 * @ALSPS_shutdown : Turn off all sensors.
 * @ALSPS_suspend : Do what suspend should do.
 * @ALSPS_resume : Do what resume should do.
 */
typedef struct ALSPS_I2C {
	void (*ALSPS_probe)(struct i2c_client *client);
	void (*ALSPS_remove)(void);
	void (*ALSPS_shutdown)(void);
	void (*ALSPS_suspend)(void);
	void (*ALSPS_resume)(void);	
}ALSPS_I2C;

/**
 * psensor_I2C - We define functions for i2c driver.
 * @psensor_probe : It will be run when "i2c_add_driver" has been called.
 * @psensor_remove : Remove all Memory.
 * @psensor_shutdown : Turn off all sensors.
 * @psensor_suspend : Do what suspend should do.
 * @psensor_resume : Do what resume should do.
 */
typedef struct psensor_I2C {
	void (*psensor_probe)(struct i2c_client *client);
	void (*psensor_remove)(void);
	void (*psensor_shutdown)(void);
	void (*psensor_suspend)(void);
	void (*psensor_resume)(void);	
}psensor_I2C;

/**
 * lsensor_I2C - We define functions for i2c driver.
 * @lsensor_probe : It will be run when "i2c_add_driver" has been called.
 * @lsensor_remove : Remove all Memory.
 * @lsensor_shutdown : Turn off all sensors.
 * @lsensor_suspend : Do what suspend should do.
 * @lsensor_resume : Do what resume should do.
 */
typedef struct lsensor_I2C {
	void (*lsensor_probe)(struct i2c_client *client);
	void (*lsensor_remove)(void);
	void (*lsensor_shutdown)(void);
	void (*lsensor_suspend)(void);
	void (*lsensor_resume)(void);	
}lsensor_I2C;

/**
 * ALSPS_FRGB_I2C - We define functions for i2c driver.
 * @ALSPS_FRGB_probe : It will be run when "i2c_add_driver" has been called.
 * @ALSPS_FRGB_remove : Remove all Memory.
 * @ALSPS_FRGB_shutdown : Turn off all sensors.
 * @ALSPS_FRGB_suspend : Do what suspend should do.
 * @ALSPS_FRGB_resume : Do what resume should do.
 */
typedef struct ALSPS_FRGB_I2C {
	void (*ALSPS_FRGB_probe)(struct i2c_client *client);
	void (*ALSPS_FRGB_remove)(void);
	void (*ALSPS_FRGB_shutdown)(void);
	void (*ALSPS_FRGB_suspend)(void);
	void (*ALSPS_FRGB_resume)(void);	
}ALSPS_FRGB_I2C;

/**
 * ALSPS_i2c_register - struct ALSPSsensor_I2C wrapped by i2c driver.
 */
extern int ALSPS_i2c_register(ALSPS_I2C *ir_i2c);

/**
 * ALSPS_i2c_unregister - i2c_del_driver.
 */
extern int ALSPS_i2c_unregister(void);

/**
 * psensor_i2c_register - struct psensor_I2C wrapped by i2c driver.
 */
extern int psensor_i2c_register(psensor_I2C *ir_i2c);

/**
 * psensor_i2c_unregister - i2c_del_driver.
 */
extern int psensor_i2c_unregister(void);

/**
 * lsensor_i2c_register - struct lsensor_I2C wrapped by i2c driver.
 */
extern int lsensor_i2c_register(lsensor_I2C *ir_i2c);

/**
 * lsensor_i2c_unregister - i2c_del_driver.
 */
extern int lsensor_i2c_unregister(void);

/**
 * ALSPS_FRGB_i2c_register - struct ALSPS_FRGB_I2C wrapped by i2c driver.
 */
extern int ALSPS_FRGB_i2c_register(ALSPS_FRGB_I2C *ir_i2c);

/**
 * ALSPS_FRGB_i2c_unregister - i2c_del_driver.
 */
extern int ALSPS_FRGB_i2c_unregister(void);

/**
 * HALLsensor_Platform - We define functions for platform driver.
 * @HALLsensor_probe : It will be run when "platform_driver_register" has been called.
 * @HALLsensor_suspend : Do what suspend should do.
 * @HALLsensor_resume : Do what resume should do.
 */
typedef struct HALLsensor_Platform {
	void (*HALLsensor_probe)(struct platform_device *pdev);
	void (*HALLsensor_suspend)(void);
	void (*HALLsensor_resume)(void);	
}HALLsensor_Platform;

/**
 * IRsensor_i2c_register - struct IRsensor_I2C wrapped by i2c driver.
 */
extern int HALLsensor_platform_register(HALLsensor_Platform *hall_platform);

/**
 * IRsensor_i2c_unregister - i2c_del_driver.
 */
extern int HALLsensor_platform_unregister(void);


/**
 * Define the return interrupt bits, which support proximity close/away and light sensor simultaneously.
 * @ALSPS_INT_PS_CLOSE : [1:0] (0,1)
 * @ALSPS_INT_PS_AWAY : [1:0] (1,0)
 * @ALSPS_INT_ALS : [2] (1)
 */
#define ALSPS_INT_PS_CLOSE 				(1)
#define ALSPS_INT_PS_AWAY     			(2) 
#define ALSPS_INT_PS_MASK				(3<< 0)
#define ALSPS_INT_ALS           				(4)
#define ALSPS_INT_ALS_MASK				(1<< 2)

/**
 * psensor_hw - the i2c control functions for proximity sensor.
 * @proximity_low_threshold_default : 
 * @proximity_hi_threshold_default : depends on each hardware situation, which impact on the CSC SMMI.
 * @proximity_hw_turn_onoff : Turn on the proximity sensor.
 * @proximity_hw_get_adc : get the count of proximity sensor.
 * @proximity_hw_set_hi_threshold :
 * @proximity_hw_set_lo_threshold : set proximity threshold which will trigger the interrupt.
 */
typedef struct psensor_hw {
	/*For psensor only +++*/
	char vendor[NAME_SIZE];
	char module_number[NAME_SIZE];
	/*For psensor only ---*/
	
	int proximity_low_threshold_default;	
	int proximity_hi_threshold_default;
	int proximity_crosstalk_default;
	int proximity_autok_min;
	int proximity_autok_max;

	/*For psensor only +++*/
	int (*proximity_hw_check_ID)(void);	
	int (*proximity_hw_init)(struct i2c_client* client);
	int (*proximity_hw_get_interrupt)(void);	
	int (*proximity_hw_show_allreg)(void);	
	int (*proximity_hw_set_register)(uint8_t reg, int value);
	int (*proximity_hw_get_register)(uint8_t reg);	
	/*For psensor only ---*/
	
	int (*proximity_hw_turn_onoff)(bool bOn);
	int (*proximity_hw_interrupt_onoff)(bool bOn);
	int (*proximity_hw_get_adc)(void);
	int (*proximity_hw_set_hi_threshold)(int hi_threshold);
	int (*proximity_hw_set_lo_threshold)(int low_threshold);
	int (*proximity_hw_set_autoK)(int autoK);
}psensor_hw;

/**
 * lsensor_hw - the i2c control functions for light sensor.
 * @light_max_threshold : the maximum count of light sensor.
 * @light_hw_turn_onoff : Turn on the light sensor.
 * @light_hw_get_adc : get the count of light sensor.
 * @light_hw_set_hi_threshold : 
 * @light_hw_set_lo_threshold : set light sensor threshold which will trigger the interrupt.
 */
typedef struct lsensor_hw {
	/*For lsensor only +++*/
	char vendor[NAME_SIZE];
	char module_number[NAME_SIZE];
	/*For lsensor only ---*/
	
	int light_max_threshold;
	int light_calibration_default;

	/*For lsensor only +++*/
	int (*light_hw_check_ID)(void);	
	int (*light_hw_init)(struct i2c_client* client);
	int (*light_hw_get_interrupt)(void);	
	int (*light_hw_show_allreg)(void);	
	int (*light_hw_set_register)(uint8_t reg, int value);
	int (*light_hw_get_register)(uint8_t reg);	
	/*For lsensor only ---*/
	
	int (*light_hw_turn_onoff)(bool bOn);
	int (*light_hw_interrupt_onoff)(bool bOn);
	int (*light_hw_get_adc)(void);
	int (*light_hw_set_hi_threshold)(int hi_threshold);
	int (*light_hw_set_lo_threshold)(int low_threshold);

	int (*light_hw_set_integration)(uint8_t integration);
	uint8_t (*light_hw_get_integration)(void);
}lsensor_hw;

/**
 * FRGB_hw - the i2c control functions for front RGB sensor.
 * @frgb_red_ratio_default : the default ratio of red.
 * @frgb_green_ratio_default : the default ratio of green.
 * @frgb_blue_ratio_default : the default ratio of blue.
 * @frgb_hw_turn_onoff : Turn on the FRGB sensor.
 * @frgb_hw_get_red : get the count of RED CHANNEL.
 * @frgb_hw_get_green : get the count of GREEN CHANNEL.
 * @frgb_hw_get_blue : get the count of BLUE CHANNEL. 
 * @frgb_hw_get_ir : get the count of IR CHANNEL.  
 */
typedef struct FRGB_hw {
	/*For FRGB only +++*/
	char vendor[NAME_SIZE];
	char module_number[NAME_SIZE];
	/*For FRGB only ---*/

	/*For FRGB only +++*/
	int (*frgb_hw_check_ID)(void);	
	int (*frgb_hw_init)(struct i2c_client* client);
	int (*frgb_hw_get_interrupt)(void);	
	int (*frgb_hw_show_allreg)(void);	
	int (*frgb_hw_set_register)(uint8_t reg, int value);
	int (*frgb_hw_get_register)(uint8_t reg);	
	/*For FRGB only ---*/
	
	int (*frgb_hw_turn_onoff)(bool bOn);
	int (*frgb_hw_get_red)(void);
	int (*frgb_hw_get_green)(void);
	int (*frgb_hw_get_blue)(void);
	int (*frgb_hw_get_ir)(void);
}FRGB_hw;

/**
 * ALSPSsensor_hw - the i2c control functions for ALSPS sensor including psensor and lsensor.
 */
 #include <linux/i2c.h>
typedef struct ALSPS_hw {	
	char vendor[NAME_SIZE];
	char module_number[NAME_SIZE];

	int (*ALSPS_hw_check_ID)(void);	
	int (*ALSPS_hw_init)(struct i2c_client* client);
	int (*ALSPS_hw_get_interrupt)(void);	
	int (*ALSPS_hw_show_allreg)(void);	
	int (*ALSPS_hw_set_register)(uint8_t reg, int value);
	int (*ALSPS_hw_get_register)(uint8_t reg);	

	psensor_hw	*mpsensor_hw;
	lsensor_hw	*mlsensor_hw;
}ALSPS_hw;

 /**
 * ALSPS_FRGB_hw - the i2c control functions for ALSPS FRGB sensor including psensor and lsensor.
 */
 #include <linux/i2c.h>
typedef struct ALSPS_FRGB_hw {	
	char vendor[NAME_SIZE];
	char module_number[NAME_SIZE];

	int (*ALSPS_FRGB_hw_check_ID)(void);	
	int (*ALSPS_FRGB_hw_init)(struct i2c_client* client);
	int (*ALSPS_FRGB_hw_get_interrupt)(void);	
	int (*ALSPS_FRGB_hw_show_allreg)(void);	
	int (*ALSPS_FRGB_hw_set_register)(uint8_t reg, int value);
	int (*ALSPS_FRGB_hw_get_register)(uint8_t reg);	

	psensor_hw	*mpsensor_hw;
	lsensor_hw	*mlsensor_hw;
	FRGB_hw	*mFRGB_hw;
}ALSPS_FRGB_hw;

/**
 * ALSPS_hw_getHardware - Before this function, you SHOULD execute ALSPSsensor_i2c_register first.
 */
extern ALSPS_hw* ALSPS_hw_getHardware(void);

/**
 * psensor_hw_getHardware - Before this function, you SHOULD execute psensor_i2c_register first.
 */
extern psensor_hw* psensor_hw_getHardware(void);

/**
 * lsensor_hw_getHardware - Before this function, you SHOULD execute lsensor_i2c_register first.
 */
extern lsensor_hw* lsensor_hw_getHardware(void);

/**
 * ALSPS_FRGB_hw_getHardware - Before this function, you SHOULD execute ALSPS_FRGB_i2c_register first.
 */
extern ALSPS_FRGB_hw* ALSPS_FRGB_hw_getHardware(void);

/**
 * Proximity auto calibration
 */
#define PROXIMITY_AUTOK_COUNT				(6)
#define PROXIMITY_AUTOK_POLLING			(500)
#define PROXIMITY_AUTOK_DELAY				(10)

#endif
