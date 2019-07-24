/*
 * Copyright (c) 2014, 2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* register and associated bit definition */

#ifndef __ICM206XX_H__
#define __ICM206XX_H__

#define REG_SAMPLE_RATE_DIV	0x19
#define REG_CONFIG		0x1A

#define REG_GYRO_CONFIG		0x1B
#define BITS_SELF_TEST_EN	0xE0
#define GYRO_CONFIG_FSR_SHIFT_ICM2069X	2
#define GYRO_CONFIG_FSR_SHIFT_ICM2060X	3

#define REG_ACCEL_CONFIG	0x1C
#define ACCL_CONFIG_FSR_SHIFT	3

#define REG_ACCEL_WOM_X_THR	0x20
#define REG_ACCEL_WOM_Y_THR	0x21
#define REG_ACCEL_WOM_Z_THR	0x22

#define REG_FIFO_EN		    0x23
#define FIFO_DISABLE_ALL	0x00
#define BIT_ACCEL_FIFO		0x08
#define BIT_GYRO_FIFO		0x70
#define BIT_TEMP_FIFO		0x80

#define REG_INT_PIN_CFG		0x37
#define BIT_INT_ACTIVE_LOW	0x80
#define BIT_INT_OPEN_DRAIN	0x40
#define BIT_INT_LATCH_EN	0x20
#define BIT_INT_RD_CLR		0x10
#define BIT_I2C_BYPASS_EN	0x02
#define BIT_INT_CFG_DEFAULT	(BIT_INT_LATCH_EN | BIT_INT_RD_CLR)

#define REG_INT_ENABLE		0x38
#define BIT_DATA_RDY_EN		0x01
#define BIT_FIFO_OVERFLOW_EN	0x10
#define BIT_WOM_Z_INT_EN	0x20
#define BIT_WOM_Y_INT_EN	0x40
#define BIT_WOM_X_INT_EN	0x80

#define REG_FIFO_WM_INT_STATUS	0x39
#define BIT_FIFO_WM_INT	    0x40

#define REG_INT_STATUS		0x3A
#define BIT_DATA_RDY_INT	0x01
#define BIT_FIFO_OVERFLOW	0x10
#define BIT_WOM_Z_INT	    0x20
#define BIT_WOM_Y_INT	    0x40
#define BIT_WOM_X_INT	    0x80

#define REG_RAW_ACCEL		0x3B
#define REG_TEMPERATURE		0x41
#define REG_RAW_GYRO		0x43
#define REG_EXT_SENS_DATA_00	0x49

#define REG_DETECT_CTRL		0x69
#define MOT_DET_DELAY_SHIFT	4

#define REG_USER_CTRL		0x6A
#define BIT_FIFO_EN		    0x40
#define BIT_FIFO_RESET		0x04

#define REG_PWR_MGMT_1		0x6B
#define BIT_H_RESET		0x80
#define BIT_SLEEP		0x40
#define BIT_CYCLE		0x20
#define BIT_CLK_MASK		0x07
#define BIT_RESET_ALL		0xCF
#define BIT_WAKEUP_AFTER_RESET	0x00

#define REG_PWR_MGMT_2		0x6C
#define BIT_PWR_ACCEL_STBY_MASK	0x38
#define BIT_PWR_GYRO_STBY_MASK	0x07
#define BITS_PWR_ALL_AXIS_STBY	(BIT_PWR_ACCEL_STBY_MASK |\
				BIT_PWR_GYRO_STBY_MASK)

#define REG_FIFO_COUNT_H	0x72
#define REG_FIFO_R_W		0x74
#define REG_WHOAMI		0x75

#define SAMPLE_DIV_MAX		0xFF
#define ODR_DLPF_DIS		8000
#define ODR_DLPF_ENA		1000

/* Min delay = MSEC_PER_SEC/ODR_DLPF_ENA */
/* Max delay = MSEC_PER_SEC/(ODR_DLPF_ENA/SAMPLE_DIV_MAX+1) */
#define DELAY_MS_MIN_DLPF	1
#define DELAY_MS_MAX_DLPF	256

/* Min delay = MSEC_PER_SEC/ODR_DLPF_DIS and round up to 1*/
/* Max delay = MSEC_PER_SEC/(ODR_DLPF_DIS/SAMPLE_DIV_MAX+1) */
#define DELAY_MS_MIN_NODLPF	1
#define DELAY_MS_MAX_NODLPF	32

/* device bootup time in millisecond */
#define POWER_UP_TIME_MS	2
/* delay to wait gyro engine stable in millisecond */
#define SENSOR_UP_TIME_MS	5
/* delay between power operation in microsecond */
#define POWER_EN_DELAY_US	10

/* initial configure */
#define INIT_FIFO_RATE		1000//200

#define DEFAULT_MOT_THR		0x08// 1
#define DEFAULT_MOT_DET_EN	0xC0

/* chip reset wait */
#define ICM_RESET_RETRY_CNT	10
#define ICM_RESET_WAIT_MS	100
#define ICM_RESET_SLEEP_US	10

/* FIFO related constant */
#define ICM_FIFO_SIZE_BYTE	1024
#define ICM_FIFO_CNT_SIZE	2

enum icm_device_id {
    ICM20602_ID = 0x12,
    ICM20626_ID = 0x2D,
    ICM20690_ID = 0x20,
};

enum icm_fsr {
	ICM_FSR_250DPS = 0,
	ICM_FSR_500DPS,
	ICM_FSR_1000DPS,
	ICM_FSR_2000DPS,
	NUM_FSR
};

enum icm_filter {
	ICM_DLPF_256HZ_NOLPF2 = 0,
	ICM_DLPF_188HZ,
	ICM_DLPF_98HZ,
	ICM_DLPF_42HZ,
	ICM_DLPF_20HZ,
	ICM_DLPF_10HZ,
	ICM_DLPF_5HZ,
	ICM_DLPF_RESERVED,
	NUM_FILTER
};

enum icm_clock_source {
	ICM_CLK_INTERNAL = 0,
	ICM_CLK_PLL_X,
	NUM_CLK
};

enum icm_accl_fs {
	ACCEL_FS_02G = 0,
	ACCEL_FS_04G,
	ACCEL_FS_08G,
	ACCEL_FS_16G,
	NUM_ACCL_FSR
};

/* Sensitivity Scale Factor
 * Sensor HAL will take 1024 LSB/g
 */
enum icm_accel_fs_shift {
	ACCEL_SCALE_SHIFT_02G = 0,
	ACCEL_SCALE_SHIFT_04G = 1,
	ACCEL_SCALE_SHIFT_08G = 2,
	ACCEL_SCALE_SHIFT_16G = 3
};

enum icm_gyro_fs_shift {
	GYRO_SCALE_SHIFT_FS0 = 3,
	GYRO_SCALE_SHIFT_FS1 = 2,
	GYRO_SCALE_SHIFT_FS2 = 1,
	GYRO_SCALE_SHIFT_FS3 = 0
};

/* device enum */
enum inv_devices {
    INV_ICM20602,
    INV_ICM20626,
    INV_ICM20690,
    INV_NUM_PARTS
};

/*
 *  struct icm_reg_map_s - Notable slave registers.
 *  @sample_rate_div:	Divider applied to gyro output rate.
 *  @lpf:		Configures internal LPF.
 *  @fifo_en:	Determines which data will appear in FIFO.
 *  @gyro_config:	gyro config register.
 *  @accel_config:	accel config register
 *  @mot_thr:	Motion detection threshold.
 *  @fifo_count_h:	Upper byte of FIFO count.
 *  @fifo_r_w:	FIFO register.
 *  @raw_gyro:	Address of first gyro register.
 *  @raw_accl:	Address of first accel register.
 *  @temperature:	temperature register.
 *  @int_pin_cfg:	Interrupt pin and I2C bypass configuration.
 *  @int_enable:	Interrupt enable register.
 *  @int_status:	Interrupt flags.
 *  @user_ctrl:	User control.
 *  @pwr_mgmt_1:	Controls chip's power state and clock source.
 *  @pwr_mgmt_2:	Controls power state of individual sensors.
 */
struct icm_reg_map {
	u8 sample_rate_div;
	u8 lpf;
	u8 fifo_en;
	u8 gyro_config;
	u8 accel_config;
	u8 fifo_count_h;
	u8 mot_thr;
	u8 mot_ctrl;
	u8 fifo_r_w;
	u8 raw_gyro;
	u8 raw_accel;
	u8 temperature;
	u8 int_pin_cfg;
	u8 int_enable;
	u8 int_status;
	u8 user_ctrl;
	u8 pwr_mgmt_1;
	u8 pwr_mgmt_2;
};

/*
 *  struct icm_chip_config - Cached chip configuration data.
 *  @fsr:		Full scale range.
 *  @lpf:		Digital low pass filter frequency.
 *  @accl_fs:		accel full scale range.
 *  @enable:		master enable to enable output
 *  @accel_enable:		enable accel functionality
 *  @accel_fifo_enable:	enable accel data output
 *  @gyro_enable:		enable gyro functionality
 *  @gyro_fifo_enable:	enable gyro data output
 *  @is_asleep:		1 if chip is powered down.
 *  @lpa_mode:		low power mode.
 *  @tap_on:		tap on/off.
 *  @flick_int_on:		flick interrupt on/off.
 *  @int_enabled:		interrupt is enabled.
 *  @mot_det_on:		motion detection wakeup enabled.
 *  @cfg_fifo_en:		FIFO R/W is enabled in USER_CTRL register.
 *  @int_pin_cfg:		interrupt pin configuration.
 *  @lpa_freq:		frequency of low power accelerometer.
 *  @rate_div:		Sampling rate divider.
 */
struct icm_chip_config {
	u32 fsr:2;
	u32 lpf:3;
	u32 accel_fs:2;
	u32 enable:1;
	u32 accel_enable:1;
	u32 accel_fifo_enable:1;
	u32 gyro_enable:1;
	u32 gyro_fifo_enable:1;
	u32 is_asleep:1;
	u32 lpa_mode:1;
	u32 tap_on:1;
	u32 flick_int_on:1;
	u32 int_enabled:1;
	u32 mot_det_on:1;
	u32 cfg_fifo_en:1;
	u8 int_pin_cfg;
	u16 lpa_freq;
	u16 rate_div;
};

/*
 *  struct icm_platform_data - device platform dependent data.
 *  @gpio_en:		enable GPIO.
 *  @gpio_int:		interrupt GPIO.
 *  @int_flags:		interrupt pin control flags.
 *  @use_int:		use interrupt mode instead of polling data.
 *  @place:			sensor place number.
 */
struct icm_platform_data {
	int gpio_en;
	int gpio_int;
	u32 int_flags;
	bool use_int;
	u8 place;
};


#define CAMERA_CCI

#ifdef CAMERA_CCI
#include <media/v4l2-event.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ioctl.h>
#include <media/cam_sensor.h>
#include <cam_sensor_i2c.h>
#include <cam_sensor_spi.h>
#include <cam_sensor_io.h>
#include <cam_cci_dev.h>
#include <cam_req_mgr_util.h>
#include <cam_req_mgr_interface.h>
#include <cam_mem_mgr.h>
#include <cam_subdev.h>
#include <cam_soc_util.h>

struct icm206xx_data {
    char name[64];
    void *client_object;
    bool is_device_remove;

    struct mutex work_mutex;
    struct delayed_work dwork;

    struct input_dev *input_dev_icm206xx;

    int is_first_irq;
    int is_first_start_done;

    int poll_mode;
    int poll_delay_ms;
    int enable_sensor;
    struct timeval start_tv;
    int enable_debug;
    bool allow_hidden_start_stop;
};
#endif

#define icm_errmsg(str, args...) \
	printk("[icm206xx][error]%s: " str, __func__, ##args)

#define icm_dbgmsg(str, args...) \
	if (icm_enable_debug) \
		printk("[icm206xx]%s: " str, __func__, ##args)

bool icm_enable_debug = true;		// set if verbose message is needed.

struct icm_ctrl_t *g_icm_ctrl = NULL;
struct icm_sensor *g_icm206xx_sensor;
static bool g_debugMode;
#define ICM206XX_ACCEL_DATA_SIZE	3
#define ASUS_2ND_ACCEL_SENSOR_IOC_MAGIC                      ('C')	///< icm206xx accel ioctl magic number 
#define ICM206XX_ACCEL_IOCTL_DATA_READ           _IOR(ASUS_2ND_ACCEL_SENSOR_IOC_MAGIC, 1, int[ICM206XX_ACCEL_DATA_SIZE])	///< icm206xx accel ioctl command - Read data xyz
#define ASUS_2ND_ACCEL_SENSOR_IOCTL_DEBUG_MODE           _IOW(ASUS_2ND_ACCEL_SENSOR_IOC_MAGIC, 2, int)	///< RGB sensor ioctl command - Get debug mode

#define ICM206XX_GYRO_DATA_SIZE	3
#define ASUS_2ND_GYRO_SENSOR_IOC_MAGIC                      ('C')	///< icm206xx accel ioctl magic number 
#define ICM206XX_GYRO_IOCTL_DATA_READ           _IOR(ASUS_2ND_GYRO_SENSOR_IOC_MAGIC, 1, int[ICM206XX_GYRO_DATA_SIZE])	///< icm206xx gyro ioctl command - Read data xyz

#endif /* __ICM206XX_H__ */
