/*
 * ICM 6-axis gyroscope + accelerometer driver
 *
 * Copyright (c) 2014-2015, 2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include "icm206xx.h"
#include <linux/kthread.h>
#include <linux/proc_fs.h>

#ifdef CAMERA_CCI
#include "cam_sensor_cmn_header.h"
#include "cam_sensor_i2c.h"
#include "cam_cci_dev.h"
#endif //CAMERA_CCI

#define DEBUG_NODE
#define DEBUG

//#define SENSORS_CLASSDEV
//struct sensors_classdev
#ifdef SENSORS_CLASSDEV
#include <linux/sensors.h>
#endif

#define INVN_TAG    "ICM206XX"

#define IS_ODD_NUMBER(x)	(x & 1UL)

/* VDD 2.375V-3.46V VLOGIC 1.8V +-5% */
#define ICM_VDD_MIN_UV	2500000
#define ICM_VDD_MAX_UV	3400000
#define ICM_VLOGIC_MIN_UV	1800000
#define ICM_VLOGIC_MAX_UV	1800000
#define ICM_VI2C_MIN_UV	1750000
#define ICM_VI2C_MAX_UV	1950000

#define ICM_ACCEL_MIN_VALUE	-40000
#define ICM_ACCEL_MAX_VALUE	32767
#define ICM_GYRO_MIN_VALUE	-40000
#define ICM_GYRO_MAX_VALUE	32767

#define ICM_MAX_EVENT_CNT	170
/* Limit mininum delay to 10ms as we do not need higher rate so far */
#define ICM_ACCEL_MIN_POLL_INTERVAL_MS	5//10
#define ICM_ACCEL_MAX_POLL_INTERVAL_MS	5000
#define ICM_ACCEL_DEFAULT_POLL_INTERVAL_MS	10//200
#define ICM_ACCEL_INT_MAX_DELAY			19

#define ICM_GYRO_MIN_POLL_INTERVAL_MS	5//10
#define ICM_GYRO_MAX_POLL_INTERVAL_MS	5000
#define ICM_GYRO_DEFAULT_POLL_INTERVAL_MS	10//200
#define ICM_GYRO_INT_MAX_DELAY		18

#define ICM_RAW_ACCEL_DATA_LEN	6
#define ICM_RAW_GYRO_DATA_LEN	6

//#define ICM_DEV_NAME_ACCEL	"ICM-accel"
//#define ICM_DEV_NAME_GYRO	"gyroscope"
#define ICM_DEV_NAME_ACCEL	INVN_TAG"-accel"
#define ICM_DEV_NAME_GYRO	INVN_TAG"-gyro"


#define ICM_PINCTRL_DEFAULT	"cam_default"
#define ICM_PINCTRL_SUSPEND	"cam_suspend"

#define CAL_SKIP_COUNT	5
#define ICM_ACC_CAL_COUNT	15
#define ICM_ACC_CAL_NUM	(ICM_ACC_CAL_COUNT - CAL_SKIP_COUNT)
#define ICM_ACC_CAL_BUF_SIZE	22
#define RAW_TO_1G	16384
#define ACCELDATAUNIT	9807
#define ICM_ACC_CAL_DELAY	100	/* ms */
#define POLL_MS_100HZ	10
#define SNS_TYPE_GYRO	0
#define SNS_TYPE_ACCEL	1


#define TO_BE_REFINED	//functions under construction

#ifdef CAMERA_CCI
#define ICM_SENSOR_NAME "icm20690"

static struct cam_subdev *g_icm_v4l2_dev_str = NULL;

struct icm_ctrl_t {
    struct platform_device *pdev;
    enum msm_camera_device_type_t device_type;
    enum cci_device_num cci_num;
    enum cci_i2c_master_t cci_master;
    struct camera_io_master io_master_info;
    struct cam_subdev v4l2_dev_str;
    struct icm206xx_data *icm206xx;
    struct msm_pinctrl_info pinctrl_info;
    uint8_t cam_pinctrl_status;
    char device_name[20];

    struct kref ref;
    struct regulator *power_supply;
    struct regulator *cci_supply;

    int pwren_gpio;
    int intr_gpio;
    int boot_reg;

    struct delayed_work dwork;

    struct hw_data_flags_t {
        unsigned pwr_owned    : 1;
        unsigned intr_owned   : 1;
        unsigned intr_started : 1;
    } io_flag;

    int irq;
};

int icm_init_cci(void);
void icm_exit_cci(void *object);
void icm_clean_up_cci(void);
int icm206xx_power_ctl_cci(void *object, bool enable);
int icm206xx_power_up_cci(void *);
int icm206xx_power_down_cci(void *);
void icm206xx_clean_up_cci(void);
int icm206xx_start_intr_cci(void *object, int *poll_mode);
void *icm206xx_get_cci(void *);
void icm206_put_cci(void *);

struct icm206xx_data *icm206xx;
#endif //CAMERA_CCI

int g_status = 0; 

enum icm_place {
	ICM_PLACE_PU = 0,
	ICM_PLACE_PR = 1,
	ICM_PLACE_LD = 2,
	ICM_PLACE_LL = 3,
	ICM_PLACE_PU_BACK = 4,
	ICM_PLACE_PR_BACK = 5,
	ICM_PLACE_LD_BACK = 6,
	ICM_PLACE_LL_BACK = 7,
	ICM_PLACE_UNKNOWN = 8,
	ICM_AXIS_REMAP_TAB_SZ = 8
};

struct icm_place_name {
	char name[32];
	enum icm_place place;
};

struct axis_data {
	s16 x;
	s16 y;
	s16 z;
	s16 rx;
	s16 ry;
	s16 rz;
};


/*
 *  struct icm_sensor - Cached chip configuration data
 *  @client:		I2C client
 *  @dev:		device structure
 *  @accel_dev:		accelerometer input device structure
 *  @gyro_dev:		gyroscope input device structure
 *  @accel_cdev:		sensor class device structure for accelerometer
 *  @gyro_cdev:		sensor class device structure for gyroscope
 *  @pdata:	device platform dependent data
 *  @op_lock:	device operation mutex
 *  @chip_type:	sensor hardware model
 *  @fifo_flush_work:	work structure to flush sensor fifo
 *  @reg:		notable slave registers
 *  @cfg:		cached chip configuration data
 *  @axis:	axis data reading
 *  @gyro_poll_ms:	gyroscope polling delay
 *  @accel_poll_ms:	accelerometer polling delay
 *  @accel_latency_ms:	max latency for accelerometer batching
 *  @gyro_latency_ms:	max latency for gyroscope batching
 *  @accel_en:	accelerometer enabling flag
 *  @gyro_en:	gyroscope enabling flag
 *  @use_poll:		use polling mode instead of  interrupt mode
 *  @motion_det_en:	motion detection wakeup is enabled
 *  @batch_accel:	accelerometer is working on batch mode
 *  @batch_gyro:	gyroscope is working on batch mode
 *  @acc_cal_buf:	accelerometer calibration string format bias
 *  @acc_cal_params:	accelerometer calibration bias
 *  @acc_use_cal:	accelerometer use calibration bias to
			compensate raw data
 *  @vlogic:	regulator data for Vlogic
 *  @vdd:	regulator data for Vdd
 *  @vi2c:	I2C bus pullup
 *  @enable_gpio:	enable GPIO
 *  @power_enabled:	flag of device power state
 *  @pinctrl:	pinctrl struct for interrupt pin
 *  @pin_default:	pinctrl default state
 *  @pin_sleep:	pinctrl sleep state
 *  @flush_count:	number of flush
 *  @fifo_start_ns:		timestamp of first fifo data
 */
 
struct icm_sensor {
#ifdef CAMERA_CCI							
	struct platform_device *pdev;
	struct cam_sensor_cci_client *cci_client;
	struct camera_io_master *io_master_info;
    int irq;                   /* irq issued by device           */	
#endif //CAMERA_CCI
	struct device *dev;
	struct hrtimer gyro_timer;
	struct hrtimer accel_timer;
	struct input_dev *accel_dev;
	struct input_dev *gyro_dev;
#ifdef SENSORS_CLASSDEV
	struct sensors_classdev accel_cdev;
	struct sensors_classdev gyro_cdev;
#endif
	struct icm_platform_data *pdata;
	struct mutex op_lock;
	enum inv_devices chip_type;
	struct workqueue_struct *data_wq;
	struct work_struct resume_work;
	struct delayed_work fifo_flush_work;
	struct icm_reg_map reg;
	struct icm_chip_config cfg;
	struct axis_data axis;
	struct wakeup_source icm206xx_wakeup_source;
	u32 deviceid;
	u32 gyro_poll_ms;
	u32 accel_poll_ms;
	u32 accel_latency_ms;
	u32 gyro_latency_ms;
	atomic_t accel_en;
	atomic_t gyro_en;
	bool use_poll;
	bool motion_det_en;
	bool batch_accel;
	bool batch_gyro;

	/* calibration */
	char acc_cal_buf[ICM_ACC_CAL_BUF_SIZE];
	int acc_cal_params[3];
	bool acc_use_cal;

	/* power control */
	struct regulator *vlogic;
	struct regulator *vdd;
	struct regulator *vi2c;
	int enable_gpio;
	bool power_enabled;

	/* pinctrl */
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	struct pinctrl_state *pin_sleep;

	u32 flush_count;
	u64 fifo_start_ns;
	int gyro_wkp_flag;
	int accel_wkp_flag;
	struct task_struct *gyr_task;
	struct task_struct *accel_task;
	bool gyro_delay_change;
	bool accel_delay_change;
	wait_queue_head_t	gyro_wq;
	wait_queue_head_t	accel_wq;
};


#ifdef SENSORS_CLASSDEV
/* Accelerometer information read by HAL */
static struct sensors_classdev icm_acc_cdev = {
	.name = "ICM-accel",
	.vendor = "Invensense",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = "156.8",	/* m/s^2 */
	.resolution = "0.000598144",	/* m/s^2 */
	.sensor_power = "0.5",	/* 0.5 mA */
	.min_delay = ICM_ACCEL_MIN_POLL_INTERVAL_MS * 1000,
	.max_delay = ICM_ACCEL_MAX_POLL_INTERVAL_MS,
	.delay_msec = ICM_ACCEL_DEFAULT_POLL_INTERVAL_MS,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.max_latency = 0,
	.flags = 0, /* SENSOR_FLAG_CONTINUOUS_MODE */
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_enable_wakeup = NULL,
	.sensors_set_latency = NULL,
	.sensors_flush = NULL,
};

/* gyroscope information read by HAL */
static struct sensors_classdev icm_gyro_cdev = {
	.name = "ICM-gyro",
	.vendor = "Invensense",
	.version = 1,
	.handle = SENSORS_GYROSCOPE_HANDLE,
	.type = SENSOR_TYPE_GYROSCOPE,
	.max_range = "34.906586",	/* rad/s */
	.resolution = "0.0010681152",	/* rad/s */
	.sensor_power = "3.6",	/* 3.6 mA */
	.min_delay = ICM_GYRO_MIN_POLL_INTERVAL_MS * 1000,
	.max_delay = ICM_GYRO_MAX_POLL_INTERVAL_MS,
	.delay_msec = ICM_ACCEL_DEFAULT_POLL_INTERVAL_MS,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.max_latency = 0,
	.flags = 0, /* SENSOR_FLAG_CONTINUOUS_MODE */
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_enable_wakeup = NULL,
	.sensors_set_latency = NULL,
	.sensors_flush = NULL,
};
#endif //#ifdef SENSORS_CLASSDEV

struct sensor_axis_remap {
	/* src means which source will be mapped to target x, y, z axis
	 * if an target OS axis is remapped from (-)x,
	 * src is 0, sign_* is (-)1
	 * if an target OS axis is remapped from (-)y,
	 * src is 1, sign_* is (-)1
	 * if an target OS axis is remapped from (-)z,
	 * src is 2, sign_* is (-)1
	 */
	int src_x:3;
	int src_y:3;
	int src_z:3;

	int sign_x:2;
	int sign_y:2;
	int sign_z:2;
};

static const struct sensor_axis_remap
icm_accel_axis_remap_tab[ICM_AXIS_REMAP_TAB_SZ] = {
	/* src_x src_y src_z  sign_x  sign_y  sign_z */
	{  0,    1,    2,     1,      1,      1 }, /* P0 */
	{  1,    0,    2,     1,     -1,      1 }, /* P1 */
	{  0,    1,    2,    -1,     -1,      1 }, /* P2 */
	{  1,    0,    2,    -1,      1,      1 }, /* P3 */

	{  0,    1,    2,    -1,      1,     -1 }, /* P4 */
	{  1,    0,    2,    -1,     -1,     -1 }, /* P5 */
	{  0,    1,    2,     1,     -1,     -1 }, /* P6 */
	{  1,    0,    2,     1,      1,     -1 }, /* P7 */
};

static const struct sensor_axis_remap
icm_gyro_axis_remap_tab[ICM_AXIS_REMAP_TAB_SZ] = {
	/* src_x src_y src_z  sign_x  sign_y  sign_z */
	{  0,    1,    2,    -1,      1,     -1 }, /* P0 */
	{  1,    0,    2,    -1,     -1,     -1 }, /* P1*/
	{  0,    1,    2,     1,     -1,     -1 }, /* P2 */
	{  1,    0,    2,     1,      1,     -1 }, /* P3 */

	{  0,    1,    2,     1,      1,      1 }, /* P4 */
	{  1,    0,    2,     1,     -1,      1 }, /* P5 */
	{  0,    1,    2,    -1,     -1,      1 }, /* P6 */
	{  1,    0,    2,    1,      1,      -1 }, /* P7 */
};

static const struct icm_place_name
icm_place_name2num[ICM_AXIS_REMAP_TAB_SZ] = {
	{"Portrait Up", ICM_PLACE_PU},
	{"Landscape Right", ICM_PLACE_PR},
	{"Portrait Down", ICM_PLACE_LD},
	{"Landscape Left", ICM_PLACE_LL},
	{"Portrait Up Back Side", ICM_PLACE_PU_BACK},
	{"Landscape Right Back Side", ICM_PLACE_PR_BACK},
	{"Portrait Down Back Side", ICM_PLACE_LD_BACK},
	{"Landscape Left Back Side", ICM_PLACE_LL_BACK},
};

/* Map gyro measurement range setting to number of bit to shift */
static const u8 icm_gyro_fs_shift[NUM_FSR] = {
	GYRO_SCALE_SHIFT_FS0, /* ICM_FSR_250DPS */
	GYRO_SCALE_SHIFT_FS1, /* ICM_FSR_500DPS */
	GYRO_SCALE_SHIFT_FS2, /* ICM_FSR_1000DPS */
	GYRO_SCALE_SHIFT_FS3, /* ICM_FSR_2000DPS */
};

/* Map accel measurement range setting to number of bit to shift */
static const u8 icm_accel_fs_shift[NUM_ACCL_FSR] = {
	ACCEL_SCALE_SHIFT_02G, /* ACCEL_FS_02G */
	ACCEL_SCALE_SHIFT_04G, /* ACCEL_FS_04G */
	ACCEL_SCALE_SHIFT_08G, /* ACCEL_FS_08G */
	ACCEL_SCALE_SHIFT_16G, /* ACCEL_FS_16G */
};

/* Function declarations */
static int gyro_poll_thread(void *data);
static int accel_poll_thread(void *data);
static void icm_pinctrl_state(struct icm_sensor *sensor,
			bool active);
static int icm_set_interrupt(struct icm_sensor *sensor,
		const u8 mask, bool on);
static int icm_set_fifo(struct icm_sensor *sensor,
					bool en_accel, bool en_gyro);
static void icm_flush_fifo(struct icm_sensor *sensor);
static int icm_config_sample_rate(struct icm_sensor *sensor);
static int icm_acc_data_process(struct icm_sensor *sensor);
static void icm_resume_work_fn(struct work_struct *work);

#ifdef CAMERA_CCI
int icm_probe_cci(struct platform_device *pdev);
int icm_remove_cci(struct platform_device *pdev);
s32 cci_read_byte_data(struct icm_sensor *sensor, u8 command);
s32 cci_write_byte_data(struct icm_sensor *sensor, u8 command, u8 value);
s32 cci_mask_write_byte_data(struct icm_sensor *sensor, u8 command, u8 mask, u8 value);
#endif //CAMERA_CCI

static inline void icm_set_fifo_start_time(struct icm_sensor *sensor)
{
	struct timespec ts;

	get_monotonic_boottime(&ts);
	sensor->fifo_start_ns = timespec_to_ns(&ts);
}
static int icm_power_ctl(struct icm_sensor *sensor, bool on)
{
	int rc = 0;

	if (on && (!sensor->power_enabled)) {

		icm_pinctrl_state(sensor, true);

		sensor->power_enabled = true;
	} else if (!on && (sensor->power_enabled)) {
		icm_pinctrl_state(sensor, false);
		sensor->power_enabled = false;
	} else {
		dev_warn(sensor->dev,
				"Ignore power status change from %d to %d\n",
				on, sensor->power_enabled);
	}

	return rc;
}

static int icm_power_init(struct icm_sensor *sensor)
{
	int ret = 0;

#if 0
	sensor->vdd = regulator_get(sensor->dev, "vdd");
	if (IS_ERR(sensor->vdd)) {
		ret = PTR_ERR(sensor->vdd);
		dev_err(sensor->dev,
			"Regulator get failed vdd ret=%d\n", ret);
		return ret;
	}

	if (regulator_count_voltages(sensor->vdd) > 0) {
		ret = regulator_set_voltage(sensor->vdd, ICM_VDD_MIN_UV,
					   ICM_VDD_MAX_UV);
		if (ret) {
			dev_err(sensor->dev,
				"Regulator set_vtg failed vdd ret=%d\n", ret);
			goto reg_vdd_put;
		}
	}

	sensor->vlogic = regulator_get(sensor->dev, "vlogic");
	if (IS_ERR(sensor->vlogic)) {
		ret = PTR_ERR(sensor->vlogic);
		dev_err(sensor->dev,
			"Regulator get failed vlogic ret=%d\n", ret);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(sensor->vlogic) > 0) {
		ret = regulator_set_voltage(sensor->vlogic,
				ICM_VLOGIC_MIN_UV,
				ICM_VLOGIC_MAX_UV);
		if (ret) {
			dev_err(sensor->dev,
			"Regulator set_vtg failed vlogic ret=%d\n", ret);
			goto reg_vlogic_put;
		}
	}

	sensor->vi2c = regulator_get(sensor->dev, "vi2c");
	if (IS_ERR(sensor->vi2c)) {
		ret = PTR_ERR(sensor->vi2c);
		dev_info(sensor->dev,
			"Regulator get failed vi2c ret=%d\n", ret);
		sensor->vi2c = NULL;
	} else if (regulator_count_voltages(sensor->vi2c) > 0) {
		ret = regulator_set_voltage(sensor->vi2c,
				ICM_VI2C_MIN_UV,
				ICM_VI2C_MAX_UV);
		if (ret) {
			dev_err(sensor->dev,
			"Regulator set_vtg failed vi2c ret=%d\n", ret);
			goto reg_vi2c_put;
		}
	}

	return 0;

reg_vi2c_put:
	regulator_put(sensor->vi2c);
	if (regulator_count_voltages(sensor->vlogic) > 0)
		regulator_set_voltage(sensor->vlogic, 0, ICM_VLOGIC_MAX_UV);
reg_vlogic_put:
	regulator_put(sensor->vlogic);
reg_vdd_set_vtg:
	if (regulator_count_voltages(sensor->vdd) > 0)
		regulator_set_voltage(sensor->vdd, 0, ICM_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(sensor->vdd);
#endif

	return ret;
}

static int icm_power_deinit(struct icm_sensor *sensor)
{
	int ret = 0;

#if 0
	if (regulator_count_voltages(sensor->vlogic) > 0)
		regulator_set_voltage(sensor->vlogic, 0, ICM_VLOGIC_MAX_UV);
	regulator_put(sensor->vlogic);
	if (regulator_count_voltages(sensor->vdd) > 0)
		regulator_set_voltage(sensor->vdd, 0, ICM_VDD_MAX_UV);
	regulator_put(sensor->vdd);
#endif

	return ret;
}

/*
 * icm_read_reg() - read multiple register data
 * @start_addr: register address read from
 * @buffer: provide register addr and get register
 * @length: length of register
 *
 * Reads the register values in one transaction or returns a negative
 * error code on failure.
 */
static int icm_read_reg(struct icm_sensor *sensor, u8 start_addr,
			       u8 *buffer, int length)
{
	int ret;
	if(!sensor || !sensor->cci_client){
		icm_errmsg("cci_client is not initialized.\n");
		return -EINVAL;
	}
	
	ret = cam_camera_cci_i2c_read_seq(sensor->cci_client, start_addr, buffer, CAMERA_SENSOR_I2C_TYPE_BYTE,
				CAMERA_SENSOR_I2C_TYPE_BYTE, length);
	return ret;
}

/*
 * icm_read_accel_data() - get accelerometer data from device
 * @sensor: sensor device instance
 * @data: axis data to update
 *
 * Return the converted X Y and Z data from the sensor device
 */
static int icm_read_accel_data(struct icm_sensor *sensor,
			     struct axis_data *data)
{
	int ret = 0;
	u16 buffer[3];

	ret = icm_read_reg(sensor, sensor->reg.raw_accel,
		(u8 *)buffer, ICM_RAW_ACCEL_DATA_LEN);
	if (!ret) {
		data->x = be16_to_cpu(buffer[0]);
		data->y = be16_to_cpu(buffer[1]);
		data->z = be16_to_cpu(buffer[2]);
	}
	return ret;
}

/*
 * icm_read_gyro_data() - get gyro data from device
 * @sensor: sensor device instance
 * @data: axis data to update
 *
 * Return the converted RX RY and RZ data from the sensor device
 */
static int icm_read_gyro_data(struct icm_sensor *sensor,
			     struct axis_data *data)
{
	int ret = 0;
	u16 buffer[3];

	ret = icm_read_reg(sensor, sensor->reg.raw_gyro,
		(u8 *)buffer, ICM_RAW_GYRO_DATA_LEN);
	if (!ret) {
		data->rx = be16_to_cpu(buffer[0]);
		data->ry = be16_to_cpu(buffer[1]);
		data->rz = be16_to_cpu(buffer[2]);

		data->rx = data->rx; //* 3140 / 164 / 180;
		data->ry = data->ry; //* 3140 / 164 / 180;
		data->rz = data->rz; //* 3140 / 164 / 180;
	}
	return ret;
}

/*
 * icm_remap_accel_data() - remap accelerometer raw data to axis data
 * @data: data needs remap
 * @place: sensor position
 */
static void icm_remap_accel_data(struct axis_data *data, int place)
{
	const struct sensor_axis_remap *remap;
	s16 tmp[3];
	/* sensor with place 0 needs not to be remapped */
	if ((place <= 0) || (place >= ICM_AXIS_REMAP_TAB_SZ))
		return;

	remap = &icm_accel_axis_remap_tab[place];

	tmp[0] = data->x;
	tmp[1] = data->y;
	tmp[2] = data->z;
	data->x = tmp[remap->src_x] * remap->sign_x;
	data->y = tmp[remap->src_y] * remap->sign_y;
	data->z = tmp[remap->src_z] * remap->sign_z;
}

/*
 * icm_remap_gyro_data() - remap gyroscope raw data to axis data
 * @data: data to remap
 * @place: sensor position
 */
static void icm_remap_gyro_data(struct axis_data *data, int place)
{
	const struct sensor_axis_remap *remap;
	s16 tmp[3];
	/* sensor with place 0 needs not to be remapped */
	if ((place <= 0) || (place >= ICM_AXIS_REMAP_TAB_SZ))
		return;

	remap = &icm_gyro_axis_remap_tab[place];
	tmp[0] = data->rx;
	tmp[1] = data->ry;
	tmp[2] = data->rz;
	data->rx = tmp[remap->src_x] * remap->sign_x;
	data->ry = tmp[remap->src_y] * remap->sign_y;
	data->rz = tmp[remap->src_z] * remap->sign_z;
}
static bool Gyro_debug_log_f = 0;
static bool G_debug_log_f = 0;
static bool g_skip_first_data = false;
static bool gyro_data_ready = true;
static u64 gyro_last_enable_time_ns = 0;
/*ASUS BSP: this function check if waiting enough time (80ms) since gyro sensor enabled
			there is a counter used to prevent data always not ready*/
bool icm_is_gyro_data_ready()
{
	struct timespec ts;
	static int l_counter = 0;
	u64 l_current_time_ns;
	if (!gyro_data_ready) {
		l_counter++;
		get_monotonic_boottime(&ts);
		l_current_time_ns = timespec_to_ns(&ts);
		if (l_current_time_ns - gyro_last_enable_time_ns > 80000000) {
			icm_dbgmsg("done, skipped %d data\n", l_counter - 1);
			gyro_data_ready = true;
			l_counter = 0;
		}
		if (l_counter > 50) {
			icm_errmsg("skipped more than 50 data! enabled time = %lld, current time = %lld\n", gyro_last_enable_time_ns, l_current_time_ns);
			gyro_data_ready = true;
			l_counter = 0;
		}
	}
	return gyro_data_ready;
}

/*
 * icm_read_single_event() - handle one sensor event.
 * @sensor: sensor device instance
 *
 * It only reads one sensor event from sensor register and send it to
 * sensor HAL, FIFO overflow and motion detection interrupt should be
 * handle by separate function.
 */
static struct timespec g_timestamp;
static int icm_read_single_event(struct icm_sensor *sensor, struct timespec timestamp1)
{
	int ret = 0;
	u32 shift;

	if (sensor->cfg.accel_enable) {
		ret = icm_acc_data_process(sensor);
		if (ret) {
			return ret;
		}
		/*ASUS_BSP: skip first accelerometer data since it's wrong*/
		if (!g_skip_first_data) {
			if (G_debug_log_f == 1){
				icm_dbgmsg("G x=%d, y=%d, z=%d\n", sensor->axis.x, sensor->axis.y, sensor->axis.z);
				G_debug_log_f = 0;
			}
			shift = icm_accel_fs_shift[sensor->cfg.accel_fs];
			input_report_abs(sensor->accel_dev, ABS_X,
				(sensor->axis.x << shift));
			input_report_abs(sensor->accel_dev, ABS_Y,
				(sensor->axis.y << shift));
			input_report_abs(sensor->accel_dev, ABS_Z,
				(sensor->axis.z << shift));
			input_report_abs(sensor->accel_dev, ABS_WHEEL,
				timestamp1.tv_sec);
			input_report_abs(sensor->accel_dev, ABS_GAS,
				timestamp1.tv_nsec);
			input_sync(sensor->accel_dev);
		} else{
			g_skip_first_data = false;
			icm_dbgmsg("skip first g sensor data\n");
		}
	}

	if (sensor->cfg.gyro_enable) {
		ret = icm_read_gyro_data(sensor, &sensor->axis);
		if (ret) {
			return ret;
		}
		icm_remap_gyro_data(&sensor->axis,
			sensor->pdata->place);
		if (icm_is_gyro_data_ready()) {
			if (Gyro_debug_log_f == 1){
				icm_dbgmsg("Gyro x=%d, y=%d, z=%d\n", sensor->axis.rx, sensor->axis.ry, sensor->axis.rz);
				Gyro_debug_log_f = 0;
			}
			shift = icm_gyro_fs_shift[sensor->cfg.fsr];
			input_report_abs(sensor->gyro_dev, ABS_RX,
				(sensor->axis.rx >> shift));
			input_report_abs(sensor->gyro_dev, ABS_RY,
				(sensor->axis.ry >> shift));
			input_report_abs(sensor->gyro_dev, ABS_RZ,
				(sensor->axis.rz >> shift));
			input_report_abs(sensor->gyro_dev, ABS_WHEEL,
				timestamp1.tv_sec);
			input_report_abs(sensor->gyro_dev, ABS_GAS,
				timestamp1.tv_nsec);
			input_sync(sensor->gyro_dev);
		}
	}
	return 0;
}
static struct delayed_work data_retry_work;
static unsigned long g_next_retry_time_ms = 100;
void data_retry_wq(struct work_struct *work)
{
	int ret = 0;
	int l_gpio_value = 0;
	struct timespec l_timestamp;
	struct icm_sensor *sensor = NULL;
	if (!g_icm_ctrl) {
		icm_errmsg("null icm ctrl!\n");
		g_next_retry_time_ms = 0;
		return;
	}
	sensor = dev_get_drvdata(&g_icm_ctrl->pdev->dev);
	mutex_lock(&sensor->op_lock);
	l_gpio_value = gpio_get_value(sensor->pdata->gpio_int);

	/*ASUS BSP: if any of below conditions found, don't need to retry
		1. l_gpio_value = 0
			=> data has handled
		2. g_next_retry_time_ms = 0
			=> irq has triggered
		3. sensor->power_enabled = 0
			=> sensor has disabled
	*/
	if (l_gpio_value && g_next_retry_time_ms && sensor->power_enabled) {
		l_timestamp = ktime_to_timespec(ktime_get_boottime());
		ret = icm_read_single_event(sensor, l_timestamp);
	}
	if (!l_gpio_value || !g_next_retry_time_ms || !sensor->power_enabled || !ret) {
		icm_dbgmsg("stop retry work with l_gpio_value = %d, retry_time_ms = %lu, power_enabled = %d, ret = %d\n",
			l_gpio_value,
			g_next_retry_time_ms,
			sensor->power_enabled ? 1: 0,
			ret);
		g_next_retry_time_ms = 0;
	} else{
		/*ASUS BSP: double retry time until it reach 6s*/
		if (g_next_retry_time_ms < 6000) {
			g_next_retry_time_ms *= 2;
		}
		/*ASUS BSP: check the boundary of retry time*/
		if (g_next_retry_time_ms < 100) {
			g_next_retry_time_ms = 100;
		} else if (g_next_retry_time_ms > 6000) {
			g_next_retry_time_ms = 6000;
		}
		schedule_delayed_work(&data_retry_work, HZ * g_next_retry_time_ms / 1000);
	}
	mutex_unlock(&sensor->op_lock);
}
static u64 g_irq_counter = 0;
/*
 * icm_interrupt_thread() - handle an IRQ
 * @irq: interrupt number
 * @data: the sensor device
 *
 * Called by the kernel single threaded after an interrupt occurs. Read
 * the sensor data and generate an input event for it.
 */
static irqreturn_t icm_interrupt_routine(int irq, void *data)
{
	g_timestamp = ktime_to_timespec(ktime_get_boottime());
	return IRQ_WAKE_THREAD;
}
static irqreturn_t icm_interrupt_thread(int irq, void *data)
{
	int ret = 0;
	struct icm_sensor *sensor = data;
	g_irq_counter++;
	mutex_lock(&sensor->op_lock);
	/*ASUS BSP: stop retry work when irq triggered*/
	g_next_retry_time_ms = 0;
	if (!sensor->power_enabled) {
		icm_errmsg("power_enabled = false, just exit\n");
		goto exit;
	}
	ret = icm_read_single_event(sensor, g_timestamp);
	if (ret) {
		icm_errmsg("data read failed, start retry work!\n");
		g_next_retry_time_ms = 100;
		schedule_delayed_work(&data_retry_work, HZ * g_next_retry_time_ms / 1000);
	}
	

exit:
	mutex_unlock(&sensor->op_lock);

	return IRQ_HANDLED;
}

static void icm_sche_next_flush(struct icm_sensor *sensor)
{
	u32 latency;

	if ((sensor->batch_accel) && (sensor->batch_gyro)) {
		if (sensor->gyro_latency_ms < sensor->accel_latency_ms)
			latency = sensor->gyro_latency_ms;
		else
			latency = sensor->accel_latency_ms;
	} else if (sensor->batch_accel)
		latency = sensor->accel_latency_ms;
	else if (sensor->batch_gyro)
		latency = sensor->gyro_latency_ms;
	else
		latency = 0;

	if (latency != 0)
		queue_delayed_work(sensor->data_wq,
			&sensor->fifo_flush_work,
			msecs_to_jiffies(latency));
	else
		dev_err(sensor->dev,
			"unknown error, accel: en=%d latency=%d gyro: en=%d latency=%d\n",
			sensor->batch_accel,
			sensor->accel_latency_ms,
			sensor->batch_gyro,
			sensor->gyro_latency_ms);
}

/*
 * icm_fifo_flush_fn() - flush shared sensor FIFO
 * @work: the work struct
 */
static void icm_fifo_flush_fn(struct work_struct *work)
{
	struct icm_sensor *sensor = container_of(
				(struct delayed_work *)work,
				struct icm_sensor, fifo_flush_work);

	icm_flush_fifo(sensor);
	icm_sche_next_flush(sensor);
}

static int icm_manage_polling(int sns_type, struct icm_sensor *sensor)
{
	ktime_t ktime;
	int ret = 0;

	switch (sns_type) {
	case SNS_TYPE_GYRO:
		if (atomic_read(&sensor->gyro_en)) {
			ktime = ktime_set(0,
				sensor->gyro_poll_ms * NSEC_PER_MSEC);
			/*ret = */hrtimer_start(&sensor->gyro_timer,
					ktime,
					HRTIMER_MODE_REL);
		} else
			ret = hrtimer_try_to_cancel(&sensor->gyro_timer);
		break;

	case SNS_TYPE_ACCEL:
		if (atomic_read(&sensor->accel_en)) {
			ktime = ktime_set(0,
				sensor->accel_poll_ms * NSEC_PER_MSEC);
			/*ret = */hrtimer_start(&sensor->accel_timer,
					ktime,
					HRTIMER_MODE_REL);
		} else
			ret = hrtimer_try_to_cancel(&sensor->accel_timer);
		break;

	default:
		dev_err(sensor->dev, "Invalid sensor type\n");
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum hrtimer_restart gyro_timer_handle(struct hrtimer *hrtimer)
{
	struct icm_sensor *sensor;

	sensor = container_of(hrtimer, struct icm_sensor, gyro_timer);
	sensor->gyro_wkp_flag = 1;
	wake_up_interruptible(&sensor->gyro_wq);
	if (icm_manage_polling(SNS_TYPE_GYRO, sensor) < 0)
		dev_err(sensor->dev,
				"gyr: failed to start/cancel timer\n");

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart accel_timer_handle(struct hrtimer *hrtimer)
{
	struct icm_sensor *sensor;

	sensor = container_of(hrtimer, struct icm_sensor, accel_timer);
	sensor->accel_wkp_flag = 1;
	wake_up_interruptible(&sensor->accel_wq);
	if (icm_manage_polling(SNS_TYPE_ACCEL, sensor) < 0)
		dev_err(sensor->dev,
				"acc: failed to start/cancel timer\n");

	return HRTIMER_NORESTART;
}

static int gyro_poll_thread(void *data)
{
	struct icm_sensor *sensor = data;
	u32 shift;
	ktime_t timestamp;

	while (1) {
		wait_event_interruptible(sensor->gyro_wq,
			((sensor->gyro_wkp_flag != 0) ||
				kthread_should_stop()));
		sensor->gyro_wkp_flag = 0;

		if (kthread_should_stop())
			break;

		mutex_lock(&sensor->op_lock);
		if (sensor->gyro_delay_change) {
			if (sensor->gyro_poll_ms <= POLL_MS_100HZ)
				set_wake_up_idle(true);
			else
				set_wake_up_idle(false);
			sensor->gyro_delay_change = false;
		}
		mutex_unlock(&sensor->op_lock);

		timestamp = ktime_get_boottime();
		icm_read_gyro_data(sensor, &sensor->axis);
		icm_remap_gyro_data(&sensor->axis, sensor->pdata->place);
		shift = icm_gyro_fs_shift[sensor->cfg.fsr];
		input_report_abs(sensor->gyro_dev, ABS_RX,
			(sensor->axis.rx >> shift));
		input_report_abs(sensor->gyro_dev, ABS_RY,
			(sensor->axis.ry >> shift));
		input_report_abs(sensor->gyro_dev, ABS_RZ,
			(sensor->axis.rz >> shift));
#if 0
		input_event(sensor->gyro_dev,
				EV_SYN, SYN_TIME_SEC,
				ktime_to_timespec(timestamp).tv_sec);
		input_event(sensor->gyro_dev, EV_SYN,
			SYN_TIME_NSEC,
			ktime_to_timespec(timestamp).tv_nsec);
#else
		input_report_abs(sensor->gyro_dev, ABS_WHEEL,
			ktime_to_timespec(timestamp).tv_sec);
		input_report_abs(sensor->gyro_dev, ABS_GAS,
			ktime_to_timespec(timestamp).tv_nsec);
#endif
		input_sync(sensor->gyro_dev);
	}

	return 0;
}

static int accel_poll_thread(void *data)
{
	struct icm_sensor *sensor = data;
	u32 shift;
	ktime_t timestamp;

	while (1) {
		wait_event_interruptible(sensor->accel_wq,
			((sensor->accel_wkp_flag != 0) ||
				kthread_should_stop()));
		sensor->accel_wkp_flag = 0;

		if (kthread_should_stop())
			break;

		mutex_lock(&sensor->op_lock);
		if (sensor->accel_delay_change) {
			if (sensor->accel_poll_ms <= POLL_MS_100HZ)
				set_wake_up_idle(true);
			else
				set_wake_up_idle(false);
			sensor->accel_delay_change = false;
		}
		mutex_unlock(&sensor->op_lock);

		timestamp = ktime_get_boottime();
		icm_acc_data_process(sensor);
		shift = icm_accel_fs_shift[sensor->cfg.accel_fs];
		input_report_abs(sensor->accel_dev, ABS_X,
			(sensor->axis.x << shift));
		input_report_abs(sensor->accel_dev, ABS_Y,
			(sensor->axis.y << shift));
		input_report_abs(sensor->accel_dev, ABS_Z,
			(sensor->axis.z << shift));
#if 0
		input_event(sensor->accel_dev,
				EV_SYN, SYN_TIME_SEC,
				ktime_to_timespec(timestamp).tv_sec);
		input_event(sensor->accel_dev, EV_SYN,
			SYN_TIME_NSEC,
			ktime_to_timespec(timestamp).tv_nsec);
#else
        input_report_abs(sensor->accel_dev, ABS_WHEEL,
            ktime_to_timespec(timestamp).tv_sec);
        input_report_abs(sensor->accel_dev, ABS_GAS,
            ktime_to_timespec(timestamp).tv_nsec);
#endif
		input_sync(sensor->accel_dev);
	}

	return 0;
}

static int icm_switch_engine(struct icm_sensor *sensor,
				bool en, u32 mask)
{
	struct icm_reg_map *reg;
	int ret;

	reg = &sensor->reg;
	/*
	 * switch clock needs to be careful. Only when gyro is on, can
	 * clock source be switched to gyro. Otherwise, it must be set to
	 * internal clock
	 */
	if ((BIT_PWR_GYRO_STBY_MASK == mask) && (!en)) {
		/*
		 * turning off gyro requires switch to internal clock first.
		 * Then turn off gyro engine
		 */
		ret = cci_mask_write_byte_data(sensor,
			reg->pwr_mgmt_1, BIT_CLK_MASK, ICM_CLK_INTERNAL);
		if (ret < 0)
			goto error;
	}

	ret = cci_mask_write_byte_data(sensor,
			reg->pwr_mgmt_2, mask, en ? 0 : mask);
	if (ret < 0)
		goto error;

	if ((BIT_PWR_GYRO_STBY_MASK == mask) && en) {
		/* wait gyro stable */
		msleep(SENSOR_UP_TIME_MS);
		/* after gyro is on & stable, switch internal clock to PLL */
		ret = cci_mask_write_byte_data(sensor,
				reg->pwr_mgmt_1, BIT_CLK_MASK, ICM_CLK_PLL_X);
		if (ret < 0)
			goto error;
	}

	return 0;

error:
	dev_err(sensor->dev, "Fail to switch ICM engine\n");

	return ret;
}

static int icm_init_engine(struct icm_sensor *sensor)
{
	int ret;

	icm_dbgmsg("++\n");
	ret = icm_switch_engine(sensor, false, BIT_PWR_GYRO_STBY_MASK);
	if (ret)
		return ret;

	ret = icm_switch_engine(sensor, false, BIT_PWR_ACCEL_STBY_MASK);
	if (ret)
		return ret;

	icm_dbgmsg("--\n");
	return 0;
}

/*
 * icm_set_power_mode() - set the power mode
 * @sensor: sensor data structure
 * @power_on: value to switch on/off of power, 1: normal power,
 *    0: low power
 *
 * Put device to normal-power mode or low-power mode.
 */
static int icm_set_power_mode(struct icm_sensor *sensor,
					bool power_on)
{
	s32 ret;
	u8 val;

	ret = cci_read_byte_data(sensor, sensor->reg.pwr_mgmt_1);
	if (ret < 0) {
		dev_err(sensor->dev,
				"Fail to read power mode, ret=%d\n", ret);
		return ret;
	}

	if (power_on)
		val = (u8)ret & ~BIT_SLEEP;
	else
		val = (u8)ret | BIT_SLEEP;
	ret = cci_write_byte_data(sensor, sensor->reg.pwr_mgmt_1, val);
	if (ret < 0) {
		dev_err(sensor->dev,
				"Fail to write power mode, ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static int icm_gyro_enable(struct icm_sensor *sensor, bool on)
{
	int ret;

	if (sensor->cfg.is_asleep) {
		dev_err(sensor->dev,
			"Fail to set gyro state, device is asleep.\n");
		return -EINVAL;
	}

	if (on) {
		ret = cci_mask_write_byte_data(sensor,
				sensor->reg.pwr_mgmt_1, BIT_SLEEP, 0);
		if (ret < 0) {
			dev_err(sensor->dev,
				"Fail to set sensor power state, ret=%d\n",
				ret);
			return ret;
		}

		ret = icm_switch_engine(sensor, true,
			BIT_PWR_GYRO_STBY_MASK);
		if (ret)
			return ret;

		sensor->cfg.gyro_enable = 1;
		sensor->cfg.enable = 1;
	} else {
		if (work_pending(&sensor->resume_work))
			cancel_work_sync(&sensor->resume_work);
		ret = icm_switch_engine(sensor, false,
			BIT_PWR_GYRO_STBY_MASK);
		if (ret)
			return ret;
		sensor->cfg.gyro_enable = 0;
		if (!sensor->cfg.accel_enable) {
			ret = cci_mask_write_byte_data(sensor,
					sensor->reg.pwr_mgmt_1, BIT_SLEEP, BIT_SLEEP);
			if (ret < 0) {
				dev_err(sensor->dev,
					"Fail to set sensor power state, ret=%d\n",
					ret);
				return ret;
			}
			sensor->cfg.enable = 0;
		}
	}

	return 0;
}

/*
 * icm_restore_context() - update the sensor register context
 */
static int icm_restore_context(struct icm_sensor *sensor)
{
	struct icm_reg_map *reg;
	int ret;
	u8 data, pwr_ctrl;
	reg = &sensor->reg;

	/* Save power state and wakeup device from sleep */
	ret = cci_read_byte_data(sensor, reg->pwr_mgmt_1);
	if (ret < 0) {
		dev_err(sensor->dev, "read power ctrl failed.\n");
		goto exit;
	}
	pwr_ctrl = (u8)ret;

	ret = cci_write_byte_data(sensor, reg->pwr_mgmt_1,
		BIT_WAKEUP_AFTER_RESET);
	if (ret < 0) {
		dev_err(sensor->dev, "wakeup sensor failed.\n");
		goto exit;
	}

    if (sensor->chip_type == INV_ICM20690) {
        ret = cci_write_byte_data(sensor, reg->gyro_config,
                sensor->cfg.fsr << GYRO_CONFIG_FSR_SHIFT_ICM2069X);
    } else {
        ret = cci_write_byte_data(sensor, reg->gyro_config,
                sensor->cfg.fsr << GYRO_CONFIG_FSR_SHIFT_ICM2060X);
    }
	if (ret < 0) {
		dev_err(sensor->dev, "update fsr failed.\n");
		goto exit;
	}

	ret = cci_write_byte_data(sensor, reg->lpf, sensor->cfg.lpf);
	if (ret < 0) {
		dev_err(sensor->dev, "update lpf failed.\n");
		goto exit;
	}

	ret = cci_write_byte_data(sensor, reg->accel_config,
			(sensor->cfg.accel_fs << ACCL_CONFIG_FSR_SHIFT));
	if (ret < 0) {
		dev_err(sensor->dev, "update accel_fs failed.\n");
		goto exit;
	}

	ret = cci_write_byte_data(sensor, reg->sample_rate_div,
			sensor->cfg.rate_div);
	if (ret < 0) {
		dev_err(sensor->dev, "set sample_rate_div failed.\n");
		goto exit;
	}

	ret = cci_read_byte_data(sensor, reg->fifo_en);
	if (ret < 0) {
		dev_err(sensor->dev, "read fifo_en failed.\n");
		goto exit;
	}

	data = (u8)ret;

	if (sensor->cfg.accel_fifo_enable)
		data |= BIT_ACCEL_FIFO;

	if (sensor->cfg.gyro_fifo_enable)
		data |= BIT_GYRO_FIFO;

	if (sensor->cfg.accel_fifo_enable || sensor->cfg.gyro_fifo_enable) {
		ret = cci_write_byte_data(sensor, reg->fifo_en, data);
		if (ret < 0) {
			dev_err(sensor->dev, "write fifo_en failed.\n");
			goto exit;
		}
	}

	if (sensor->cfg.cfg_fifo_en) {
		/* Assume DMP and external I2C is not in use*/
		ret = cci_write_byte_data(sensor, reg->user_ctrl,
				BIT_FIFO_EN);
		if (ret < 0) {
			dev_err(sensor->dev, "enable FIFO R/W failed.\n");
			goto exit;
		}
	}

	/* Accel and Gyro should set to standby by default */
	ret = cci_write_byte_data(sensor, reg->pwr_mgmt_2,
			BITS_PWR_ALL_AXIS_STBY);
	if (ret < 0) {
		dev_err(sensor->dev, "set pwr_mgmt_2 failed.\n");
		goto exit;
	}

	ret = cci_write_byte_data(sensor, reg->int_pin_cfg,
			sensor->cfg.int_pin_cfg);
	if (ret < 0) {
		dev_err(sensor->dev, "set int_pin_cfg failed.\n");
		goto exit;
	}

	ret = cci_write_byte_data(sensor, reg->pwr_mgmt_1,
		pwr_ctrl);
	if (ret < 0) {
		dev_err(sensor->dev, "write saved power state failed.\n");
		goto exit;
	}

	dev_dbg(sensor->dev, "restore context finished\n");

exit:
	return ret;
}

/*
 * icm_reset_chip() - reset chip to default state
 */
static void icm_reset_chip(struct icm_sensor *sensor)
{
	int ret;

	ret = cci_write_byte_data(sensor, sensor->reg.pwr_mgmt_1,
			/*BIT_RESET_ALL*/BIT_H_RESET);
	if (ret < 0) {
		dev_err(sensor->dev, "Reset chip fail!\n");
		goto exit;
	}

#if 1
	mdelay(ICM_RESET_WAIT_MS);	// use mdelay instead of udelay, because linux MAX_UDELAY_MS of udealy is set at 2ms.
#else
	for (int i = 0; i < ICM_RESET_RETRY_CNT; i++) {
		ret = cci_read_byte_data(sensor,
					sensor->reg.pwr_mgmt_1);
		if (ret < 0) {
			dev_err(sensor->dev,
				"Fail to get reset state ret=%d\n", ret);
			goto exit;
		}

		if ((ret & BIT_H_RESET) == 0) {
			dev_dbg(sensor->dev,
				"Chip reset success! i=%d\n", i);
			break;
		}

		udelay(ICM_RESET_SLEEP_US);
	}
#endif

exit:
	return;
}

static int icm_gyro_batching_enable(struct icm_sensor *sensor)
{
	int ret = 0;
	u32 latency;

	if (!sensor->batch_accel) {
		latency = sensor->gyro_latency_ms;
	} else {
		cancel_delayed_work_sync(&sensor->fifo_flush_work);
		if (sensor->accel_latency_ms < sensor->gyro_latency_ms)
			latency = sensor->accel_latency_ms;
		else
			latency = sensor->gyro_latency_ms;
	}
	ret = icm_set_fifo(sensor, sensor->cfg.accel_enable, true);
	if (ret < 0) {
		dev_err(sensor->dev,
			"Fail to enable FIFO for gyro, ret=%d\n", ret);
		return ret;
	}

	if (sensor->use_poll) {
		queue_delayed_work(sensor->data_wq,
			&sensor->fifo_flush_work,
			msecs_to_jiffies(latency));
	} else if (!sensor->cfg.int_enabled) {
		icm_set_interrupt(sensor, BIT_FIFO_OVERFLOW, true);
		sensor->cfg.int_enabled = true;
	}

	return ret;
}

static int icm_gyro_batching_disable(struct icm_sensor *sensor)
{
	int ret = 0;
	u32 latency;

	ret = icm_set_fifo(sensor, sensor->cfg.accel_enable, false);
	if (ret < 0) {
		dev_err(sensor->dev,
			"Fail to disable FIFO for accel, ret=%d\n", ret);
		return ret;
	}
	if (!sensor->use_poll) {
		if (sensor->cfg.int_enabled && !sensor->cfg.accel_enable) {
			icm_set_interrupt(sensor,
				BIT_FIFO_OVERFLOW, false);
			sensor->cfg.int_enabled = false;
		}
	} else {
		if (!sensor->batch_accel) {
			cancel_delayed_work_sync(&sensor->fifo_flush_work);
		} else if (sensor->gyro_latency_ms <
				sensor->accel_latency_ms) {
			cancel_delayed_work_sync(&sensor->fifo_flush_work);
			latency = sensor->accel_latency_ms;
			queue_delayed_work(sensor->data_wq,
				&sensor->fifo_flush_work,
				msecs_to_jiffies(latency));
		}
	}
	sensor->batch_gyro = false;

	return ret;
}

static int icm_gyro_do_enable(struct icm_sensor *sensor, bool enable)
{
	int ret = 0;
	struct timespec ts;

	dev_dbg(sensor->dev,
		"icm_gyro_do_enable enable=%d\n", enable);
	mutex_lock(&sensor->op_lock);
	if (enable) {
		if (!sensor->power_enabled) {
			ret = icm_power_ctl(sensor, true);
			if (ret < 0) {
				dev_err(sensor->dev,
						"Failed to power up icm\n");
				goto exit;
			}
			ret = icm_restore_context(sensor);
			if (ret < 0) {
				dev_err(sensor->dev,
						"Failed to restore context\n");
				goto exit;
			}
		}

		ret = icm_gyro_enable(sensor, true);
		if (ret) {
			dev_err(sensor->dev,
				"Fail to enable gyro engine ret=%d\n", ret);
			ret = -EBUSY;
			goto exit;
		}
		atomic_set(&sensor->gyro_en, 1);

		ret = icm_config_sample_rate(sensor);
		if (ret < 0)
			dev_info(sensor->dev,
				"Unable to update sampling rate! ret=%d\n",
				ret);

		if (sensor->batch_gyro) {
			ret = icm_gyro_batching_enable(sensor);
			if (ret) {
				dev_err(sensor->dev,
					"Fail to enable gyro batching =%d\n",
					ret);
				ret = -EBUSY;
				goto exit;
			}
		} else {
			if (sensor->use_poll) {
			ktime_t ktime;

			ktime = ktime_set(0,
					sensor->gyro_poll_ms * NSEC_PER_MSEC);
			hrtimer_start(&sensor->gyro_timer, ktime,
					HRTIMER_MODE_REL);
			} else {
				icm_set_interrupt(sensor, BIT_DATA_RDY_EN, true);
				sensor->cfg.int_enabled = true;

				}
		}
		get_monotonic_boottime(&ts);
		gyro_last_enable_time_ns = timespec_to_ns(&ts);
		gyro_data_ready = false;
	} else {
		atomic_set(&sensor->gyro_en, 0);
		if (sensor->batch_gyro) {
			ret = icm_gyro_batching_disable(sensor);
			if (ret) {
				dev_err(sensor->dev,
					"Fail to enable gyro batching =%d\n",
					ret);
				ret = -EBUSY;
				goto exit;
			}
		} else {
			ret = hrtimer_try_to_cancel(&sensor->gyro_timer);
		}
		ret = icm_gyro_enable(sensor, false);
		if (ret) {
			dev_err(sensor->dev,
				"Fail to disable gyro engine ret=%d\n", ret);
			ret = -EBUSY;
			goto exit;
		}

	}

exit:
	mutex_unlock(&sensor->op_lock);

	icm_dbgmsg("done\n");
	return ret;
}
/*+++ASUS BSP: check if has enabled before.+++*/
static int icm_gyro_set_enable(struct icm_sensor *sensor, bool enable)
{
	int ret = 0;
	static int l_count = 0;

	icm_dbgmsg("enable = %d, l_count = %d\n", enable ? 1 : 0, l_count);
	if(enable==1){
		Gyro_debug_log_f = 1;
	}else if (enable == 0){
		icm_dbgmsg("Gyro x=%d, y=%d, z=%d\n", sensor->axis.rx, sensor->axis.ry, sensor->axis.rz);
	}
	if ((enable && l_count == 0) || (!enable && l_count == 1)) {
		if (g_icm_ctrl && enable) {
			icm206xx_power_ctl_cci(g_icm_ctrl, true);
		}
		ret = icm_gyro_do_enable(sensor, enable);
		if (g_icm_ctrl && !enable) {
			icm206xx_power_ctl_cci(g_icm_ctrl, false);
		}
	}

	if (enable) {
		l_count++;
	} else{
		l_count--;
	}
	return ret;
}
/*---ASUS BSP: check if has enabled before.---*/

/*
 * Set interrupt enabling bits to enable/disable specific type of interrupt.
 */
static int icm_set_interrupt(struct icm_sensor *sensor,
		const u8 mask, bool on)
{
	int ret;
	u8 data;

	if (sensor->cfg.is_asleep)
		return -EINVAL;

	ret = cci_read_byte_data(sensor,
				sensor->reg.int_enable);
	if (ret < 0) {
		dev_err(sensor->dev,
			"Fail read interrupt mode. ret=%d\n", ret);
		return ret;
	}

	if (on) {
		data = (u8)ret;
		data |= mask;
	} else {
		data = (u8)ret;
		data &= ~mask;
	}

	ret = cci_write_byte_data(sensor,
			sensor->reg.int_enable, data);
	if (ret < 0) {
		dev_err(sensor->dev,
			"Fail to set interrupt. ret=%d\n", ret);
		return ret;
	}

	return 0;
}

/*
 * Enable/disable motion detection interrupt.
 */
static int icm_set_motion_det(struct icm_sensor *sensor, bool on)
{
	int ret;

	if (on) {
		ret = cci_write_byte_data(sensor,
				sensor->reg.mot_thr, DEFAULT_MOT_THR);
		if (ret < 0)
			goto err_exit;
		ret = cci_write_byte_data(sensor,
				sensor->reg.mot_thr+1, DEFAULT_MOT_THR);
		if (ret < 0)
			goto err_exit;
		ret = cci_write_byte_data(sensor,
				sensor->reg.mot_thr+2, DEFAULT_MOT_THR);
		if (ret < 0)
			goto err_exit;

		ret = cci_write_byte_data(sensor,
				sensor->reg.mot_ctrl, DEFAULT_MOT_DET_EN);
		if (ret < 0)
			goto err_exit;

	} else {
		ret = cci_write_byte_data(sensor,
				sensor->reg.mot_ctrl, 0x00);
		if (ret < 0)
			goto err_exit;
        
    }

	ret = icm_set_interrupt(sensor, BIT_WOM_X_INT_EN|BIT_WOM_Y_INT_EN|BIT_WOM_Z_INT_EN, on);
	if (ret < 0)
		goto err_exit;

	sensor->cfg.mot_det_on = on;
	/* Use default motion detection delay 4ms */

	return 0;

err_exit:
	dev_err(sensor->dev,
			"Fail to set motion detection. ret=%d\n", ret);

	return ret;
}

/* Update sensor sample rate divider upon accel and gyro polling rate. */
static int icm_config_sample_rate(struct icm_sensor *sensor)
{
	int ret;
	u32 delay_ms;
	u8 div, saved_pwr;

	if (sensor->cfg.is_asleep)
		return -EINVAL;

	if (!atomic_read(&sensor->gyro_en)) {
		delay_ms = sensor->accel_poll_ms;
	} else if (!atomic_read(&sensor->accel_en)) {
		delay_ms = sensor->gyro_poll_ms;
	} else if (sensor->accel_poll_ms <= sensor->gyro_poll_ms) {
		delay_ms = sensor->accel_poll_ms;
	} else{
		delay_ms = sensor->gyro_poll_ms;
	}

	/* Sample_rate = internal_ODR/(1+SMPLRT_DIV) */
	if ((sensor->cfg.lpf != ICM_DLPF_256HZ_NOLPF2) &&
		(sensor->cfg.lpf != ICM_DLPF_RESERVED)) {
		if (delay_ms > DELAY_MS_MAX_DLPF)
			delay_ms = DELAY_MS_MAX_DLPF;
		if (delay_ms < DELAY_MS_MIN_DLPF)
			delay_ms = DELAY_MS_MIN_DLPF;

		div = (u8)(((ODR_DLPF_ENA * delay_ms) / MSEC_PER_SEC) - 1);
	} else {
		if (delay_ms > DELAY_MS_MAX_NODLPF)
			delay_ms = DELAY_MS_MAX_NODLPF;
		if (delay_ms < DELAY_MS_MIN_NODLPF)
			delay_ms = DELAY_MS_MIN_NODLPF;
		div = (u8)(((ODR_DLPF_DIS * delay_ms) / MSEC_PER_SEC) - 1);
	}

	icm_dbgmsg("previous div: %u, current div: %u\n", sensor->cfg.rate_div, div);
	if (sensor->cfg.rate_div == div)
		return 0;

	ret = cci_read_byte_data(sensor, sensor->reg.pwr_mgmt_1);
	if (ret < 0)
		goto err_exit;

	saved_pwr = (u8)ret;

	ret = cci_write_byte_data(sensor, sensor->reg.pwr_mgmt_1,
		(saved_pwr & ~BIT_SLEEP));
	if (ret < 0)
		goto err_exit;

	ret = cci_write_byte_data(sensor,
		sensor->reg.sample_rate_div, div);
	if (ret < 0)
		goto err_exit;

	ret = cci_write_byte_data(sensor, sensor->reg.pwr_mgmt_1,
		saved_pwr);
	if (ret < 0)
		goto err_exit;

	sensor->cfg.rate_div = div;

	return 0;
err_exit:
	dev_err(sensor->dev,
		"update sample div failed, div=%d, ret=%d\n",
		div, ret);

	return ret;
}

/*
 * Calculate sample interval according to sample rate.
 * Return sample interval in millisecond.
 */
static inline u64 icm_get_sample_interval(struct icm_sensor *sensor)
{
	u64 interval_ns;

	if ((sensor->cfg.lpf == ICM_DLPF_256HZ_NOLPF2) ||
		(sensor->cfg.lpf == ICM_DLPF_RESERVED)) {
		interval_ns = (sensor->cfg.rate_div + 1) * NSEC_PER_MSEC;
		interval_ns /= 8;
	} else {
		interval_ns = (sensor->cfg.rate_div + 1) * NSEC_PER_MSEC;
	}

	return interval_ns;
}

/*
 * icm_flush_fifo() - flush fifo and send sensor event
 * @sensor: sensor device instance
 * Return 0 on success and returns a negative error code on failure.
 *
 * This function assumes only accel and gyro data will be stored into FIFO
 * and does not check FIFO enabling bits, if other sensor data is stored into
 * FIFO, it will cause confusion.
 */
static void icm_flush_fifo(struct icm_sensor *sensor)
{
	u64 interval_ns, ts_ns, sec;
	int ret, i, ns;
	u16 *buf, cnt;
	u8 shift;

	ret = icm_read_reg(sensor, sensor->reg.fifo_count_h,
			(u8 *)&cnt, ICM_FIFO_CNT_SIZE);
	if (ret < 0) {
		dev_err(sensor->dev, "read FIFO count failed, ret=%d\n", ret);
		return;
	}

	cnt = be16_to_cpu(cnt);
	dev_dbg(sensor->dev, "Flush: FIFO count=%d\n", cnt);
	if (cnt == 0)
		return;
	if (cnt > ICM_FIFO_SIZE_BYTE || IS_ODD_NUMBER(cnt)) {
		dev_err(sensor->dev, "Invalid FIFO count number %d\n", cnt);
		return;
	}

	interval_ns = icm_get_sample_interval(sensor);
	dev_dbg(sensor->dev, "interval_ns=%llu, fifo_start_ns=%llu\n",
		interval_ns, sensor->fifo_start_ns);
	ts_ns = sensor->fifo_start_ns + interval_ns;
	icm_set_fifo_start_time(sensor);

	buf = kmalloc(cnt, GFP_KERNEL);
	if (!buf)
		return;

	ret = icm_read_reg(sensor, sensor->reg.fifo_r_w,
			(u8 *)buf, cnt);
	if (ret < 0) {
		dev_err(sensor->dev, "Read FIFO data error!\n");
		goto exit;
	}

	for (i = 0; i < (cnt >> 1); ts_ns += interval_ns) {
		if (sensor->cfg.accel_fifo_enable) {
			sensor->axis.x = be16_to_cpu(buf[i++]);
			sensor->axis.y = be16_to_cpu(buf[i++]);
			sensor->axis.z = be16_to_cpu(buf[i++]);
			sec = ts_ns;
			ns = do_div(sec, NSEC_PER_SEC);

			icm_remap_accel_data(&sensor->axis,
				sensor->pdata->place);

			shift = icm_accel_fs_shift[sensor->cfg.accel_fs];
			input_report_abs(sensor->accel_dev, ABS_X,
				(sensor->axis.x << shift));
			input_report_abs(sensor->accel_dev, ABS_Y,
				(sensor->axis.y << shift));
			input_report_abs(sensor->accel_dev, ABS_Z,
				(sensor->axis.z << shift));
#if 0
			input_event(sensor->accel_dev,
				EV_SYN, SYN_TIME_SEC,
				(int)sec);
			input_event(sensor->accel_dev,
				EV_SYN, SYN_TIME_NSEC,
				(int)ns);
#else
            input_report_abs(sensor->accel_dev, ABS_WHEEL,
                (int)sec);
            input_report_abs(sensor->accel_dev, ABS_GAS,
                (int)ns);
#endif
			input_sync(sensor->accel_dev);
		}

		if (sensor->cfg.gyro_fifo_enable) {
			sensor->axis.rx = be16_to_cpu(buf[i++]);
			sensor->axis.ry = be16_to_cpu(buf[i++]);
			sensor->axis.rz = be16_to_cpu(buf[i++]);
			sec = ts_ns;
			ns = do_div(sec, NSEC_PER_SEC);

			icm_remap_gyro_data(&sensor->axis,
				sensor->pdata->place);

			shift = icm_gyro_fs_shift[sensor->cfg.fsr];
			input_report_abs(sensor->gyro_dev, ABS_RX,
				(sensor->axis.rx >> shift));
			input_report_abs(sensor->gyro_dev, ABS_RY,
				(sensor->axis.ry >> shift));
			input_report_abs(sensor->gyro_dev, ABS_RZ,
				(sensor->axis.rz >> shift));
#if 0
			input_event(sensor->gyro_dev,
				EV_SYN, SYN_TIME_SEC,
				(int)sec);
			input_event(sensor->gyro_dev,
				EV_SYN, SYN_TIME_NSEC,
				(int)ns);
#else
            input_report_abs(sensor->gyro_dev, ABS_WHEEL,
                (int)sec);
            input_report_abs(sensor->gyro_dev, ABS_GAS,
                (int)ns);
#endif
			input_sync(sensor->gyro_dev);
		}
	}

exit:
	kfree(buf);
}

/*
 * icm_set_fifo() - Configure and enable sensor FIFO
 * @sensor: sensor device instance
 * @en_accel: buffer accel event to fifo
 * @en_gyro: buffer gyro event to fifo
 * Return 0 on success and returns a negative error code on failure.
 *
 * This function will remove all existing FIFO setting and flush FIFO data,
 * new FIFO setting will be applied after that.
 */
static int icm_set_fifo(struct icm_sensor *sensor,
					bool en_accel, bool en_gyro)
{
	struct icm_reg_map *reg = &sensor->reg;
	int ret;
	u8 en, user_ctl;

	en = FIFO_DISABLE_ALL;
	ret = cci_write_byte_data(sensor,
			reg->fifo_en, en);
	if (ret < 0)
		goto err_exit;

	icm_flush_fifo(sensor);

	/* Enable sensor output to FIFO */
	if (en_accel)
		en |= BIT_ACCEL_FIFO;

	if (en_gyro)
		en |= BIT_GYRO_FIFO;

	ret = cci_write_byte_data(sensor,
			reg->fifo_en, en);
	if (ret < 0)
		goto err_exit;

	/* Enable/Disable FIFO RW*/
	ret = cci_read_byte_data(sensor,
			reg->user_ctrl);
	if (ret < 0)
		goto err_exit;

	user_ctl = (u8)ret;
	if (en_accel | en_gyro) {
		user_ctl |= BIT_FIFO_EN;
		sensor->cfg.cfg_fifo_en = true;
	} else {
		user_ctl &= ~BIT_FIFO_EN;
		sensor->cfg.cfg_fifo_en = false;
	}

	ret = cci_write_byte_data(sensor,
			reg->user_ctrl, user_ctl);
	if (ret < 0)
		goto err_exit;

	icm_set_fifo_start_time(sensor);
	sensor->cfg.accel_fifo_enable = en_accel;
	sensor->cfg.gyro_fifo_enable = en_gyro;

	return 0;

err_exit:
	dev_err(sensor->dev, "Set fifo failed, ret=%d\n", ret);

	return ret;
}

static int icm_gyro_set_poll_delay(struct icm_sensor *sensor,
					unsigned long delay)
{
	int ret = 0;

	dev_dbg(sensor->dev,
		"icm_gyro_set_poll_delay delay=%ld\n", delay);
	if (delay < ICM_GYRO_MIN_POLL_INTERVAL_MS)
		delay = ICM_GYRO_MIN_POLL_INTERVAL_MS;
	if (delay > ICM_GYRO_MAX_POLL_INTERVAL_MS)
		delay = ICM_GYRO_MAX_POLL_INTERVAL_MS;

	mutex_lock(&sensor->op_lock);
	if (sensor->gyro_poll_ms == delay)
		goto exit;

	sensor->gyro_delay_change = true;
	sensor->gyro_poll_ms = delay;

	if (!atomic_read(&sensor->gyro_en))
		goto exit;

	if (sensor->use_poll) {
		ktime_t ktime;

		ret = hrtimer_try_to_cancel(&sensor->gyro_timer);
		ktime = ktime_set(0,
				sensor->gyro_poll_ms * NSEC_PER_MSEC);
		hrtimer_start(&sensor->gyro_timer, ktime, HRTIMER_MODE_REL);

	} else {
		ret = icm_config_sample_rate(sensor);
		if (ret < 0)
			dev_err(sensor->dev,
				"Unable to set polling delay for gyro!\n");
	}

exit:
	mutex_unlock(&sensor->op_lock);

	return ret;
}

#ifdef SENSORS_CLASSDEV
static int icm_gyro_cdev_enable(struct sensors_classdev *sensors_cdev,
			unsigned int enable)
{
	struct icm_sensor *sensor = container_of(sensors_cdev,
			struct icm_sensor, gyro_cdev);

	return icm_gyro_set_enable(sensor, enable);
}

static int icm_gyro_cdev_poll_delay(struct sensors_classdev *sensors_cdev,
			unsigned int delay_ms)
{
	struct icm_sensor *sensor = container_of(sensors_cdev,
			struct icm_sensor, gyro_cdev);

	return icm_gyro_set_poll_delay(sensor, delay_ms);
}

static int icm_gyro_cdev_flush(struct sensors_classdev *sensors_cdev)
{
	struct icm_sensor *sensor = container_of(sensors_cdev,
			struct icm_sensor, gyro_cdev);

	mutex_lock(&sensor->op_lock);
	icm_flush_fifo(sensor);
	input_event(sensor->gyro_dev,
		EV_SYN, SYN_CONFIG, sensor->flush_count++);
	input_sync(sensor->gyro_dev);
	mutex_unlock(&sensor->op_lock);

	return 0;
}

static int icm_gyro_cdev_set_latency(struct sensors_classdev *sensors_cdev,
					unsigned int max_latency)
{
	struct icm_sensor *sensor = container_of(sensors_cdev,
			struct icm_sensor, gyro_cdev);

	mutex_lock(&sensor->op_lock);

	if (max_latency <= sensor->gyro_poll_ms)
		sensor->batch_gyro = false;
	else
		sensor->batch_gyro = true;

	sensor->gyro_latency_ms = max_latency;
	mutex_unlock(&sensor->op_lock);

	return 0;
}
#endif //#ifdef SENSORS_CLASSDEV

/*
 * icm_gyro_attr_get_polling_delay() - get the sampling rate
 */
static ssize_t icm_gyro_attr_get_polling_delay(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int val;
	struct icm_sensor *sensor = dev_get_drvdata(dev);

	val = sensor ? sensor->gyro_poll_ms : 0;

	return snprintf(buf, 8, "%d\n", val);
}

/*
 * icm_gyro_attr_set_polling_delay() - set the sampling rate
 */
static ssize_t icm_gyro_attr_set_polling_delay(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct icm_sensor *sensor = dev_get_drvdata(dev);
	unsigned long interval_ms;
	int ret;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;

	ret = icm_gyro_set_poll_delay(sensor, interval_ms);

	return ret ? -EBUSY : size;
}

static ssize_t icm_gyro_attr_get_enable(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct icm_sensor *sensor = dev_get_drvdata(dev);

	return snprintf(buf, 4, "%d\n", sensor->cfg.gyro_enable);
}

/*
 * icm_gyro_attr_set_enable() -
 *    Set/get enable function is just needed by sensor HAL.
 */
static ssize_t icm_gyro_attr_set_enable(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct icm_sensor *sensor = dev_get_drvdata(dev);
	unsigned long enable;
	int ret;

	if (kstrtoul(buf, 10, &enable))
		return -EINVAL;

	if (enable)
		ret = icm_gyro_set_enable(sensor, true);
	else
		ret = icm_gyro_set_enable(sensor, false);

	return ret ? -EBUSY : count;
}

static struct device_attribute gyro_attr[] = {
	__ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		icm_gyro_attr_get_polling_delay,
		icm_gyro_attr_set_polling_delay),
	__ATTR(enable, S_IRUGO | S_IWUSR,
		icm_gyro_attr_get_enable,
		icm_gyro_attr_set_enable),
};

static int create_gyro_sysfs_interfaces(struct device *dev)
{
	int i;
	int err;

	for (i = 0; i < ARRAY_SIZE(gyro_attr); i++) {
		err = device_create_file(dev, gyro_attr + i);
		if (err)
			goto error;
	}

	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, gyro_attr + i);
	dev_err(dev, "Unable to create interface\n");

	return err;
}

static int remove_gyro_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(gyro_attr); i++)
		device_remove_file(dev, gyro_attr + i);

	return 0;
}

static int icm_accel_enable(struct icm_sensor *sensor, bool on)
{
	int ret;

	if (sensor->cfg.is_asleep)
		return -EINVAL;

	if (on) {
		ret = cci_mask_write_byte_data(sensor,
				sensor->reg.pwr_mgmt_1, BIT_SLEEP, 0);
		if (ret < 0) {
			dev_err(sensor->dev,
				"Fail to set sensor power state, ret=%d\n",
				ret);
			return ret;
		}

		ret = icm_switch_engine(sensor, true,
			BIT_PWR_ACCEL_STBY_MASK);
		if (ret)
			return ret;

		sensor->cfg.accel_enable = 1;
		sensor->cfg.enable = 1;
	} else {
		if (work_pending(&sensor->resume_work))
			cancel_work_sync(&sensor->resume_work);
		ret = icm_switch_engine(sensor, false,
			BIT_PWR_ACCEL_STBY_MASK);
		if (ret)
			return ret;
		sensor->cfg.accel_enable = 0;

		if (!sensor->cfg.gyro_enable) {
			ret = cci_mask_write_byte_data(sensor,
					sensor->reg.pwr_mgmt_1, BIT_SLEEP, BIT_SLEEP);
			if (ret < 0) {
				dev_err(sensor->dev,
					"Fail to set sensor power state for accel, ret=%d\n",
					ret);
				return ret;
			}
			sensor->cfg.enable = 0;
		}
	}

	return 0;
}

static int icm_accel_batching_enable(struct icm_sensor *sensor)
{
	int ret = 0;
	u32 latency;

	if (!sensor->batch_gyro) {
		latency = sensor->accel_latency_ms;
	} else {
		cancel_delayed_work_sync(&sensor->fifo_flush_work);
		if (sensor->accel_latency_ms < sensor->gyro_latency_ms)
			latency = sensor->accel_latency_ms;
		else
			latency = sensor->gyro_latency_ms;
	}

	ret = icm_set_fifo(sensor, true, sensor->cfg.gyro_enable);
	if (ret < 0) {
		dev_err(sensor->dev,
			"Fail to enable FIFO for accel, ret=%d\n", ret);
		return ret;
	}

	if (sensor->use_poll) {
		queue_delayed_work(sensor->data_wq,
			&sensor->fifo_flush_work,
			msecs_to_jiffies(latency));
	} else if (!sensor->cfg.int_enabled) {
		icm_set_interrupt(sensor, BIT_FIFO_OVERFLOW, true);
		sensor->cfg.int_enabled = true;
	}

	return ret;
}

static int icm_accel_batching_disable(struct icm_sensor *sensor)
{
	int ret = 0;
	u32 latency;

	ret = icm_set_fifo(sensor, false, sensor->cfg.gyro_enable);
	if (ret < 0) {
		dev_err(sensor->dev,
			"Fail to disable FIFO for accel, ret=%d\n", ret);
		return ret;
	}
	if (!sensor->use_poll) {
		if (sensor->cfg.int_enabled && !sensor->cfg.gyro_enable) {
			icm_set_interrupt(sensor,
				BIT_FIFO_OVERFLOW, false);
			sensor->cfg.int_enabled = false;
		}
	} else {
		if (!sensor->batch_gyro) {
			cancel_delayed_work_sync(&sensor->fifo_flush_work);
		} else if (sensor->accel_latency_ms <
				sensor->gyro_latency_ms) {
			cancel_delayed_work_sync(&sensor->fifo_flush_work);
			latency = sensor->gyro_latency_ms;
			queue_delayed_work(sensor->data_wq,
				&sensor->fifo_flush_work,
				msecs_to_jiffies(latency));
		}
	}
	sensor->batch_accel = false;

	return ret;
}
static int icm_accel_do_enable(struct icm_sensor *sensor, bool enable)
{
	int ret = 0;

	dev_dbg(sensor->dev,
		"icm_accel_do_enable enable=%d\n", enable);
	mutex_lock(&sensor->op_lock);
	if (enable) {
		if (!sensor->power_enabled) {
			ret = icm_power_ctl(sensor, true);
			if (ret < 0) {
				dev_err(sensor->dev,
						"Failed to power up icm\n");
				goto exit;
			}
			ret = icm_restore_context(sensor);
			if (ret < 0) {
				dev_err(sensor->dev,
						"Failed to restore context\n");
				goto exit;
			}
		}

		ret = icm_accel_enable(sensor, true);
		if (ret) {
			dev_err(sensor->dev,
				"Fail to enable accel engine ret=%d\n", ret);
			ret = -EBUSY;
			goto exit;
		}
		atomic_set(&sensor->accel_en, 1);

		ret = icm_config_sample_rate(sensor);
		if (ret < 0)
			dev_info(sensor->dev,
				"Unable to update sampling rate! ret=%d\n",
				ret);

		if (sensor->batch_accel) {
			ret = icm_accel_batching_enable(sensor);
			if (ret) {
				dev_err(sensor->dev,
					"Fail to enable accel batching =%d\n",
					ret);
				ret = -EBUSY;
				goto exit;
			}
		} else {
			if (sensor->use_poll) {
			ktime_t ktime;

			ktime = ktime_set(0,
					sensor->accel_poll_ms * NSEC_PER_MSEC);
			hrtimer_start(&sensor->accel_timer, ktime,
					HRTIMER_MODE_REL);
			} else {
				icm_set_interrupt(sensor, BIT_DATA_RDY_EN, true);
				sensor->cfg.int_enabled = true;

			}
		}
		g_skip_first_data = true;
	} else {
		atomic_set(&sensor->accel_en, 0);
		if (sensor->batch_accel) {
			ret = icm_accel_batching_disable(sensor);
			if (ret) {
				dev_err(sensor->dev,
					"Fail to disable accel batching =%d\n",
					ret);
				ret = -EBUSY;
				goto exit;
			}
		} else {
			if (sensor->use_poll) {
				ret = hrtimer_try_to_cancel(&sensor->accel_timer);
			} else {
				sensor->cfg.int_enabled = false;

			}
		}

		ret = icm_accel_enable(sensor, false);
		if (ret) {
			dev_err(sensor->dev,
				"Fail to disable accel engine ret=%d\n", ret);
			ret = -EBUSY;
			goto exit;
		}

	}

exit:
	mutex_unlock(&sensor->op_lock);

	return ret;
}

/*+++ASUS BSP: check if has enabled before.+++*/
static int icm_accel_set_enable(struct icm_sensor *sensor, bool enable)
{
	int ret = 0;
	static int l_count = 0;

	icm_dbgmsg("enable = %d, l_count = %d\n", enable ? 1 : 0, l_count);
	if(enable==1){
		G_debug_log_f = 1;
	}else if (enable == 0){
		icm_dbgmsg("G x=%d, y=%d, z=%d\n", sensor->axis.x, sensor->axis.y, sensor->axis.z);
	}
	if ((enable && l_count == 0) || (!enable && l_count == 1)) {
		if (g_icm_ctrl && enable) {
			icm206xx_power_ctl_cci(g_icm_ctrl, true);
		}
		ret = icm_accel_do_enable(sensor, enable);
		if (g_icm_ctrl && !enable) {
			icm206xx_power_ctl_cci(g_icm_ctrl, false);
		}
	}

	if (enable) {
		l_count++;
	} else{
		l_count--;
	}
	return ret;
}
/*---ASUS BSP: check if has enabled before.---*/
static int icm_accel_set_poll_delay(struct icm_sensor *sensor,
					unsigned long delay)
{
	int ret;

	dev_dbg(sensor->dev,
		"icm_accel_set_poll_delay delay_ms=%ld\n", delay);
	if (delay < ICM_ACCEL_MIN_POLL_INTERVAL_MS)
		delay = ICM_ACCEL_MIN_POLL_INTERVAL_MS;
	if (delay > ICM_ACCEL_MAX_POLL_INTERVAL_MS)
		delay = ICM_ACCEL_MAX_POLL_INTERVAL_MS;

	mutex_lock(&sensor->op_lock);
	if (sensor->accel_poll_ms == delay)
		goto exit;

	sensor->accel_delay_change = true;
	sensor->accel_poll_ms = delay;

	if (!atomic_read(&sensor->accel_en))
		goto exit;


	if (sensor->use_poll) {
		ktime_t ktime;

		ret = hrtimer_try_to_cancel(&sensor->accel_timer);
		ktime = ktime_set(0,
				sensor->accel_poll_ms * NSEC_PER_MSEC);
		hrtimer_start(&sensor->accel_timer, ktime, HRTIMER_MODE_REL);
	} else {
		ret = icm_config_sample_rate(sensor);
		if (ret < 0)
			dev_err(sensor->dev,
				"Unable to set polling delay for accel!\n");
	}

exit:
	mutex_unlock(&sensor->op_lock);

	return ret;
}

#ifdef SENSORS_CLASSDEV
static int icm_accel_cdev_enable(struct sensors_classdev *sensors_cdev,
			unsigned int enable)
{
	struct icm_sensor *sensor = container_of(sensors_cdev,
			struct icm_sensor, accel_cdev);
	int err;

	err = icm_accel_set_enable(sensor, enable);

	return err;
}

static int icm_accel_cdev_poll_delay(struct sensors_classdev *sensors_cdev,
			unsigned int delay_ms)
{
	struct icm_sensor *sensor = container_of(sensors_cdev,
			struct icm_sensor, accel_cdev);

	return icm_accel_set_poll_delay(sensor, delay_ms);
}

static int icm_accel_cdev_flush(struct sensors_classdev *sensors_cdev)
{
	struct icm_sensor *sensor = container_of(sensors_cdev,
			struct icm_sensor, accel_cdev);

	mutex_lock(&sensor->op_lock);
	icm_flush_fifo(sensor);
	mutex_unlock(&sensor->op_lock);
	input_event(sensor->accel_dev,
		EV_SYN, SYN_CONFIG, sensor->flush_count++);
	input_sync(sensor->accel_dev);

	return 0;
}

static int icm_accel_cdev_set_latency(struct sensors_classdev *sensors_cdev,
					unsigned int max_latency)
{
	struct icm_sensor *sensor = container_of(sensors_cdev,
			struct icm_sensor, accel_cdev);

	mutex_lock(&sensor->op_lock);
	if (max_latency <= sensor->accel_poll_ms)
		sensor->batch_accel = false;
	else
		sensor->batch_accel = true;

	sensor->accel_latency_ms = max_latency;
	mutex_unlock(&sensor->op_lock);

	return 0;
}

static int icm_accel_cdev_enable_wakeup(
			struct sensors_classdev *sensors_cdev,
			unsigned int enable)
{
	struct icm_sensor *sensor = container_of(sensors_cdev,
			struct icm_sensor, accel_cdev);

	if (sensor->use_poll)
		return -ENODEV;

	sensor->motion_det_en = enable;

	return 0;
}

static int icm_accel_calibration(struct sensors_classdev *sensors_cdev,
		int axis, int apply_now)
{
	struct icm_sensor *sensor = container_of(sensors_cdev,
			struct icm_sensor, accel_cdev);
	int ret;
	bool pre_enable;
	int arry[3] = { 0 };
	int i, delay_ms;

	if (axis < AXIS_X && axis > AXIS_XYZ) {
		dev_err(sensor->dev,
				"accel calibration cmd error\n");
		ret = -EINVAL;
		goto exit;
	}

	pre_enable = sensor->cfg.accel_enable;
	if (pre_enable)
		icm_accel_set_enable(sensor, false);

	if (!sensor->power_enabled) {
		ret = icm_power_ctl(sensor, true);
		if (ret < 0) {
			dev_err(sensor->dev,
					"Failed to set power up icm");
			goto exit;
		}

		ret = icm_restore_context(sensor);
		if (ret < 0) {
			dev_err(sensor->dev,
					"Failed to restore context");
			goto exit;
		}
	}

	delay_ms = sensor->accel_poll_ms;
	sensor->accel_poll_ms = ICM_ACCEL_MIN_POLL_INTERVAL_MS;
	ret = icm_config_sample_rate(sensor);
	if (ret < 0)
		dev_info(sensor->dev,
				"Unable to update sampling rate! ret=%d\n",
				ret);

	ret = icm_accel_enable(sensor, true);
	if (ret) {
		dev_err(sensor->dev,
				"Fail to enable accel engine ret=%d\n", ret);
		ret = -EBUSY;
		goto exit;
	}

	for (i = 0; i < ICM_ACC_CAL_COUNT; i++) {
		msleep(ICM_ACC_CAL_DELAY);
		icm_read_accel_data(sensor, &sensor->axis);
		if (i < CAL_SKIP_COUNT)
			continue;
		icm_remap_accel_data(&sensor->axis, sensor->pdata->place);
		arry[0] += sensor->axis.x;
		arry[1] += sensor->axis.y;
		arry[2] += sensor->axis.z;
	}

	arry[0] = arry[0] / (ICM_ACC_CAL_NUM);
	arry[1] = arry[1] / (ICM_ACC_CAL_NUM);
	arry[2] = arry[2] / (ICM_ACC_CAL_NUM);

	switch (axis) {
	case AXIS_X:
		arry[1] = 0;
		arry[2] = 0;
		break;
	case AXIS_Y:
		arry[0] = 0;
		arry[2] = 0;
		break;
	case AXIS_Z:
		arry[0] = 0;
		arry[1] = 0;
		arry[2] -= RAW_TO_1G;
		break;
	case AXIS_XYZ:
		arry[2] -= RAW_TO_1G;
		break;
	default:
		dev_err(sensor->dev,
				"calibrate icm accel CMD error\n");
		ret = -EINVAL;
		goto exit;
	}

	if (apply_now) {
		sensor->acc_cal_params[0] = arry[0];
		sensor->acc_cal_params[1] = arry[1];
		sensor->acc_cal_params[2] = arry[2];
		sensor->acc_use_cal = true;
	}
	snprintf(sensor->acc_cal_buf, sizeof(sensor->acc_cal_buf),
			"%d,%d,%d", arry[0], arry[1], arry[2]);
	sensors_cdev->params = sensor->acc_cal_buf;

	sensor->accel_poll_ms = delay_ms;
	ret = icm_config_sample_rate(sensor);
	if (ret < 0)
		dev_info(sensor->dev,
				"Unable to update sampling rate! ret=%d\n",
				ret);

	ret = icm_accel_enable(sensor, false);
	if (ret) {
		dev_err(sensor->dev,
				"Fail to disable accel engine ret=%d\n", ret);
		ret = -EBUSY;
		goto exit;
	}
	if (pre_enable)
		icm_accel_set_enable(sensor, true);

exit:

	return ret;
}

static int icm_write_accel_cal_params(struct sensors_classdev *sensors_cdev,
		struct cal_result_t *cal_result)
{
	struct icm_sensor *sensor = container_of(sensors_cdev,
			struct icm_sensor, accel_cdev);

	mutex_lock(&sensor->op_lock);

	sensor->acc_cal_params[0] = cal_result->offset_x;
	sensor->acc_cal_params[1] = cal_result->offset_y;
	sensor->acc_cal_params[2] = cal_result->offset_z;
	snprintf(sensor->acc_cal_buf, sizeof(sensor->acc_cal_buf),
			"%d,%d,%d", sensor->acc_cal_params[0],
			sensor->acc_cal_params[1], sensor->acc_cal_params[2]);
	sensors_cdev->params = sensor->acc_cal_buf;
	sensor->acc_use_cal = true;

	mutex_unlock(&sensor->op_lock);

	return 0;
}
#endif //#ifdef SENSORS_CLASSDEV

static int icm_acc_data_process(struct icm_sensor *sensor)
{
	int ret = 0;
	ret = icm_read_accel_data(sensor, &sensor->axis);
	if (!ret) {
		icm_remap_accel_data(&sensor->axis, sensor->pdata->place);
		if (sensor->acc_use_cal) {
			sensor->axis.x -= sensor->acc_cal_params[0];
			sensor->axis.y -= sensor->acc_cal_params[1];
			sensor->axis.z -= sensor->acc_cal_params[2];
		}
		sensor->axis.x = sensor->axis.x * ACCELDATAUNIT / RAW_TO_1G;
		sensor->axis.y = sensor->axis.y * ACCELDATAUNIT / RAW_TO_1G;
		sensor->axis.z = sensor->axis.z * ACCELDATAUNIT / RAW_TO_1G;
	}
	return ret;
}

/*
 * icm_accel_attr_get_polling_delay() - get the sampling rate
 */
static ssize_t icm_accel_attr_get_polling_delay(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int val;
	struct icm_sensor *sensor = dev_get_drvdata(dev);

	val = sensor ? sensor->accel_poll_ms : 0;

	return snprintf(buf, 8, "%d\n", val);
}

/*
 * icm_accel_attr_set_polling_delay() - set the sampling rate
 */
static ssize_t icm_accel_attr_set_polling_delay(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct icm_sensor *sensor = dev_get_drvdata(dev);
	unsigned long interval_ms;
	int ret;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;

	ret = icm_accel_set_poll_delay(sensor, interval_ms);

	return ret ? -EBUSY : size;
}

static ssize_t icm_accel_attr_get_enable(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct icm_sensor *sensor = dev_get_drvdata(dev);
	
	return snprintf(buf, 4, "%d\n", sensor->cfg.accel_enable);
}

/*
 * icm_accel_attr_set_enable() -
 *    Set/get enable function is just needed by sensor HAL.
 */

static ssize_t icm_accel_attr_set_enable(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct icm_sensor *sensor = dev_get_drvdata(dev);
	unsigned long enable;
	int ret;

	if (kstrtoul(buf, 10, &enable))
		return -EINVAL;

	if (enable)
		ret = icm_accel_set_enable(sensor, true);
	else
		ret = icm_accel_set_enable(sensor, false);

	return ret ? -EBUSY : count;
}

#ifdef DEBUG_NODE
u8 icm_address;
u8 icm_data;

static ssize_t icm_accel_attr_get_reg_addr(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, 8, "%d\n", icm_address);
}

static ssize_t icm_accel_attr_set_reg_addr(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long addr;

	if (kstrtoul(buf, 10, &addr))
		return -EINVAL;
	if ((addr < 0) || (addr > 255))
		return -EINVAL;

	icm_address = addr;
	dev_info(dev, "icm_address =%d\n", icm_address);

	return size;
}

static ssize_t icm_accel_attr_get_data(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct icm_sensor *sensor = dev_get_drvdata(dev);
	int ret;

	ret = cci_read_byte_data(sensor, icm_address);
	dev_info(dev, "read addr(0x%x)=0x%x\n", icm_address, ret);
	if (ret >= 0 && ret <= 255)
		icm_data = ret;

	return snprintf(buf, 8, "0x%x\n", ret);
}

static ssize_t icm_accel_attr_set_data(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long reg_data;

	if (kstrtoul(buf, 10, &reg_data))
		return -EINVAL;
	if ((reg_data < 0) || (reg_data > 255))
		return -EINVAL;

	icm_data = reg_data;
	dev_info(dev, "set icm_data =0x%x\n", icm_data);

	return size;
}
static ssize_t icm_accel_attr_reg_write(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct icm_sensor *sensor = dev_get_drvdata(dev);
	int ret;

	ret = cci_write_byte_data(sensor,
		icm_address, icm_data);
	dev_info(dev, "write addr(0x%x)<-0x%x ret=%d\n",
		icm_address, icm_data, ret);

	return size;
}

#endif

static struct device_attribute accel_attr[] = {
	__ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		icm_accel_attr_get_polling_delay,
		icm_accel_attr_set_polling_delay),
	__ATTR(enable, S_IRUGO | S_IWUSR,
		icm_accel_attr_get_enable,
		icm_accel_attr_set_enable),
#ifdef DEBUG_NODE
	__ATTR(addr, S_IRUSR | S_IWUSR,
		icm_accel_attr_get_reg_addr,
		icm_accel_attr_set_reg_addr),
	__ATTR(reg, S_IRUSR | S_IWUSR,
		icm_accel_attr_get_data,
		icm_accel_attr_set_data),
	__ATTR(write, S_IWUSR,
		NULL,
		icm_accel_attr_reg_write),
#endif
};

static int create_accel_sysfs_interfaces(struct device *dev)
{
	int i;
	int err;

	for (i = 0; i < ARRAY_SIZE(accel_attr); i++) {
		err = device_create_file(dev, accel_attr + i);
		if (err)
			goto error;
	}

	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, accel_attr + i);
	dev_err(dev, "Unable to create interface\n");

	return err;
}

static int remove_accel_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(accel_attr); i++)
		device_remove_file(dev, accel_attr + i);

	return 0;
}


static void setup_icm_reg(struct icm_reg_map *reg)
{
	reg->sample_rate_div	= REG_SAMPLE_RATE_DIV;
	reg->lpf		= REG_CONFIG;
	reg->fifo_en		= REG_FIFO_EN;
	reg->gyro_config	= REG_GYRO_CONFIG;
	reg->accel_config	= REG_ACCEL_CONFIG;
	reg->mot_thr		= REG_ACCEL_WOM_X_THR;
	reg->mot_ctrl		= REG_DETECT_CTRL;
	reg->fifo_count_h	= REG_FIFO_COUNT_H;
	reg->fifo_r_w		= REG_FIFO_R_W;
	reg->raw_gyro		= REG_RAW_GYRO;
	reg->raw_accel		= REG_RAW_ACCEL;
	reg->temperature	= REG_TEMPERATURE;
	reg->int_pin_cfg	= REG_INT_PIN_CFG;
	reg->int_enable		= REG_INT_ENABLE;
	reg->int_status		= REG_INT_STATUS;
	reg->user_ctrl		= REG_USER_CTRL;
	reg->pwr_mgmt_1		= REG_PWR_MGMT_1;
	reg->pwr_mgmt_2		= REG_PWR_MGMT_2;
};

/*
 * icm_check_chip_type() - check and setup chip type.
 */
static int icm_check_chip_type(struct icm_sensor *sensor)
{
	struct icm_reg_map *reg;
	s32 ret;

	reg = &sensor->reg;
	setup_icm_reg(reg);
	
#ifndef TO_BE_REFINED	// those icm_set_power_mode() causes kernel bringup halt.
	/* turn off and turn on power to ensure gyro engine is on */

	ret = icm_set_power_mode(sensor, false);
	if (ret)
		return ret;
	ret = icm_set_power_mode(sensor, true);
	if (ret)
		return ret;
#endif

    ret = cci_read_byte_data(sensor,
            REG_WHOAMI);
    if (ret < 0)
        return ret;
    
    sensor->deviceid = ret;
    pr_info(INVN_TAG", WHOAMI=0x%x", ret);

	if (sensor->deviceid == ICM20602_ID) {
		sensor->chip_type = INV_ICM20602;
	} else if (sensor->deviceid == ICM20626_ID) {
		sensor->chip_type = INV_ICM20626;
	} else if (sensor->deviceid == ICM20690_ID) {
		sensor->chip_type = INV_ICM20690;
	} else {
		dev_err(sensor->dev,
			"Invalid chip ID %d\n", sensor->deviceid);
		return -ENODEV;
	}

	return 0;
}

/*
 *  icm_init_config() - Initialize hardware, disable FIFO.
 *  @indio_dev:	Device driver instance.
 *  Initial configuration:
 *  FSR: +/- 2000DPS
 *  DLPF: 188Hz
 *  FIFO rate: 1000Hz
 *  AFS: 2G
 */
static int icm_init_config(struct icm_sensor *sensor)
{
	struct icm_reg_map *reg;
	s32 ret;
	u8 data;

	icm_dbgmsg("++\n");
	if (sensor->cfg.is_asleep)
		return -EINVAL;

	reg = &sensor->reg;
	icm_reset_chip(sensor);
	memset(&sensor->cfg, 0, sizeof(struct icm_chip_config));

	/* Wake up from sleep */
	ret = cci_write_byte_data(sensor, reg->pwr_mgmt_1,
		BIT_WAKEUP_AFTER_RESET);
	if (ret < 0)
		return ret;

	/* Gyro full scale range configure */
    if (sensor->chip_type == INV_ICM20690) {
        ret = cci_write_byte_data(sensor, reg->gyro_config,
            ICM_FSR_2000DPS << GYRO_CONFIG_FSR_SHIFT_ICM2069X);
    } else {
        ret = cci_write_byte_data(sensor, reg->gyro_config,
            ICM_FSR_2000DPS << GYRO_CONFIG_FSR_SHIFT_ICM2060X);
    }
	if (ret < 0)
		return ret;
	sensor->cfg.fsr = ICM_FSR_2000DPS;

	ret = cci_write_byte_data(sensor, reg->lpf, ICM_DLPF_188HZ);
	if (ret < 0)
		return ret;
	sensor->cfg.lpf = ICM_DLPF_188HZ;

	data = (u8)(ODR_DLPF_ENA / INIT_FIFO_RATE - 1);
	ret = cci_write_byte_data(sensor, reg->sample_rate_div, data);
	if (ret < 0)
		return ret;
	sensor->cfg.rate_div = data;

	ret = cci_write_byte_data(sensor, reg->accel_config,
		(ACCEL_FS_02G << ACCL_CONFIG_FSR_SHIFT));
	if (ret < 0)
		return ret;
	sensor->cfg.accel_fs = ACCEL_FS_02G;

	if ((sensor->pdata->int_flags & IRQF_TRIGGER_FALLING) ||
		(sensor->pdata->int_flags & IRQF_TRIGGER_LOW))
		data = BIT_INT_CFG_DEFAULT | BIT_INT_ACTIVE_LOW;
	else
		data = BIT_INT_CFG_DEFAULT;
	ret = cci_write_byte_data(sensor, reg->int_pin_cfg, data);
	if (ret < 0)
		return ret;
	sensor->cfg.int_pin_cfg = data;

	/* Put sensor into sleep mode */
	ret = cci_read_byte_data(sensor,
		sensor->reg.pwr_mgmt_1);
	if (ret < 0)
		return ret;

	data = (u8)ret;
	data |=  BIT_SLEEP;
	ret = cci_write_byte_data(sensor,
		sensor->reg.pwr_mgmt_1, data);
	if (ret < 0)
		return ret;

	sensor->cfg.gyro_enable = 0;
	sensor->cfg.gyro_fifo_enable = 0;
	sensor->cfg.accel_enable = 0;
	sensor->cfg.accel_fifo_enable = 0;

	icm_dbgmsg("--\n");
	return 0;
}

static int icm_pinctrl_init(struct icm_sensor *sensor)
{
	sensor->pinctrl = devm_pinctrl_get(sensor->dev);
	if (IS_ERR_OR_NULL(sensor->pinctrl)) {
		dev_err(sensor->dev, "Failed to get pinctrl\n");
		return PTR_ERR(sensor->pinctrl);
	}

	sensor->pin_default =
		pinctrl_lookup_state(sensor->pinctrl, ICM_PINCTRL_DEFAULT);
	if (IS_ERR_OR_NULL(sensor->pin_default))
		dev_err(sensor->dev, "Failed to look up default state\n");

	sensor->pin_sleep =
		pinctrl_lookup_state(sensor->pinctrl, ICM_PINCTRL_SUSPEND);
	if (IS_ERR_OR_NULL(sensor->pin_sleep))
		dev_err(sensor->dev, "Failed to look up sleep state\n");

	return 0;
}

static void icm_pinctrl_state(struct icm_sensor *sensor,
			bool active)
{
	int ret;

	dev_dbg(sensor->dev, "icm_pinctrl_state en=%d\n", active);

	if (active) {
		if (!IS_ERR_OR_NULL(sensor->pin_default)) {
			ret = pinctrl_select_state(sensor->pinctrl,
				sensor->pin_default);
			if (ret)
				dev_err(sensor->dev,
					"Error pinctrl_select_state(%s) err:%d\n",
					ICM_PINCTRL_DEFAULT, ret);
		}
	} else {
		if (!IS_ERR_OR_NULL(sensor->pin_sleep)) {
			ret = pinctrl_select_state(sensor->pinctrl,
				sensor->pin_sleep);
			if (ret)
				dev_err(sensor->dev,
					"Error pinctrl_select_state(%s) err:%d\n",
					ICM_PINCTRL_SUSPEND, ret);
		}
	}
}

#ifdef CONFIG_OF
static int icm_dt_get_place(struct device *dev,
			struct icm_platform_data *pdata)
{
	const char *place_name;
	int rc;
	int i;

	rc = of_property_read_string(dev->of_node, "invn,place", &place_name);
	if (rc) {
		dev_err(dev, "Cannot get place configuration!\n");
		return -EINVAL;
	}

	icm_dbgmsg("place_name=[%s]\n", place_name);
	
	for (i = 0; i < ICM_AXIS_REMAP_TAB_SZ; i++) {
		if (!strcmp(place_name, icm_place_name2num[i].name)) {
			pdata->place = icm_place_name2num[i].place;
			break;
		}
	}
	if (i >= ICM_AXIS_REMAP_TAB_SZ) {
		dev_warn(dev, "Invalid place parameter, use default value 0\n");
		pdata->place = 0;
	}

	return 0;
}

static int icm_parse_dt(struct device *dev,
			struct icm_platform_data *pdata)
{
	int rc;

	rc = icm_dt_get_place(dev, pdata);
	if (rc)
		return rc;

	pdata->gpio_int = of_get_named_gpio_flags(dev->of_node,
				"invn,gpio-int", 0, &pdata->int_flags);

	pdata->gpio_en = of_get_named_gpio_flags(dev->of_node,
				"invn,gpio-en", 0, NULL);

	pdata->use_int = of_property_read_bool(dev->of_node,
				"invn,use-interrupt");

	return 0;
}
#else
static int icm_parse_dt(struct device *dev,
			struct icm_platform_data *pdata)
{
	return -EINVAL;
}
#endif

/*+++ASUS BSP proc asusIcm206xxPolling Interface+++*/
static int asusIcm206xxPolling_proc_read(struct seq_file *buf, void *v)
{
	int result = 0;
	struct icm_sensor *sensor = NULL;
	if (!g_icm_ctrl) {
		icm_errmsg("null icm ctrl!");
		result = -1;
	} else{
		sensor = dev_get_drvdata(&g_icm_ctrl->pdev->dev);
		if (sensor->use_poll) {
			result = 1;
		} else{
			result = 0;
		}
	}
	seq_printf(buf, "%d\n", result);
	return 0;
}
static int asusIcm206xxPolling_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, asusIcm206xxPolling_proc_read, NULL);
}

static ssize_t asusIcm206xxPolling_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val;
	char messages[256];
	struct icm_sensor *sensor = NULL;
	if (!g_icm_ctrl) {
		icm_errmsg("null icm ctrl!");
		return len;
	}
	sensor = dev_get_drvdata(&g_icm_ctrl->pdev->dev);

	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);
	if (val == 0) {
		sensor->use_poll = false;
	} else{
		sensor->use_poll = true;
	}
	icm_dbgmsg("%d\n", val);
	return len;
}
static void create_asusIcm206xxPolling_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open =  asusIcm206xxPolling_proc_open,
		.write = asusIcm206xxPolling_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/asusIcm206xxPolling", 0664, NULL, &proc_fops);

	if (!proc_file) {
		icm_errmsg("failed!\n");
	}
	return;
}
/*---ASUS BSP proc asusIcm206xxPolling Interface---*/
static ssize_t accel2_poll_delay_show(struct class *c, struct class_attribute *attr,
			char *ubuf)
{
	struct icm_sensor *sensor = NULL;
	icm_dbgmsg("%s\n", ubuf);
	if (!g_icm_ctrl) {
		icm_errmsg("null icm ctrl!");
		return snprintf(ubuf, PAGE_SIZE, "%d\n", 0);
	}
	sensor = dev_get_drvdata(&g_icm_ctrl->pdev->dev);
	return snprintf(ubuf, PAGE_SIZE, "%u\n", sensor->accel_poll_ms);
}

static ssize_t accel2_poll_delay_store(struct class *c, struct class_attribute *attr,
			const char *ubuf, size_t count)
{
	unsigned long interval_ms;
	struct icm_sensor *sensor = NULL;
	icm_dbgmsg("%s ms\n", ubuf);
	G_debug_log_f = 1;
	if (!g_icm_ctrl) {
		icm_errmsg("null icm ctrl!");
		return count;
	}
	sensor = dev_get_drvdata(&g_icm_ctrl->pdev->dev);
	if (!kstrtoul(ubuf, 10, &interval_ms)) {
		icm_accel_set_poll_delay(sensor, interval_ms);
	}
	return count;
}
static CLASS_ATTR_RW(accel2_poll_delay);

static ssize_t gyro2_poll_delay_show(struct class *c, struct class_attribute *attr,
			char *ubuf)
{
	struct icm_sensor *sensor = NULL;
	icm_dbgmsg("%s\n", ubuf);
	if (!g_icm_ctrl) {
		icm_errmsg("null icm ctrl!");
		return snprintf(ubuf, PAGE_SIZE, "%d\n", 0);
	}
	sensor = dev_get_drvdata(&g_icm_ctrl->pdev->dev);
	return snprintf(ubuf, PAGE_SIZE, "%u\n", sensor->gyro_poll_ms);
}

static ssize_t gyro2_poll_delay_store(struct class *c, struct class_attribute *attr,
			const char *ubuf, size_t count)
{
	unsigned long interval_ms;
	struct icm_sensor *sensor = NULL;
	icm_dbgmsg("%s ms\n", ubuf);
	Gyro_debug_log_f = 1;
	
	if (!g_icm_ctrl) {
		icm_errmsg("null icm ctrl!");
		return count;
	}
	sensor = dev_get_drvdata(&g_icm_ctrl->pdev->dev);
	if (!kstrtoul(ubuf, 10, &interval_ms)) {
		icm_gyro_set_poll_delay(sensor, interval_ms);
	}
	return count;
}
static CLASS_ATTR_RW(gyro2_poll_delay);

enum {
	ACCEL2_POLL_DELAY = 0,
	GYRO2_POLL_DELAY = 1,
};
static struct attribute *icm206xx_class_attrs[] = {
	[ACCEL2_POLL_DELAY]	= &class_attr_accel2_poll_delay.attr,
	[GYRO2_POLL_DELAY]	= &class_attr_gyro2_poll_delay.attr,
	NULL,
};
ATTRIBUTE_GROUPS(icm206xx_class);
struct class icm206xx_class = {
	.name = "icm206xx",
	.owner = THIS_MODULE,
	.class_groups = icm206xx_class_groups
};
static int icm206xx_classRegister(void)
{
	class_register(&icm206xx_class);
	return 0;
}
/*+++ASUS BSP proc asusIcm206xxDebug Interface+++*/
static int asusIcm206xxDebug_proc_read(struct seq_file *buf, void *v)
{
	int result = 0;
	if (g_debugMode) {
		result = 1;
	} else{
		result = 0;
	}
	seq_printf(buf, "%d\n", result);
	return 0;
}
static int asusIcm206xxDebug_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, asusIcm206xxDebug_proc_read, NULL);
}

static ssize_t asusIcm206xxDebug_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val;
	char messages[256];
	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);
	if (val == 0) {
		g_debugMode = false;
	} else{
		g_debugMode = true;
	}
	icm_dbgmsg("%d\n", val);
	return len;
}
static void create_asusIcm206xxDebug_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open =  asusIcm206xxDebug_proc_open,
		.write = asusIcm206xxDebug_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/asusIcm206xxDebug", 0664, NULL, &proc_fops);

	if (!proc_file) {
		icm_errmsg("failed!\n");
	}
	return;
}
/*---ASUS BSP proc asusIcm206xxDebug Interface---*/
static int icm206xx_accel_miscOpen(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct icm_sensor *sensor = NULL;
	if (!g_icm_ctrl) {
		icm_errmsg("null icm ctrl!");
		return -1;
	}
	sensor = dev_get_drvdata(&g_icm_ctrl->pdev->dev);
	ret = icm_accel_set_enable(sensor, true);
	icm_dbgmsg("ret = %d\n", ret);
	if (ret < 0) {
		icm_accel_set_enable(sensor, false);
	}
	return ret;
}

static int icm206xx_accel_miscRelease(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct icm_sensor *sensor = NULL;
	if (!g_icm_ctrl) {
		icm_errmsg("null icm ctrl!");
		return -1;
	}
	sensor = dev_get_drvdata(&g_icm_ctrl->pdev->dev);
	ret = icm_accel_set_enable(sensor, false);
	icm_dbgmsg("ret = %d\n", ret);
	return ret;
}
static long icm206xx_accel_miscIoctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int dataI[ICM206XX_ACCEL_DATA_SIZE];
	u8 shift;
	struct icm_sensor *sensor = NULL;
	if (!g_icm_ctrl) {
		icm_errmsg("null icm ctrl!");
		ret = -1;
		goto end;
	}
	sensor = dev_get_drvdata(&g_icm_ctrl->pdev->dev);
	switch (cmd) {
		case ICM206XX_ACCEL_IOCTL_DATA_READ:
			icm_acc_data_process(sensor);
			shift = icm_accel_fs_shift[sensor->cfg.accel_fs];
			dataI[0] = sensor->axis.x << shift;
			dataI[1] = sensor->axis.y << shift;
			dataI[2] = sensor->axis.z << shift;
			icm_dbgmsg("cmd = DATA_READ, data[0] = %d, data[1] = %d, data[2] = %d\n", dataI[0], dataI[1], dataI[2]);
			ret = copy_to_user((int __user*)arg, &dataI, sizeof(dataI));
			break;
		default:
			ret = -1;
			icm_errmsg("default\n", __func__);
	}
end:
	return ret;
}
static struct file_operations icm206xx_accel_fops = {
  .owner = THIS_MODULE,
  .open = icm206xx_accel_miscOpen,
  .release = icm206xx_accel_miscRelease,
  .unlocked_ioctl = icm206xx_accel_miscIoctl,
  .compat_ioctl = icm206xx_accel_miscIoctl
};
struct miscdevice icm206xx_accel_misc = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = "asus2ndAccelSensor",
  .fops = &icm206xx_accel_fops
};
static int icm206xx_gyro_miscOpen(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct icm_sensor *sensor = NULL;
	if (!g_icm_ctrl) {
		icm_errmsg("null icm ctrl!");
		return -1;
	}
	sensor = dev_get_drvdata(&g_icm_ctrl->pdev->dev);
	ret = icm_gyro_set_enable(sensor, true);
	icm_dbgmsg("ret = %d\n", ret);
	if (ret < 0) {
		icm_gyro_set_enable(sensor, false);
	}
	return ret;
}

static int icm206xx_gyro_miscRelease(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct icm_sensor *sensor = NULL;
	if (!g_icm_ctrl) {
		icm_errmsg("null icm ctrl!");
		return -1;
	}
	sensor = dev_get_drvdata(&g_icm_ctrl->pdev->dev);
	ret = icm_gyro_set_enable(sensor, false);
	icm_dbgmsg("ret = %d\n", ret);
	return ret;
}
static long icm206xx_gyro_miscIoctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int dataI[ICM206XX_GYRO_DATA_SIZE];
	u8 shift;
	struct icm_sensor *sensor = NULL;
	if (!g_icm_ctrl) {
		icm_errmsg("null icm ctrl!");
		ret = -1;
		goto end;
	}
	sensor = dev_get_drvdata(&g_icm_ctrl->pdev->dev);
	switch (cmd) {
		case ICM206XX_GYRO_IOCTL_DATA_READ:
			icm_read_gyro_data(sensor, &sensor->axis);
			icm_remap_gyro_data(&sensor->axis,
				sensor->pdata->place);
			shift = icm_gyro_fs_shift[sensor->cfg.fsr];
			dataI[0] = sensor->axis.rx >> shift;
			dataI[1] = sensor->axis.ry >> shift;
			dataI[2] = sensor->axis.rz >> shift;
			icm_dbgmsg("cmd = DATA_READ, data[0] = %d, data[1] = %d, data[2] = %d\n", dataI[0], dataI[1], dataI[2]);
			ret = copy_to_user((int __user*)arg, &dataI, sizeof(dataI));
			break;
		default:
			ret = -1;
			icm_errmsg("default\n", __func__);
	}
end:
	return ret;
}
static struct file_operations icm206xx_gyro_fops = {
  .owner = THIS_MODULE,
  .open = icm206xx_gyro_miscOpen,
  .release = icm206xx_gyro_miscRelease,
  .unlocked_ioctl = icm206xx_gyro_miscIoctl,
  .compat_ioctl = icm206xx_gyro_miscIoctl
};
struct miscdevice icm206xx_gyro_misc = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = "asus2ndGyroSensor",
  .fops = &icm206xx_gyro_fops
};
static int icm206xx_miscRegister(void)
{
	int rtn = 0;
	rtn = misc_register(&icm206xx_accel_misc);
	if (rtn < 0) {
		icm_errmsg("Unable to register misc deive\n");
		misc_deregister(&icm206xx_accel_misc);
	}
	rtn = misc_register(&icm206xx_gyro_misc);
	if (rtn < 0) {
		icm_errmsg("Unable to register misc deive\n");
		misc_deregister(&icm206xx_gyro_misc);
	}
	return rtn;
}
void print_sensor_status(void)
{
	char sensorInfo[256];
	struct icm_sensor *sensor = NULL;
	if (!g_icm_ctrl) {
		icm_errmsg("null icm ctrl!");
		return;
	}
	sensor = dev_get_drvdata(&g_icm_ctrl->pdev->dev);

	snprintf(sensorInfo, sizeof(sensorInfo), "irq_counter = %llu, accel_enable = %u, gyro_enable = %u, sample_rate = %u ms",
		g_irq_counter,
		sensor->cfg.accel_enable,
		sensor->cfg.gyro_enable,
		sensor->cfg.rate_div + 1);
	icm_dbgmsg("%s\n", sensorInfo);
}
static struct delayed_work work_report;
static unsigned long g_next_report_time_s = 2;
void report_wq(struct work_struct *work)
{
	print_sensor_status();

	/*ASUS_BSP: calculate report time, the report interval(s) will be: 0, 4, 8, 16, 32, 64, 128, 256, 512, 600*/
	if (g_next_report_time_s != 600) {
		g_next_report_time_s *= 2;
	}
	if (g_next_report_time_s > 600) {
		g_next_report_time_s = 600;
	}
	schedule_delayed_work(&work_report, HZ * g_next_report_time_s);
}
/*
 * icm_probe() - device detection callback
 * @client: i2c client of found device
 * @id: id match information
 *
 * The I2C layer calls us when it believes a sensor is present at this
 * address. Probe to see if this is correct and to validate the device.
 *
 * If present install the relevant sysfs interfaces and input device.
 */
static int icm_platform_probe(struct platform_device *pdev)
{
	struct icm_sensor *sensor;
	struct icm_platform_data *pdata;
	int ret;

	icm_dbgmsg("+\n");
	if (!cam_cci_get_subdev(1)) {
		icm_errmsg("cam_cci_get_subdev() not ready\n");
		return -EPROBE_DEFER;	//defer the probe call.
	}

	sensor = devm_kzalloc(&pdev->dev, sizeof(struct icm_sensor),
			GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->pdev = pdev;
	sensor->dev = &pdev->dev;

	dev_set_drvdata(&pdev->dev, sensor);
	INIT_DELAYED_WORK(&work_report, report_wq);
	INIT_DELAYED_WORK(&data_retry_work, data_retry_wq);

	wakeup_source_init(&sensor->icm206xx_wakeup_source, "icm206xx_wakeup_source");

	if (pdev->dev.of_node) {
		pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct icm_platform_data), GFP_KERNEL);
		if (!pdata) {
			ret = -ENOMEM;
			goto err_free_devmem;
		}

		ret = icm_parse_dt(&pdev->dev, pdata);
		icm_dbgmsg("gpio_int=%d, int_flags=%d, gpio_en=%d, use_int=%d\n", pdata->gpio_int, pdata->int_flags, pdata->gpio_en, pdata->use_int ? 1 : 0);
		if (ret) {
			dev_err(&pdev->dev, "Failed to parse device tree\n");
			ret = -EINVAL;
			goto err_free_devmem;
		}
	} else {
		pdata = pdev->dev.platform_data;
	}

	if (!pdata) {
		dev_err(&pdev->dev, "Cannot get device platform data\n");
		ret = -EINVAL;
		goto err_free_devmem;
	}

	mutex_init(&sensor->op_lock);
	sensor->pdata = pdata;
	sensor->enable_gpio = sensor->pdata->gpio_en;

	if ((sensor->pdata->use_int) &&
		gpio_is_valid(sensor->pdata->gpio_int)) {
		sensor->use_poll = 0;

		/* configure interrupt gpio */
		ret = gpio_request(sensor->pdata->gpio_int,
							"icm_gpio_int");
		if (ret) {
			dev_err(&pdev->dev,
				"Unable to request interrupt gpio %d\n",
				sensor->pdata->gpio_int);
			goto err_free_devmem;
		}

		ret = gpio_direction_input(sensor->pdata->gpio_int);
		if (ret) {
			dev_err(&pdev->dev,
				"Unable to set direction for gpio %d\n",
				sensor->pdata->gpio_int);
			goto err_free_gpio;
		}
		sensor->irq = gpio_to_irq(sensor->pdata->gpio_int);

		ret = request_threaded_irq(sensor->irq,
				     icm_interrupt_routine, icm_interrupt_thread,
				     sensor->pdata->int_flags | IRQF_ONESHOT,
				     "icm", sensor);
		disable_irq(sensor->irq);
		if (ret) {
			dev_err(&pdev->dev,
				"Can't get IRQ %d, error %d\n",
				sensor->irq, ret);
			sensor->irq = 0;
			goto err_free_gpio;
		}

	} else {
		sensor->use_poll = 1;
		dev_dbg(&pdev->dev,
			"Polling mode is enabled. use_int=%d gpio_int=%d",
			sensor->pdata->use_int, sensor->pdata->gpio_int);
	}

	ret = icm_probe_cci(pdev);
	if (ret) {
		icm_errmsg("icm_probe_cci fail\n");
		goto err_free_gpio;
	}

	ret = icm_pinctrl_init(sensor);
	if (ret) {
		dev_err(&pdev->dev, "Can't initialize pinctrl\n");
		goto err_power_down;
	}
	
	if (gpio_is_valid(sensor->enable_gpio)) {
		ret = gpio_request(sensor->enable_gpio, "ICM_EN_PM");
		gpio_direction_output(sensor->enable_gpio, 0);
	}
	
	ret = icm_power_init(sensor);
	if (ret) {
		dev_err(&pdev->dev, "Failed to init regulator\n");
		goto err_free_enable_gpio;
	}

	ret = icm_power_ctl(sensor, true);
	if (ret) {
		dev_err(&pdev->dev, "Failed to power on device\n");
		goto err_deinit_regulator;
	}

	ret = icm_check_chip_type(sensor);
	if (ret) {
		dev_err(&pdev->dev, "Cannot get invalid chip type\n");
		goto err_power_off_device;
	}

	icm_dbgmsg("sensor->deviceid=0x%02X, sensor->chip_type=0x%02X\n", sensor->deviceid, sensor->chip_type);
	if(sensor->deviceid == ICM20690_ID)
		g_status = 1;
	
	ret = icm_init_engine(sensor);
	if (ret) {
		dev_err(&pdev->dev, "Failed to init chip engine\n");
		goto err_power_off_device;
	}

	sensor->cfg.is_asleep = false;
	atomic_set(&sensor->accel_en, 0);
	atomic_set(&sensor->gyro_en, 0);
	ret = icm_init_config(sensor);
	if (ret) {
		dev_err(&pdev->dev, "Failed to set default config\n");
		goto err_power_off_device;
	}
	
	sensor->accel_dev = devm_input_allocate_device(&pdev->dev);
	if (!sensor->accel_dev) {
		dev_err(&pdev->dev,
			"Failed to allocate accelerometer input device\n");
		ret = -ENOMEM;
		goto err_power_off_device;
	}

	sensor->gyro_dev = devm_input_allocate_device(&pdev->dev);
	if (!sensor->gyro_dev) {
		dev_err(&pdev->dev,
			"Failed to allocate gyroscope input device\n");
		ret = -ENOMEM;
		goto err_power_off_device;
	}

	sensor->accel_dev->name = ICM_DEV_NAME_ACCEL;
	sensor->gyro_dev->name = ICM_DEV_NAME_GYRO;
	sensor->accel_dev->id.bustype = BUS_I2C;
	sensor->gyro_dev->id.bustype = BUS_I2C;
	sensor->accel_poll_ms = ICM_ACCEL_DEFAULT_POLL_INTERVAL_MS;
	sensor->gyro_poll_ms = ICM_GYRO_DEFAULT_POLL_INTERVAL_MS;
	sensor->acc_use_cal = false;
	
	input_set_capability(sensor->accel_dev, EV_ABS, ABS_WHEEL); //sec
	input_set_capability(sensor->accel_dev, EV_ABS, ABS_GAS); //nsec
	input_set_capability(sensor->gyro_dev, EV_ABS, ABS_WHEEL); //sec
	input_set_capability(sensor->gyro_dev, EV_ABS, ABS_GAS); //nsec

	input_set_capability(sensor->accel_dev, EV_ABS, ABS_MISC);
	input_set_capability(sensor->gyro_dev, EV_ABS, ABS_MISC);
	input_set_abs_params(sensor->accel_dev, ABS_X,
			ICM_ACCEL_MIN_VALUE, ICM_ACCEL_MAX_VALUE,
			0, 0);
	input_set_abs_params(sensor->accel_dev, ABS_Y,
			ICM_ACCEL_MIN_VALUE, ICM_ACCEL_MAX_VALUE,
			0, 0);
	input_set_abs_params(sensor->accel_dev, ABS_Z,
			ICM_ACCEL_MIN_VALUE, ICM_ACCEL_MAX_VALUE,
			0, 0);
	input_set_abs_params(sensor->gyro_dev, ABS_RX,
			     ICM_GYRO_MIN_VALUE, ICM_GYRO_MAX_VALUE,
			     0, 0);
	input_set_abs_params(sensor->gyro_dev, ABS_RY,
			     ICM_GYRO_MIN_VALUE, ICM_GYRO_MAX_VALUE,
			     0, 0);
	input_set_abs_params(sensor->gyro_dev, ABS_RZ,
			     ICM_GYRO_MIN_VALUE, ICM_GYRO_MAX_VALUE,
			     0, 0);
	sensor->accel_dev->dev.parent = &pdev->dev;
	sensor->gyro_dev->dev.parent = &pdev->dev;
	input_set_drvdata(sensor->accel_dev, sensor);
	input_set_drvdata(sensor->gyro_dev, sensor);

	sensor->data_wq = create_freezable_workqueue("icm_data_work");
	if (!sensor->data_wq) {
		dev_err(&pdev->dev, "Cannot create workqueue!\n");
		goto err_power_off_device;
	}

	INIT_DELAYED_WORK(&sensor->fifo_flush_work, icm_fifo_flush_fn);
	INIT_WORK(&sensor->resume_work, icm_resume_work_fn);

	hrtimer_init(&sensor->gyro_timer, CLOCK_BOOTTIME, HRTIMER_MODE_REL);
	sensor->gyro_timer.function = gyro_timer_handle;
	hrtimer_init(&sensor->accel_timer, CLOCK_BOOTTIME, HRTIMER_MODE_REL);
	sensor->accel_timer.function = accel_timer_handle;

	init_waitqueue_head(&sensor->gyro_wq);
	init_waitqueue_head(&sensor->accel_wq);
	sensor->gyro_wkp_flag = 0;
	sensor->accel_wkp_flag = 0;

	sensor->gyr_task = kthread_run(gyro_poll_thread, sensor, "sns_gyro");
	sensor->accel_task = kthread_run(accel_poll_thread, sensor,
						"sns_accel");

	ret = input_register_device(sensor->accel_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register input device\n");
		goto err_destroy_workqueue;
	}
	ret = input_register_device(sensor->gyro_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register input device\n");
		goto err_destroy_workqueue;
	}

	create_asusIcm206xxPolling_proc_file();
	create_asusIcm206xxDebug_proc_file();
	icm206xx_classRegister();
	icm206xx_miscRegister();

	ret = create_accel_sysfs_interfaces(&sensor->accel_dev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create sysfs for accel\n");
		goto err_destroy_workqueue;
	}
	ret = create_gyro_sysfs_interfaces(&sensor->gyro_dev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create sysfs for gyro\n");
		goto err_remove_accel_sysfs;
	}

#ifdef SENSORS_CLASSDEV
	sensor->accel_cdev = icm_acc_cdev;
	sensor->accel_cdev.delay_msec = sensor->accel_poll_ms;
	sensor->accel_cdev.sensors_enable = icm_accel_cdev_enable;
	sensor->accel_cdev.sensors_poll_delay = icm_accel_cdev_poll_delay;
	sensor->accel_cdev.sensors_enable_wakeup =
					icm_accel_cdev_enable_wakeup;
	sensor->accel_cdev.fifo_reserved_event_count = 0;
	sensor->accel_cdev.sensors_set_latency = icm_accel_cdev_set_latency;
	sensor->accel_cdev.sensors_flush = icm_accel_cdev_flush;
	sensor->accel_cdev.sensors_calibrate = icm_accel_calibration;
	sensor->accel_cdev.sensors_write_cal_params =
		icm_write_accel_cal_params;
	if ((sensor->pdata->use_int) &&
			gpio_is_valid(sensor->pdata->gpio_int))
		sensor->accel_cdev.max_delay = ICM_ACCEL_INT_MAX_DELAY;

	ret = sensors_classdev_register(&sensor->accel_dev->dev,
			&sensor->accel_cdev);
	if (ret) {
		dev_err(&pdev->dev,
			"create accel class device file failed!\n");
		ret = -EINVAL;
		goto err_remove_gyro_sysfs;
	}

	sensor->gyro_cdev = icm_gyro_cdev;
	sensor->gyro_cdev.delay_msec = sensor->gyro_poll_ms;
	sensor->gyro_cdev.sensors_enable = icm_gyro_cdev_enable;
	sensor->gyro_cdev.sensors_poll_delay = icm_gyro_cdev_poll_delay;
	sensor->gyro_cdev.fifo_reserved_event_count = 0;
	sensor->gyro_cdev.sensors_set_latency = icm_gyro_cdev_set_latency;
	sensor->gyro_cdev.sensors_flush = icm_gyro_cdev_flush;
	if ((sensor->pdata->use_int) &&
			gpio_is_valid(sensor->pdata->gpio_int))
		sensor->gyro_cdev.max_delay = ICM_GYRO_INT_MAX_DELAY;

	ret = sensors_classdev_register(&sensor->gyro_dev->dev,
			&sensor->gyro_cdev);
	if (ret) {
		dev_err(&pdev->dev,
			"create accel class device file failed!\n");
		ret = -EINVAL;
		goto err_remove_accel_cdev;
	}
#endif //#ifdef SENSORS_CLASSDEV

	if (g_icm_ctrl) {
		icm206xx_power_ctl_cci(g_icm_ctrl, false);
	}
	icm_dbgmsg("-\n");
	return 0;

#ifdef SENSORS_CLASSDEV
err_remove_gyro_cdev:
	sensors_classdev_unregister(&sensor->gyro_cdev);
err_remove_accel_cdev:
	 sensors_classdev_unregister(&sensor->accel_cdev);
err_remove_gyro_sysfs:
	remove_accel_sysfs_interfaces(&sensor->gyro_dev->dev);
#endif //#ifdef SENSORS_CLASSDEV

err_remove_accel_sysfs:
	remove_accel_sysfs_interfaces(&sensor->accel_dev->dev);
err_destroy_workqueue:
	destroy_workqueue(sensor->data_wq);
	if (sensor->irq > 0)
		free_irq(sensor->irq, sensor);
	hrtimer_try_to_cancel(&sensor->gyro_timer);
	hrtimer_try_to_cancel(&sensor->accel_timer);
	kthread_stop(sensor->gyr_task);
	kthread_stop(sensor->accel_task);

err_power_off_device:
	icm_power_ctl(sensor, false);
err_deinit_regulator:
	icm_power_deinit(sensor);
err_free_enable_gpio:
	if (gpio_is_valid(sensor->enable_gpio))
		gpio_free(sensor->enable_gpio);
err_power_down:
	if (g_icm_ctrl) {
		icm206xx_power_ctl_cci(g_icm_ctrl, false);
	}
err_free_gpio:
	if ((sensor->pdata->use_int) &&
		(gpio_is_valid(sensor->pdata->gpio_int)))
		gpio_free(sensor->pdata->gpio_int);
err_free_devmem:
	wakeup_source_trash(&sensor->icm206xx_wakeup_source);
	devm_kfree(&pdev->dev, sensor);
	dev_err(&pdev->dev, "Probe device return error%d\n", ret);

	return ret;
}


/*
 * icm_remove() - remove a sensor
 * @client: i2c client of sensor being removed
 *
 * Our sensor is going away, clean up the resources.
 */
static int icm_platform_remove(struct platform_device *pdev)
{
	struct icm_sensor *sensor = dev_get_drvdata(&pdev->dev);

	icm_dbgmsg("+\n");
	icm_remove_cci(pdev);

#ifdef SENSORS_CLASSDEV
	sensors_classdev_unregister(&sensor->accel_cdev);
	sensors_classdev_unregister(&sensor->gyro_cdev);
#endif
	remove_gyro_sysfs_interfaces(&sensor->gyro_dev->dev);
	remove_accel_sysfs_interfaces(&sensor->accel_dev->dev);
	destroy_workqueue(sensor->data_wq);
	hrtimer_try_to_cancel(&sensor->gyro_timer);
	hrtimer_try_to_cancel(&sensor->accel_timer);
	kthread_stop(sensor->gyr_task);
	kthread_stop(sensor->accel_task);
	if (sensor->irq > 0)
		free_irq(sensor->irq, sensor);
	if ((sensor->pdata->use_int) &&
		(gpio_is_valid(sensor->pdata->gpio_int)))
		gpio_free(sensor->pdata->gpio_int);
	icm_power_ctl(sensor, false);
	icm_power_deinit(sensor);
	if (gpio_is_valid(sensor->enable_gpio))
		gpio_free(sensor->enable_gpio);
	wakeup_source_trash(&sensor->icm206xx_wakeup_source);
	devm_kfree(sensor->dev, sensor);

	icm_dbgmsg("-\n");
	return 0;
}

static void icm_resume_work_fn(struct work_struct *work)
{
	struct icm_sensor *sensor;
	int ret = 0;

	icm_dbgmsg("\n");
	sensor = container_of(work,
			struct icm_sensor, resume_work);

	mutex_lock(&sensor->op_lock);
	if ((sensor->batch_accel) || (sensor->batch_gyro)) {
		icm_set_interrupt(sensor,
				BIT_FIFO_OVERFLOW, true);
		icm_sche_next_flush(sensor);
	}

	if (sensor->cfg.mot_det_on) {
		/* keep accel on and config motion detection wakeup */
		irq_set_irq_wake(sensor->irq, 0);
		icm_set_motion_det(sensor, false);
		icm_set_interrupt(sensor,
				BIT_DATA_RDY_EN, true);
		dev_dbg(sensor->dev,
				"Disable motion detection success\n");
		goto exit;
	}

	/* Keep sensor power on to prevent bad power state */
	ret = icm_power_ctl(sensor, true);
	if (ret < 0) {
		dev_err(sensor->dev, "Power on icm failed\n");
		goto exit;
	}
	/* Reset sensor to recovery from unexpected state */
	icm_reset_chip(sensor);

	ret = icm_restore_context(sensor);
	if (ret < 0) {
		dev_err(sensor->dev, "Failed to restore context\n");
		goto exit;
	}

	/* Enter sleep mode if both accel and gyro are not enabled */
	ret = icm_set_power_mode(sensor, sensor->cfg.enable);
	if (ret < 0) {
		dev_err(sensor->dev, "Failed to set power mode enable=%d\n",
				sensor->cfg.enable);
		goto exit;
	}

	if (sensor->cfg.gyro_enable) {
		ret = icm_gyro_enable(sensor, true);
		if (ret < 0) {
			dev_err(sensor->dev, "Failed to enable gyro\n");
			goto exit;
		}

		if (sensor->use_poll) {
			ktime_t ktime;

			ktime = ktime_set(0,
					sensor->gyro_poll_ms * NSEC_PER_MSEC);
			hrtimer_start(&sensor->gyro_timer, ktime,
					HRTIMER_MODE_REL);
		} else{
			icm_set_interrupt(sensor, BIT_DATA_RDY_EN, true);
		}
	}

	if (sensor->cfg.accel_enable) {
		ret = icm_accel_enable(sensor, true);
		if (ret < 0) {
			dev_err(sensor->dev, "Failed to enable accel\n");
			goto exit;
		}

		if (sensor->use_poll) {
			ktime_t ktime;

			ktime = ktime_set(0,
					sensor->accel_poll_ms * NSEC_PER_MSEC);
			hrtimer_start(&sensor->accel_timer, ktime,
					HRTIMER_MODE_REL);
		} else{
			icm_set_interrupt(sensor, BIT_DATA_RDY_EN, true);
		}
	}

exit:
	mutex_unlock(&sensor->op_lock);
	dev_dbg(sensor->dev, "Resume complete, ret = %d\n", ret);
}

#ifdef CONFIG_PM
/*
 * icm_suspend() - called on device suspend
 * @dev: device being suspended
 *
 * Put the device into sleep mode before we suspend the machine.
 */
static int icm_suspend(struct device *dev)
{
	struct icm_sensor *sensor = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&sensor->op_lock);
	if (!(sensor->cfg.gyro_enable) && !(sensor->cfg.accel_enable))
		goto exit;

	if ((sensor->batch_accel) || (sensor->batch_gyro)) {
		icm_set_interrupt(sensor,
				BIT_FIFO_OVERFLOW, false);
		cancel_delayed_work_sync(&sensor->fifo_flush_work);
		goto exit;
	}
	if (sensor->motion_det_en) {
		/* keep accel on and config motion detection wakeup */
		ret = icm_set_interrupt(sensor,
				BIT_DATA_RDY_EN, false);
		if (ret == 0)
			ret = icm_set_motion_det(sensor, true);
		if (ret == 0) {
			irq_set_irq_wake(sensor->irq, 1);

			dev_dbg(sensor->dev,
				"Enable motion detection success\n");
			goto exit;
		}
		/* if motion detection config does not success,
		 * not exit suspend and sensor will be power off.
		 */
	}

	if (!sensor->use_poll) {
	} else {
		if (sensor->cfg.gyro_enable)
			ret = hrtimer_try_to_cancel(&sensor->gyro_timer);

		if (sensor->cfg.accel_enable)
			ret = hrtimer_try_to_cancel(&sensor->accel_timer);
	}

	icm_set_power_mode(sensor, false);
	ret = icm_power_ctl(sensor, false);
	if (ret < 0) {
		dev_err(sensor->dev, "Power off icm failed\n");
		goto exit;
	}

exit:
	mutex_unlock(&sensor->op_lock);
	dev_dbg(sensor->dev, "Suspend completed, ret=%d\n", ret);

	return ret;
}

/*
 * icm_resume() - called on device resume
 * @dev: device being resumed
 *
 * Put the device into powered mode on resume.
 */
static int icm_resume(struct device *dev)
{
	struct icm_sensor *sensor = dev_get_drvdata(dev);
	
	mutex_lock(&sensor->op_lock);

	if (sensor->cfg.gyro_enable || sensor->cfg.accel_enable)
		queue_work(sensor->data_wq, &sensor->resume_work);

	mutex_unlock(&sensor->op_lock);

	return 0;
}

static UNIVERSAL_DEV_PM_OPS(icm_pm, icm_suspend, icm_resume, NULL);
#endif



#ifdef CAMERA_CCI
static int icm206xx_request_pwren(struct icm_ctrl_t *icm_ctrl)
{
        int rc = 0;

        icm_ctrl->io_flag.pwr_owned = 0;
        if (icm_ctrl->pwren_gpio == -1) {
                goto no_gpio;
        }

        rc = gpio_request(icm_ctrl->pwren_gpio, "icm206xx_pwren");
        if (rc) {
                goto no_gpio;
        }

        rc = gpio_direction_output(icm_ctrl->pwren_gpio, 0);
        if (rc) {
                goto direction_failed;
        }
        icm_ctrl->io_flag.pwr_owned = 1;

        return rc;

direction_failed:
        gpio_free(icm_ctrl->pwren_gpio);

no_gpio:
        return rc;
}

static void icm206xx_release_pwren(struct icm_ctrl_t *icm_ctrl)
{
        if (icm_ctrl->io_flag.pwr_owned) {
                gpio_free(icm_ctrl->pwren_gpio);
                icm_ctrl->io_flag.pwr_owned = 0;
        }

        icm_ctrl->pwren_gpio = -1;
}

static int icm206xx_request_intr(struct icm_ctrl_t *icm_ctrl)
{
        int rc = 0;
        const char *desc = "icm_irq";

        icm_ctrl->io_flag.intr_owned = 0;
        if (icm_ctrl->intr_gpio == -1) {
                goto end;
        }

        icm_ctrl->irq = gpio_to_irq(icm_ctrl->intr_gpio);
        if (icm_ctrl->irq < 0) {
                goto end;
        }

        rc = devm_gpio_request_one(&icm_ctrl->pdev->dev, icm_ctrl->intr_gpio, GPIOF_IN, desc);
        if (rc < 0) {
                goto end;
        }

        icm_ctrl->io_flag.intr_owned = 1;
end:
        return rc;
}

static void icm206xx_release_intr(struct icm_ctrl_t *icm_ctrl)
{
        if (icm_ctrl->io_flag.intr_owned) {
                if (icm_ctrl->io_flag.intr_started) {
                        free_irq(icm_ctrl->irq, icm_ctrl);
                        icm_ctrl->io_flag.intr_started = 0;
                }
                gpio_free(icm_ctrl->intr_gpio);
                icm_ctrl->io_flag.intr_owned = 0;
        }
        icm_ctrl->intr_gpio = -1;
}

static void icm206xx_release_gpios_cci(struct icm_ctrl_t *i_ctrl)
{
        if (i_ctrl->power_supply) {
                regulator_put(i_ctrl->power_supply);
                i_ctrl->power_supply = NULL;
        }
        icm206xx_release_pwren(i_ctrl);
        icm206xx_release_intr(i_ctrl);
}

static icm206xx_get_dt_info(struct device *dev, struct icm_ctrl_t *i_ctrl)
{
        int rc = 0;
        struct device_node *of_node = NULL;

        if (!dev || !i_ctrl)
            return -EINVAL;

        of_node = dev->of_node;
        if (!of_node) {
            printk("icm206xx of_node is NULL%d\n", __LINE__);
            return -EINVAL;
        }

        i_ctrl->pwren_gpio = -1;
        i_ctrl->intr_gpio  = -1;

        rc = of_property_read_u32(of_node, "cell-index", &i_ctrl->pdev->id);
        if (rc < 0) {
            printk("icm206xx failed to read cell index%d\n", __LINE__);
            return rc;
        }

        rc = of_property_read_u32(of_node, "cci-master", &i_ctrl->cci_master);
        if (rc < 0) {
            i_ctrl->cci_num = CCI_DEVICE_0;
            rc = 0;
        }
        i_ctrl->io_master_info.cci_client->cci_device = i_ctrl->cci_num;

        i_ctrl->power_supply = regulator_get(dev, "icm206xx");
        if (IS_ERR(i_ctrl->power_supply) || i_ctrl->power_supply == NULL) {
            i_ctrl->power_supply = NULL;
            rc = of_property_read_u32_array(dev->of_node, "pwren-gpio", &i_ctrl->pwren_gpio, 1);
            if (rc) {
                i_ctrl->pwren_gpio = -1;
                printk("icm206xx no regulator, nor power gpio => power ctrl disabled\n");
            }
        }

        i_ctrl->cci_supply = regulator_get(dev, "cci");
        if (IS_ERR(i_ctrl->cci_supply)) {
            i_ctrl->cci_supply = NULL;
            printk("icm206xx Unable to cci power supply\n");
        }

        rc = icm206xx_request_pwren(i_ctrl);
        if (rc)
            goto no_pwren;
        rc = icm206xx_request_intr(i_ctrl);
        if (rc)
            goto no_intr;

        return rc;

no_intr:
        if (i_ctrl->power_supply) {
            regulator_put(i_ctrl->power_supply);
            i_ctrl->power_supply = NULL;
        }
no_pwren:
        icm206xx_release_pwren(i_ctrl);

        return rc;
}

static ssize_t accel_status_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_status);
}

static ssize_t gyro_status_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_status);
}

static struct device_attribute dev_attr_accel_status =
__ATTR(accel_status, 0664, accel_status_show, NULL);

static struct device_attribute dev_attr_gyro_status =
__ATTR(gyro_status, 0664, gyro_status_show, NULL);

static struct attribute *icm206xx_attributes[] = {
        &dev_attr_accel_status.attr,
        &dev_attr_gyro_status.attr,
        NULL
};

static struct attribute_group icm206xx_attr_group = {
        .attrs = icm206xx_attributes,
};

static int msm_icm_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
        int rc = 0;
        return rc;
}

static const struct v4l2_subdev_internal_ops msm_icm_internal_ops = {
        .close = msm_icm_close,
};

static long msm_icm_subdev_ioctl(struct v4l2_subdev *sd,
                                 unsigned int cmd, void *arg)
{
        int32_t rc = 0;
        return rc;
}

static int32_t msm_icm_power(struct v4l2_subdev *sd, int on)
{
        return 0;
}

static struct v4l2_subdev_core_ops msm_icm_subdev_core_ops = {
        .ioctl = msm_icm_subdev_ioctl,
        .s_power = msm_icm_power,
};

static struct v4l2_subdev_ops msm_icm_subdev_ops = {
        .core = &msm_icm_subdev_core_ops,
};

static int icm206xx_input_setup(struct icm206xx_data *icm206xx)
{
        int rc = 0;
        struct input_dev *idev;

        idev = input_allocate_device();
        if (idev == NULL) {
            rc = -ENOMEM;
            goto exit_err;
        }

        idev->name = "Invensense ICM206xx Accel_Gyro";
        rc = input_register_device(idev);
        if (rc) {
            rc = -ENOMEM;
            printk("icm206xx register input device fail rc = %d\n", rc);
            goto exit_free_dev;
        }

        input_set_drvdata(idev, icm206xx);
        icm206xx->input_dev_icm206xx = idev;
        return 0;

exit_free_dev:
        input_free_device(icm206xx->input_dev_icm206xx);
exit_err:
        return rc;
}

int icm206xx_setup(struct icm206xx_data *icm206xx)
{
        int rc = 0;
        icm_dbgmsg("++\n");

        rc = icm206xx_power_ctl_cci(icm206xx->client_object, true);
        if (rc) {
            printk("icm206xx power_up error rc %d\n", rc);
            goto exit_cleanup;
        }

        rc = icm206xx_input_setup(icm206xx);
        if (rc) {
            printk("icm206xx inpu setup fail%d\n", rc);
            goto exit_power_down;
        }

        rc = sysfs_create_group(&icm206xx->input_dev_icm206xx->dev.kobj,
                                &icm206xx_attr_group);
        if (rc) {
            rc = -ENOMEM;
            printk("icm206xx create sysfs fail\n");
            goto exit_remove_sysfs;
        }
        icm_dbgmsg("--\n");
        return 0;

exit_remove_sysfs:
        sysfs_remove_group(&icm206xx->input_dev_icm206xx->dev.kobj,
                           &icm206xx_attr_group);
exit_power_down:
        icm206xx_power_ctl_cci(icm206xx->client_object, false);
exit_cleanup:
        icm206xx_clean_up_cci();
		

        return rc;
}

int icm_probe_cci(struct platform_device *pdev)
{
        int32_t rc = 0;
        struct icm206xx_data *icm206xx = NULL;
        struct icm_ctrl_t *icm_ctrl    = NULL;
        struct cam_sensor_cci_client *cci_client = NULL;
		struct icm_sensor *sensor;

		icm_dbgmsg("+\n");
		sensor = dev_get_drvdata(&pdev->dev);
		
        icm206xx = kzalloc(sizeof(struct icm206xx_data), GFP_KERNEL);
        if (!icm206xx) {
           return -ENOMEM;
        }

        if (icm206xx) {
            icm206xx->client_object = kzalloc(sizeof(struct icm_ctrl_t), GFP_KERNEL);
            if (!icm206xx->client_object) {
                rc = -ENOMEM;
                goto free_icm206xx_data;
            }
            icm_ctrl = (struct icm_ctrl_t *)icm206xx->client_object;
        }
        g_icm_ctrl = icm_ctrl;
        icm_ctrl->pdev = pdev;

        icm_ctrl->icm206xx = icm206xx;
        icm_ctrl->device_type = MSM_CAMERA_PLATFORM_DEVICE;

        icm_ctrl->io_master_info.master_type = CCI_MASTER;
        icm_ctrl->io_master_info.cci_client = kzalloc(sizeof(struct cam_sensor_cci_client),
                                                     GFP_KERNEL);
        if (!icm_ctrl->io_master_info.cci_client) {
            goto free_icm_ctrl;
        }

        rc = icm206xx_get_dt_info(&pdev->dev, icm_ctrl);
        if (rc < 0) {
            icm_errmsg("icm206xx %d, fail to get dt info rc %d\n", __LINE__, rc);
            goto free_cci_client;
        }
        rc = msm_camera_pinctrl_init(&(icm_ctrl->pinctrl_info), &icm_ctrl->pdev->dev);
		
        cci_client = icm_ctrl->io_master_info.cci_client;
        cci_client->cci_i2c_master = icm_ctrl->cci_master;
        cci_client->sid = 0x68;
        cci_client->retries = 3;
        cci_client->id_map = 0;
        cci_client->i2c_freq_mode = I2C_FAST_MODE;

		sensor->cci_client = cci_client;
		sensor->io_master_info = &icm_ctrl->io_master_info;
		
        icm_ctrl->v4l2_dev_str.internal_ops = &msm_icm_internal_ops;
        icm_ctrl->v4l2_dev_str.ops = &msm_icm_subdev_ops;
        strlcpy(icm_ctrl->device_name, ICM_SENSOR_NAME, sizeof(icm_ctrl->device_name));
        icm_ctrl->v4l2_dev_str.name = icm_ctrl->device_name;
        icm_ctrl->v4l2_dev_str.sd_flags = (V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS);
        icm_ctrl->v4l2_dev_str.ent_function = 0x000100ff;
        icm_ctrl->v4l2_dev_str.token = icm_ctrl;

        rc = cam_register_subdev(&(icm_ctrl->v4l2_dev_str));
        if (rc) {
            icm_errmsg("fail to create subdev\n");
            goto unregister_subdev;
        }

        g_icm_v4l2_dev_str = &icm_ctrl->v4l2_dev_str;

													//#KILL		/* setup device data */
													//#KILL        dev_set_drvdata(&pdev->dev, icm206xx);

        rc = icm206xx_setup(icm206xx);
        if (rc) {
            goto release_gpios;
        }
        kref_init(&icm_ctrl->ref);

		icm_dbgmsg("-\n");		
        return rc;

release_gpios:
        icm206xx_release_gpios_cci(icm_ctrl);
unregister_subdev:
        cam_unregister_subdev(&(icm_ctrl->v4l2_dev_str));
free_cci_client:
        kfree(icm_ctrl->io_master_info.cci_client);
free_icm_ctrl:
        kfree(icm_ctrl);
free_icm206xx_data:
        kfree(icm206xx);
        return rc;
}

int icm_remove_cci(struct platform_device *pdev)
{
        int ret = 0;
        struct icm206xx_data *icm206xx = platform_get_drvdata(pdev);
        struct icm_ctrl_t *icm_ctrl = (struct icm_ctrl_t *)icm206xx->client_object;
		
		icm_dbgmsg("+\n");
        mutex_lock(&icm206xx->work_mutex);

        icm206xx_clean_up_cci();

        if (icm_ctrl->cam_pinctrl_status) {
            ret = pinctrl_select_state(
                    icm_ctrl->pinctrl_info.pinctrl,
                    icm_ctrl->pinctrl_info.gpio_state_suspend);
            printk("icm206xx cannot set pin to suspend state\n");

            devm_pinctrl_put(icm_ctrl->pinctrl_info.pinctrl);
        }

        icm206xx_release_gpios_cci(icm_ctrl);
        platform_set_drvdata(pdev, NULL);
        mutex_unlock(&icm206xx->work_mutex);

        kfree(icm206xx->client_object);
        kfree(icm206xx);

		icm_dbgmsg("-\n");
        return 0;
}

int icm206xx_power_ctl_cci(void *object, bool enable)
{
	int ret = 0;
	static int l_count = 0;
	struct icm_ctrl_t *icm_ctrl = (struct icm_ctrl_t *)object;
	struct icm_sensor *sensor = dev_get_drvdata(&icm_ctrl->pdev->dev);

	icm_dbgmsg("enable = %d, l_count = %d\n", enable ? 1 : 0, l_count);
	if ((enable && l_count == 0) || (!enable && l_count == 1)) {
		if (enable) {
			ret = icm206xx_power_up_cci(object);
			if (sensor->irq) {
				enable_irq(sensor->irq);
			} else{
				icm_errmsg("invalid irq");
			}
			/*ASUS_BSP: initial the report time, and continuously print some information when sensor is enabled*/
			g_next_report_time_s = 2;
			schedule_delayed_work(&work_report, 0);
		} else{
			icm_power_ctl(sensor, false);
			if (sensor->irq) {
				disable_irq(sensor->irq);
			} else{
				icm_errmsg("invalid irq");
			}
			ret = icm206xx_power_down_cci(object);
			
			/*ASUS_BSP: cancel report worker when sensor is disabled*/
			cancel_delayed_work(&work_report);
		}
	}

	if (enable) {
		l_count++;
	} else{
		l_count--;
	}
	return ret;
}
int icm206xx_power_up_cci(void *object)
{
        int rc = 0;
        struct icm_ctrl_t *icm_ctrl = (struct icm_ctrl_t *)object;
        struct icm_sensor *sensor = dev_get_drvdata(&icm_ctrl->pdev->dev);

        if (!icm_ctrl) {
                printk( "stmvl53l1_power_up_cci failed %d\n", __LINE__);
                return -EINVAL;
        }

        /* turn on power */
        if (icm_ctrl->power_supply) {
                //rc = cam_soc_util_regulator_enable(tof_ctrl->power_supply, "laser", 2800000, 2800000, 80000, 0);
                rc = cam_soc_util_regulator_enable(icm_ctrl->power_supply, "icm206xx", 1710000, 3450000, 80000, 0);
                rc |= regulator_enable(icm_ctrl->cci_supply);
                if (rc) {
                        printk("fail to turn on regulator\n");
                        return rc;
                }
                msleep(POWER_UP_TIME_MS);
        } else if (icm_ctrl->pwren_gpio != -1) {
                gpio_set_value_cansleep(icm_ctrl->pwren_gpio, 1);
                printk("icm206xx slow power on");
        } else {
                printk("icm206xx no power control");
        }
        rc = camera_io_init(&icm_ctrl->io_master_info);
        if (rc < 0)
                printk("icm206xx cci init failed: rc: %d", rc);

        __pm_stay_awake(&sensor->icm206xx_wakeup_source);

        return rc;
}

int icm206xx_power_down_cci(void *cci_object)
{
        int rc = 0;
        struct icm_ctrl_t *icm_ctrl = (struct icm_ctrl_t *)cci_object;
        struct icm_sensor *sensor = dev_get_drvdata(&icm_ctrl->pdev->dev);

        if (!icm_ctrl) {
                printk("icm206xx_power_down_cci failed %d\n", __LINE__);
                return -EINVAL;
        }

        /* turn off power */
        if (icm_ctrl->power_supply) {
                //rc = cam_soc_util_regulator_disable(tof_ctrl->power_supply, "laser", 2800000, 2800000, 80000, 0);
                rc = cam_soc_util_regulator_disable(icm_ctrl->power_supply, "icm206xx", 1710000, 3450000, 80000, 0);
                rc = regulator_disable(icm_ctrl->cci_supply);
                if (rc)
                        printk("reg disable failed. rc=%d\n", rc);
        } else if (icm_ctrl->pwren_gpio != -1) {
                gpio_set_value_cansleep(icm_ctrl->pwren_gpio, 0);
        }

        camera_io_release(&icm_ctrl->io_master_info);
        __pm_relax(&sensor->icm206xx_wakeup_source);

        return rc;
}

void icm206xx_clean_up_cci(void)
{
        int rc = 0;
        rc = cam_unregister_subdev(g_icm_v4l2_dev_str);
}

static int icm206xx_intr_handler(struct icm206xx_data *icm206xx)
{
        int rc;

        mutex_lock(&icm206xx->work_mutex);
        rc = 0;
        mutex_unlock(&icm206xx->work_mutex);

        return rc;
}

static irqreturn_t icm206xx_irq_handler_cci(int irq, void *object)
{
        //int rc = 0;
        struct icm_ctrl_t *icm_ctrl = (struct icm_ctrl_t *)object;

        if (!icm_ctrl) {
                printk("icm206xx Invalid parameter of intr function\n");
                return -EINVAL;
        }

        if (icm_ctrl->irq == irq) {
                icm206xx_intr_handler(icm_ctrl->icm206xx);
        }

        return IRQ_HANDLED;
}

int icm206xx_start_intr_cci(void *object, int *poll_mode)
{
        int rc = 0;
        struct icm_ctrl_t *icm_ctrl  = NULL;

        if (!object || !poll_mode) {
              printk("icm206xx Invalid parameter in intr function = %d\n", rc);
        }

        icm_ctrl = (struct icm_ctrl_t *)object;
        /* irq and gpio acquire config done in parse_tree */
        if (icm_ctrl->irq == 0) {
                /* the i2c tree as no intr force polling mode */
                *poll_mode = 1;
                return 0;
        }
        /* if started do no nothing */
        if (icm_ctrl->io_flag.intr_started) {
                *poll_mode = 0;
                return 0;
        }

        printk("ICM206xx to register_irq:%d\n", icm_ctrl->irq);
        rc = request_threaded_irq(icm_ctrl->irq, NULL,
                                        icm206xx_irq_handler_cci,
                                        IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
                                        "icm206xx_interrupt",
                                        (void *)icm_ctrl);
        if (rc) {
                printk("icm206xx fail to req threaded irq rc = %d\n", rc);
                *poll_mode = 1;
        } else {
                printk("icm206xx irq %d now handled \n", icm_ctrl->irq);
                icm_ctrl->io_flag.intr_started = 1;
                *poll_mode = 0;
        }
        return rc;
}

void *icm206xx_get_cci(void *object)
{
        struct icm_ctrl_t *icm_ctrl = (struct icm_ctrl_t *)object;
        kref_get(&icm_ctrl->ref);

        return object;
}

static void memory_release_cci(struct kref *kref)
{
        struct icm_ctrl_t *icm_ctrl = container_of(kref, struct icm_ctrl_t, ref);

        kfree(icm_ctrl->icm206xx);
        kfree(icm_ctrl);
}

void icm206xx_put_cci(void *object)
{
        struct icm_ctrl_t *icm_ctrl = (struct icm_ctrl_t *)object;

        kref_put(&icm_ctrl->ref, memory_release_cci);
}

s32 cci_read_byte_data(struct icm_sensor *sensor, u8 command)
{
	uint8_t data;
	int ret = 0;

	if(!sensor || !sensor->cci_client){
		icm_errmsg("cci_client is not initialized.\n");
		return -EINVAL;
	}

	ret = cam_camera_cci_i2c_read_seq(sensor->cci_client, command, &data, CAMERA_SENSOR_I2C_TYPE_BYTE,
			CAMERA_SENSOR_I2C_TYPE_BYTE, 1);
	return (ret==0? data: ret);
}

s32 cci_write_byte_data(struct icm_sensor *sensor, u8 command, u8 value)
{
	int ret = 0;
	struct cam_sensor_i2c_reg_setting write_reg_setting;
	struct cam_sensor_i2c_reg_array reg_array;
	
	if(!sensor || !sensor->io_master_info || !sensor->cci_client){
		icm_errmsg("cci_client is not initialized.\n");
		return -EINVAL;
	}	
	
	reg_array.reg_addr = command;
	reg_array.reg_data = value;
	
	write_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_reg_setting.size      = 1;
	write_reg_setting.reg_setting = &reg_array;
	write_reg_setting.delay     = 0;

	ret = cam_cci_i2c_write_table(sensor->io_master_info, &write_reg_setting);

	return ret;
}

s32 cci_mask_write_byte_data(struct icm_sensor *sensor, u8 command, u8 mask, u8 value)
{
	s32 rc, read_data;
 	u8 write_data;

	read_data = cci_read_byte_data(sensor, command);
	if (read_data < 0) {
		icm_errmsg("Failed to read reg 0x%02x, read_data=%d\n", command, read_data);
		goto out;
	}
	write_data = (u8)read_data;
	write_data &= ~mask;
	write_data |= value & mask;
	rc = cci_write_byte_data(sensor, command, write_data);
	if (rc) {
		icm_errmsg("Failed to write reg 0x%02x, rc=%d\n", command, rc);
	}
out:
	return rc;
}

static const struct of_device_id icm_of_match[] = {
        { .compatible = "invn,icm20690", },
        { },
};
MODULE_DEVICE_TABLE(of, icm_of_match);

static struct platform_driver icm_platform_driver = {
        .probe          = icm_platform_probe,
        .remove         = icm_platform_remove,
        .driver = {
                .name   = "icm20690",
                .owner  = THIS_MODULE,
                .pm     = &icm_pm,
                .of_match_table = icm_of_match,
        },
};

int icm_init_cci(void)
{
    int ret;
    
	icm_dbgmsg("+\n");
    ret = platform_driver_register(&icm_platform_driver);
    if (ret)
        icm_errmsg("error ret=%d\n", ret);	

    icm_dbgmsg("-\n");
    return ret;
}

void icm_exit_cci(void *object)
{
    struct icm_ctrl_t *icm_ctrl = (struct icm_ctrl_t *) object;

    if (icm_ctrl && icm_ctrl->io_master_info.cci_client)
        kfree(icm_ctrl->io_master_info.cci_client);	
}

void icm_clean_up_cci(void)
{
	cam_unregister_subdev(g_icm_v4l2_dev_str);
}
#endif //CAMERA_CCI

static int __init icm_platform_init(void)
{
	int ret;
	
	ret = icm_init_cci();
    if (ret)
        icm_errmsg("error ret=%d\n", ret);	
	
	return ret; 
}

static void __exit icm_platform_exit(void)
{
	icm_exit_cci(NULL);
	icm_clean_up_cci();
}

late_initcall(icm_platform_init);
module_exit(icm_platform_exit);

MODULE_DESCRIPTION("ICM Tri-axis gyroscope driver");
MODULE_LICENSE("GPL v2");
