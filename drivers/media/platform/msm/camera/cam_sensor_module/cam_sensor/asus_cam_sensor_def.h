#ifndef ASUS_CAM_SENSOR_DEF_H
#define ASUS_CAM_SENSOR_DEF_H

#define SENSOR_ID_IMX214  0x0214
#define SENSOR_ID_IMX298  0x0298
#define SENSOR_ID_IMX351  0x0351
#define SENSOR_ID_IMX362  0x0362
#define SENSOR_ID_IMX363  0x0333
#define SENSOR_ID_IMX563  0x0563
#define SENSOR_ID_IMX586  0x0586

#define SENSOR_ID_OV5670  0x5670
#define SENSOR_ID_OV8856  0x885a
#define SENSOR_ID_OV13855  0xD855

#define SENSOR_ID_S5K3M3  0x30D3

#define PROC_REAR_MODULE_1	"driver/RearModule"
#define PROC_REAR_MODULE_2	"driver/RearModule2"
#define PROC_FRONT_MODULE_1 "driver/FrontModule"
#define PROC_FRONT_MODULE_2 "driver/FrontModule2"

#define OTP_DATA_LEN_WORD (32)
#define OTP_DATA_LEN_BYTE (OTP_DATA_LEN_WORD*2)
#define OTP_ID_LEN (12)

#define PROC_OTP_REAR       "driver/rear_otp"
#define PROC_OTP_REAR_AUX   "driver/rear2_otp"
#define PROC_OTP_FRONT      "driver/front_otp"
#define PROC_OTP_FRONT_AUX  "driver/front2_otp"

#define PROC_THERMAL_REAR	 "driver/rear_temp"
#define PROC_THERMAL_FRONT	 "driver/front_temp"
#define PROC_THERMAL_REAR_2	 "driver/rear2_temp"
#define PROC_THERMAL_FRONT_2 "driver/front2_temp"

#define THERMAL_TYPE_REAR	 "rear_camera"
#define THERMAL_TYPE_FRONT	 "front_camera"
#define THERMAL_TYPE_REAR_2	 "rear_camera2"
#define THERMAL_TYPE_FRONT_2 "front_camera2"

#define SYSFS_ROOT_DIR		 "camera_sensor"
#define SYSFS_RESOLUTION_DIR "resolution"
#define SYSFS_STATUS_DIR	 "status"

#define SYSFS_ATTR_REAR_1 camera_rear		//camera
#define SYSFS_ATTR_REAR_2 camera_rear_aux	//camera_2
#define SYSFS_ATTR_FRONT_1 camera_front		//vga
#define SYSFS_ATTR_FRONT_2 camera_front_aux //vga_2

#define PROC_SENSOR_I2C_RW "driver/sensor_i2c_rw"

#define PROC_EEPROM_REAR	"driver/rear_eeprom"
#define PROC_EEPROM_FRONT	"driver/front_eeprom"
#define PROC_EEPROM_REAR2	"driver/rear2_eeprom"
#define PROC_EEPROM_FRONT2	"driver/front2_eeprom"

#define PROC_EEPROM_I2C_R  "driver/eeprom_i2c_r"

#define PROC_ARCSOFT_CALI "driver/dualcam_cali"

#define	PROC_DIT_EEPROM_REAR	"driver/dit_rear_eeprom"
#define	PROC_DIT_EEPROM_FRONT	"driver/dit_front_eeprom"
#define	PROC_DIT_EEPROM_REAR2	"driver/dit_rear2_eeprom"
#define	PROC_DIT_EEPROM_FRONT2	"driver/dit_front2_eeprom"

#define FACTORYDIR "/vendor/factory/"
#define GOLDENDIR "/vendor/lib/camera/"
#define DIT_DUT_REAR "dut_rear.bin"
#define DIT_DUT_REAR2 "dut_rear2.bin"
#define DIT_DUT_FRONT "dut_rear.bin"
#define DIT_DUT_FRONT2 "dut_rear2.bin"
#define DUAL_CALI_BIN ""FACTORYDIR"dualcam_cali.bin"

#define PROC_SENSORS_RES "driver/camera_res"

#define PROC_MODULE_CHANGE_REAR "driver/rear_module_change"
#define PROC_MODULE_CHANGE_FRONT "driver/front_module_change"
#define PROC_MODULE_CHANGE_REAR2 "driver/rear2_module_change"
#define PROC_MODULE_CHANGE_FRONT2 "driver/front2_module_change"

#endif
