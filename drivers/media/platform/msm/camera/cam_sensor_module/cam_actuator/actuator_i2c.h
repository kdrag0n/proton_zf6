#ifndef ASUS_ACTUATOR_I2C_H
#define ASUS_ACTUATOR_I2C_H
#include <linux/types.h>
#include <linux/delay.h>

#include "cam_actuator_dev.h"

int actuator_read_byte(struct cam_actuator_ctrl_t * ctrl,uint32_t reg_addr, uint32_t* reg_data, enum camera_sensor_i2c_type addr_type);
int actuator_read_word(struct cam_actuator_ctrl_t * ctrl,uint32_t reg_addr, uint32_t* reg_data, enum camera_sensor_i2c_type addr_type);
int actuator_read_dword(struct cam_actuator_ctrl_t * ctrl,uint32_t reg_addr, uint32_t* reg_data, enum camera_sensor_i2c_type addr_type);
int actuator_read_seq_bytes(struct cam_actuator_ctrl_t * ctrl,uint32_t reg_addr, uint8_t* reg_data, uint32_t size, enum camera_sensor_i2c_type addr_type);

int actuator_poll_byte(struct cam_actuator_ctrl_t * ctrl,uint32_t reg_addr, uint16_t reg_data, uint32_t delay_ms, enum camera_sensor_i2c_type addr_type);
int actuator_poll_word(struct cam_actuator_ctrl_t * ctrl,uint32_t reg_addr, uint16_t reg_data, uint32_t delay_ms, enum camera_sensor_i2c_type addr_type);

int actuator_write_byte(struct cam_actuator_ctrl_t * ctrl,uint32_t reg_addr, uint32_t reg_data, enum camera_sensor_i2c_type addr_type);
int actuator_write_word(struct cam_actuator_ctrl_t * ctrl,uint32_t reg_addr, uint32_t reg_data, enum camera_sensor_i2c_type addr_type);
int actuator_write_dword(struct cam_actuator_ctrl_t * ctrl,uint32_t reg_addr, uint32_t reg_data, enum camera_sensor_i2c_type addr_type);
int actuator_write_seq_bytes(struct cam_actuator_ctrl_t * ctrl,uint32_t reg_addr, uint8_t* reg_data,uint32_t size, enum camera_sensor_i2c_type addr_type);
#endif
