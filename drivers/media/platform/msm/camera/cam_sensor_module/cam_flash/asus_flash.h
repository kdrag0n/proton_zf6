#ifndef ASUS_FLASH_H
#define ASUS_FLASH_H

#include "cam_flash_dev.h"

void asus_flash_init(struct cam_flash_ctrl * ctrl);
void asus_flash_set_camera_state(uint8_t in_use);//set camera in use state

void asus_flash_set_led_fault(int error_value);//set fault from qpnp flash

int asus_flash_is_battery_low(void);//check if battery capacity low

#endif
