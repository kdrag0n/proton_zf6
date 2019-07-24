obj-y += ASH_Hardware.o
obj-y += HALLsensor_Hardware.o
obj-n += ALSPSsensor_Hardware/
obj-n += psensor_Hardware/
obj-n += lsensor_Hardware/
obj-y += ALSPS_FRGB_Hardware/
obj-y += SARsensor_Hardware/

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DASH_I2C_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)
