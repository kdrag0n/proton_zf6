obj-y 	+= ALSPSsensor_Hardware.o
obj-n	+= cm36686/
obj-n	+= ap3425/
obj-n	+= ap3045/

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DASH_HW_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)

