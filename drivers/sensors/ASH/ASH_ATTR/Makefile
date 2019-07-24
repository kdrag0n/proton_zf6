obj-y += ASH_ATTR.o
obj-y += HALLsensor_ATTR.o
obj-y += psensor_ATTR.o
obj-y += lsensor_ATTR.o
obj-y += FRGBsensor_ATTR.o
obj-y += SARsensor_ATTR.o

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DASH_ATTR_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)


