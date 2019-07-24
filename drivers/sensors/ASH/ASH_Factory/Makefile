obj-y += psensor_Factory.o
obj-y += lsensor_Factory.o
obj-y += FRGBsensor_Factory.o

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DASH_FACTORY_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)