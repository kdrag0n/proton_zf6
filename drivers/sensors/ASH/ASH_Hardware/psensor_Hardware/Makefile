obj-y 	+= IRsensor_Hardware.o
obj-n	+= cm36686/
obj-y	+= cm36656/
obj-n	+= ap3425/
obj-n	+= ap3045/
obj-y	+= cm36675/
obj-y	+= cm3327/

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DASH_HW_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)

