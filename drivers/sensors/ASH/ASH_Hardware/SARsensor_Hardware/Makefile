obj-y 	+= SARsensor_Hardware.o
#obj-y	+= sx9310/
obj-y	+= sx9325/
obj-n	+= example/

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DASH_HW_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)

#ccflags-y += -DUSE_SX9310
ccflags-y += -DUSE_SX9325
