obj-y += ALSPSsensor_GPIO.o
obj-y += psensor_GPIO.o
obj-y += lsensor_GPIO.o
obj-y += HALLsensor_GPIO.o
obj-y += SARsensor_GPIO.o

INTEL = n
ifeq ($(INTEL),y)
	LOCAL_CFALG = -DGPIO_INTEL
else
	LOCAL_CFALG = -DGPIO_QCOM
endif

ccflags-y += $(LOCAL_CFALG)

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DASH_GPIO_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)

