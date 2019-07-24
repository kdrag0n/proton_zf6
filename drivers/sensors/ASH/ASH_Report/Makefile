obj-y += psensor_Report.o
obj-n += lsensor_Report.o
obj-n += FRGBsensor_Report.o
obj-y += HALLsensor_Report.o
obj-y += LwithRGBsensor_Report.o
obj-y += SARsensor_Report.o

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DASH_REPORT_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)

