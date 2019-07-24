obj-y 	+= ALSPS_FRGB_Hardware.o
obj-n	+= cm36656/
obj-y   += vcnl36863/
obj-n	+= example/

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DASH_HW_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)

