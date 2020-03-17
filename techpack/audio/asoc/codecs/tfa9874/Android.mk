LOCAL_PATH := $(call my-dir)

DLKM_DIR := $(TOP)/device/qcom/common/dlkm
ifneq ($(findstring opensource,$(LOCAL_PATH)),)
#	AUDIO_BLD_DIR := $(ANDROID_BUILD_TOP)/vendor/qcom/opensource/audio-kernel
	AUDIO_BLD_DIR := $(shell pwd)/vendor/qcom/opensource/audio-kernel
endif # opensource

# KBUILD_OPTIONS := AUDIO_ROOT=$(AUDIO_BLD_DIR)

#KBUILD_OPTIONS := AUDIO_ROOT=$(ANDROID_BUILD_TOP)/vendor/qcom/opensource/audio-kernel MODNAME=tfa9874 BOARD_PLATFORM=msmnile CONFIG_SND_SOC_SDM855=m V=1
KBUILD_OPTIONS := AUDIO_ROOT=$(AUDIO_BLD_DIR) MODNAME=tfa9874 BOARD_PLATFORM=msmnile CONFIG_SND_SOC_SDM855=m V=1

# $(warn TFA9874 mk file is parsed)

include $(CLEAR_VARS)
LOCAL_MODULE              := audio_tfa9874.ko
LOCAL_MODULE_KBUILD_NAME  := snd-soc-tfa9874.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE       := tfa98xx.cnt
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_SRC_FILES    := $(LOCAL_MODULE)
LOCAL_MODULE_PATH  := $(PRODUCT_OUT)/$(TARGET_COPY_OUT_VENDOR)/firmware
LOCAL_REQUIRED_MODULES := stereo.cnt
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE       := stereo.cnt
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_SRC_FILES    := $(LOCAL_MODULE)
LOCAL_MODULE_PATH  := $(PRODUCT_OUT)/$(TARGET_COPY_OUT_VENDOR)/firmware
include $(BUILD_PREBUILT)

