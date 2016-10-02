LOCAL_PATH := $(call my-dir)
ifeq ($(strip $(BOARD_HAVE_WIFI)),true)

##################################################

include $(CLEAR_VARS)
LOCAL_MODULE := wifi_rtl_8723
LOCAL_MODULE_TAGS := optional
LOCAL_REQUIRED_MODULES :=  \
    lib_driver_cmd_rtl \
    wlan_prov.rtl

# WARNING: To be kept as the last required module.
LOCAL_REQUIRED_MODULES +=  \
    wifi_common
include $(BUILD_PHONY_PACKAGE)

##################################################

endif
