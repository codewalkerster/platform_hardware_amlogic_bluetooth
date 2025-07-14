ifneq ($(BOARD_USE_ODM_WIFI_BT),true)
ifeq ($(BOARD_HAVE_BLUETOOTH_AMLOGIC),true)
LOCAL_PATH := $(call my-dir)
include $(call all-subdir-makefiles)
endif
endif
