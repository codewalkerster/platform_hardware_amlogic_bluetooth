ifneq ($(BOARD_USE_ODM_WIFI_BT),true)
ifeq ($(BOARD_HAVE_BLUETOOTH_AMLOGIC),true)
LOCAL_PATH := $(call my-dir)
include $(call all-subdir-makefiles)

ifeq ($(shell test $(PLATFORM_SDK_VERSION) -lt 34 && echo OK),OK)
#for p/r/s/t fw copy
include $(CLEAR_VARS)
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE := w1_bt_fw_uart
LOCAL_SRC_FILES := ../sdio_driver_bt/w1_bt_fw_uart.bin
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_SUFFIX := .bin
LOCAL_MODULE_PATH  := $(TARGET_OUT_VENDOR)/lib/firmware
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE := w1u_bt_fw_uart
LOCAL_SRC_FILES := ../w1u_driver_bt/w1u_bt_fw_uart.bin
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_SUFFIX := .bin
LOCAL_MODULE_PATH  := $(TARGET_OUT_VENDOR)/lib/firmware
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE := w1u_bt_fw_usb
LOCAL_SRC_FILES := ../w1u_driver_bt/w1u_bt_fw_usb.bin
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_SUFFIX := .bin
LOCAL_MODULE_PATH  := $(TARGET_OUT_VENDOR)/lib/firmware
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE := w2_bt_fw_uart
LOCAL_SRC_FILES := ../usb_driver_bt/w2_bt_fw_uart.bin
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_SUFFIX := .bin
LOCAL_MODULE_PATH  := $(TARGET_OUT_VENDOR)/lib/firmware
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE := w2_bt_fw_usb
LOCAL_SRC_FILES := ../usb_driver_bt/w2_bt_fw_usb.bin
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_SUFFIX := .bin
LOCAL_MODULE_PATH  := $(TARGET_OUT_VENDOR)/lib/firmware
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE := w2l_bt_15p4_fw_uart
LOCAL_SRC_FILES := ../w2l_driver_bt/w2l_bt_15p4_fw_uart.bin
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_SUFFIX := .bin
LOCAL_MODULE_PATH  := $(TARGET_OUT_VENDOR)/lib/firmware
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE := w2l_bt_15p4_fw_usb
LOCAL_SRC_FILES := ../w2l_driver_bt/w2l_bt_15p4_fw_usb.bin
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_SUFFIX := .bin
LOCAL_MODULE_PATH  := $(TARGET_OUT_VENDOR)/lib/firmware
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)
endif

endif
endif
