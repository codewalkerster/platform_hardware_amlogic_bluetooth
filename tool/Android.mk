LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE := btd
LOCAL_SRC_FILES := btd
LOCAL_MODULE_CLASS := ETC
#LOCAL_MODULE_SUFFIX := .bin
LOCAL_MODULE_PATH  := $(TARGET_OUT_VENDOR)/xbin
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE := btiw
LOCAL_SRC_FILES := btiw
LOCAL_MODULE_CLASS := ETC
#LOCAL_MODULE_SUFFIX := .bin
LOCAL_MODULE_PATH  := $(TARGET_OUT_VENDOR)/xbin
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)