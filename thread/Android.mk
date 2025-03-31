LOCAL_PATH := $(call my-dir)

ifeq ($(shell test $(PLATFORM_SDK_VERSION) -ge 26 && echo OK),OK)
LIB_PATH_32 := $(TARGET_OUT_VENDOR)/lib/
LIB_PATH_64 := $(TARGET_OUT_VENDOR)/lib64/
else
LIB_PATH_32 := $(TARGET_OUT)/lib/
LIB_PATH_64 := $(TARGET_OUT)/lib64/
endif

include $(CLEAR_VARS)
LOCAL_MODULE := thread_ncp_aml
LOCAL_SRC_FILES_32 := thread_ncp_aml.so
LOCAL_MULTILIB := 32
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_SUFFIX := .so
LOCAL_MODULE_PATH_32 := $(LIB_PATH_32)
LOCAL_CHECK_ELF_FILES := false
LOCAL_MODULE_TAGS ：= optional
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE := libmnl
LOCAL_SRC_FILES_32 := libmnl.so
LOCAL_MULTILIB := 32
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_SUFFIX := .so
LOCAL_MODULE_PATH_32 := $(LIB_PATH_32)
LOCAL_CHECK_ELF_FILES := false
LOCAL_MODULE_TAGS ：= optional
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE := libnetfilter_queue
LOCAL_SRC_FILES_32 := libnetfilter_queue.so
LOCAL_MULTILIB := 32
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_SUFFIX := .so
LOCAL_MODULE_PATH_32 := $(LIB_PATH_32)
LOCAL_CHECK_ELF_FILES := false
LOCAL_MODULE_TAGS ：= optional
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE := thread_otbr_aml
LOCAL_SRC_FILES_32 := thread_otbr_aml.so
LOCAL_MULTILIB := 32
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_SUFFIX := .so
LOCAL_MODULE_PATH_32 := $(LIB_PATH_32)
LOCAL_CHECK_ELF_FILES := false
LOCAL_MODULE_TAGS ：= optional
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE := libprotobuf-cpp-lite_aml
LOCAL_SRC_FILES_32 := libprotobuf-cpp-lite_aml.so
LOCAL_MULTILIB := 32
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_SUFFIX := .so
LOCAL_MODULE_PATH_32 := $(LIB_PATH_32)
LOCAL_CHECK_ELF_FILES := false
LOCAL_MODULE_TAGS ：= optional
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE := threadbr_aml
LOCAL_SRC_FILES := threadbr_aml
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_SUFFIX :=
LOCAL_MODULE_PATH  := $(TARGET_OUT_VENDOR)/bin
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE := udpServer
LOCAL_SRC_FILES := udpServer
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_SUFFIX :=
LOCAL_MODULE_PATH  := $(TARGET_OUT_VENDOR)/bin
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

# include $(CLEAR_VARS)
# LOCAL_MODULE := libnfnetlink
# LOCAL_SRC_FILES_32 := libnfnetlink.so
# LOCAL_MULTILIB := 32
# LOCAL_MODULE_CLASS := SHARED_LIBRARIES
# LOCAL_MODULE_SUFFIX := .so
# LOCAL_MODULE_PATH_32 := $(LIB_PATH_32)
# LOCAL_CHECK_ELF_FILES := false
# LOCAL_MODULE_TAGS ：= optional
# include $(BUILD_PREBUILT)