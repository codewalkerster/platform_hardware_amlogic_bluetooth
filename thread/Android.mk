ifneq (,$(findstring 64,$(TARGET_ARCH)))

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_SYSTEM_EXT_MODULE := true
#LOCAL_ALLOW_UNDEFINED_SYMBOLS := true
#LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE := otbr-agent-aml
LOCAL_SRC_FILES := 64bit/otbr-agent-aml
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_MODULE_SUFFIX :=
LOCAL_MODULE_PATH  :=$(TARGET_OUT_SYSTEM_EXT)/bin
LOCAL_SHARED_LIBRARIES := libbase libcutils libutils libmdnssd libc++
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_SYSTEM_EXT_MODULE := true
#LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE := ot-ctl-aml
LOCAL_SRC_FILES := 64bit/ot-ctl-aml
$(info Building for 32-bit architecture ot-ctl-aml)
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_MODULE_SUFFIX :=
LOCAL_MODULE_PATH  :=$(TARGET_OUT_SYSTEM_EXT)/bin
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

else

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
LOCAL_SRC_FILES_32 := 32bit/thread_ncp_aml.so
LOCAL_MULTILIB := 32
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_SUFFIX := .so
LOCAL_MODULE_PATH_32 := $(LIB_PATH_32)
LOCAL_CHECK_ELF_FILES := false
LOCAL_MODULE_TAGS := optional
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE := libmnl_aml
LOCAL_SRC_FILES_32 := 32bit/libmnl_aml.so
LOCAL_MULTILIB := 32
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_SUFFIX := .so
LOCAL_MODULE_PATH_32 := $(LIB_PATH_32)
LOCAL_CHECK_ELF_FILES := false
LOCAL_MODULE_TAGS := optional
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE := libnetfilter_queue_aml
LOCAL_SRC_FILES_32 := 32bit/libnetfilter_queue_aml.so
LOCAL_MULTILIB := 32
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_SUFFIX := .so
LOCAL_MODULE_PATH_32 := $(LIB_PATH_32)
LOCAL_CHECK_ELF_FILES := false
LOCAL_MODULE_TAGS := optional
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE := libnfnetlink_aml
LOCAL_SRC_FILES_32 := 32bit/libnfnetlink_aml.so
LOCAL_MULTILIB := 32
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_SUFFIX := .so
LOCAL_MODULE_PATH_32 := $(LIB_PATH_32)
LOCAL_CHECK_ELF_FILES := false
LOCAL_MODULE_TAGS := optional
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE := libmdnssd_aml
LOCAL_SRC_FILES_32 := 32bit/libmdnssd_aml.so
LOCAL_MULTILIB := 32
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_SUFFIX := .so
LOCAL_MODULE_PATH_32 := $(LIB_PATH_32)
LOCAL_CHECK_ELF_FILES := false
LOCAL_MODULE_TAGS := optional
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE := thread_otbr_aml
LOCAL_SRC_FILES_32 := 32bit/thread_otbr_aml.so
LOCAL_MULTILIB := 32
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_SUFFIX := .so
LOCAL_MODULE_PATH_32 := $(LIB_PATH_32)
LOCAL_CHECK_ELF_FILES := false
LOCAL_MODULE_TAGS := optional
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE := libprotobuf-cpp-lite_aml
LOCAL_SRC_FILES_32 := 32bit/libprotobuf-cpp-lite_aml.so
LOCAL_MULTILIB := 32
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_SUFFIX := .so
LOCAL_MODULE_PATH_32 := $(LIB_PATH_32)
LOCAL_CHECK_ELF_FILES := false
LOCAL_MODULE_TAGS := optional
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE := otbr-agent-aml
LOCAL_SRC_FILES := 32bit/otbr-agent-aml
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_SUFFIX :=
LOCAL_MODULE_PATH  := $(TARGET_OUT_VENDOR)/bin
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE := ot-ctl-aml
LOCAL_SRC_FILES := 32bit/ot-ctl-aml
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_SUFFIX :=
LOCAL_MODULE_PATH  := $(TARGET_OUT_VENDOR)/bin
LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only
include $(BUILD_PREBUILT)

endif
