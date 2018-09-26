LOCAL_PATH:= $(call my-dir)

common_includes := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
common_deps := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
common_cflags := -Wall -funsafe-math-optimizations -D_POSIX_SOURCE

include $(CLEAR_VARS)
#
## TP Application
#
#
LOCAL_MODULE:= ts_srv
LOCAL_MODULE_TAGS := optional eng

LOCAL_SRC_FILES:= ts_srv.c digitizer.c
LOCAL_CFLAGS := $(common_cflags)
LOCAL_C_INCLUDES := $(common_includes)
LOCAL_ADDITIONAL_DEPENDENCIES := $(common_deps)

ifeq ($(RECOVERY_BUILD),)
LOCAL_SHARED_LIBRARIES := liblog
else
LOCAL_FORCE_STATIC_EXECUTABLE := true
LOCAL_MODULE_PATH := $(TARGET_ROOT_OUT_SBIN)
LOCAL_UNSTRIPPED_PATH := $(TARGET_ROOT_OUT_SBIN_UNSTRIPPED)
LOCAL_STATIC_LIBRARIES := \
    liblog \
    libcutils \
    libc

# libc++ not available on windows yet
ifneq ($(HOST_OS),windows)
    LOCAL_CXX_STL := libc++_static
endif
LOCAL_LDLIBS := -L$(SYSROOT)/usr/lib -llog
endif

include $(BUILD_EXECUTABLE)

## ts_srv_set application for changing modes of touchscreen operation
## used to set finger or stylus mode
include $(CLEAR_VARS)

LOCAL_MODULE := ts_srv_set
LOCAL_MODULE_TAGS := optional eng

LOCAL_SRC_FILES := ts_srv_set.c
LOCAL_CFLAGS := $(common_cflags)
LOCAL_C_INCLUDES := $(common_includes)
LOCAL_ADDITIONAL_DEPENDENCIES := $(common_deps)

LOCAL_SHARED_LIBRARIES := liblog

include $(BUILD_EXECUTABLE)
