LOCAL_PATH:= $(call my-dir)
kernel_includes += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include

include $(CLEAR_VARS)
#
## TP Application
#
#
#LOCAL_C_INCLUDES:= uim.h
LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
LOCAL_SRC_FILES:= \
	ts_srv.c \
	digitizer.c
LOCAL_CFLAGS:= -g -c -W -Wall -O2 -mtune=cortex-a9 -mfpu=neon -mfloat-abi=softfp -funsafe-math-optimizations -D_POSIX_SOURCE
LOCAL_C_INCLUDES:= $(kernel_includes)
LOCAL_MODULE:= ts_srv
LOCAL_MODULE_TAGS:= eng
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
endif
LOCAL_LDLIBS := -L$(SYSROOT)/usr/lib -llog
include $(BUILD_EXECUTABLE)


## ts_srv_set application for changing modes of touchscreen operation
## used to set finger or stylus mode
include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	ts_srv_set.c
LOCAL_CFLAGS:= -g -c -W -Wall -O2 -mtune=cortex-a9 -mfpu=neon -mfloat-abi=softfp -funsafe-math-optimizations -D_POSIX_SOURCE
LOCAL_C_INCLUDES:= $(kernel_includes)
LOCAL_MODULE:=ts_srv_set
LOCAL_MODULE_TAGS:= eng
LOCAL_SHARED_LIBRARIES := liblog

LOCAL_LDLIBS := -L$(SYSROOT)/usr/lib -llog
include $(BUILD_EXECUTABLE)
