LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE_RELATIVE_PATH := hw
LOCAL_MODULE := camera.$(TARGET_BOARD_PLATFORM)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_OWNER := qcom
LOCAL_PROPRIETARY_MODULE := true

LOCAL_SRC_FILES := \
    QualcommCameraHardware.cpp \
    QCameraParameters.cpp \
    cameraHAL.cpp

LOCAL_CFLAGS := \
    -DDLOPEN_LIBMMCAMERA=1 \
    -DHW_ENCODE \
    -DNUM_PREVIEW_BUFFERS=4 \
    -D_ANDROID_ \
    -DUSE_NEON_CONVERSION \
    -DUSE_ION

ifeq ($(BOARD_DEBUG_MEMLEAKS),true)
    LOCAL_CFLAGS += -DHEAPTRACKER
endif

LOCAL_C_INCLUDES :=  \
    frameworks/base/include \
    $(TARGET_OUT_HEADERS)/mm-camera \
    $(TARGET_OUT_HEADERS)/mm-still/jpeg \
    $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include \
    system/media/camera/include

LOCAL_ADDITIONAL_DEPENDENCIES := \
    $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_C_INCLUDES +=  \
    $(call project-path-for,qcom-display)/libgralloc \
    $(call project-path-for,qcom-display)/libgenlock

LOCAL_SHARED_LIBRARIES := \
    libutils \
    libgui \
    libui \
    libcamera_client \
    liblog \
    libcutils \
    libgenlock \
    libbinder \
    libdl \
    libhardware \
    libstagefrighthw

ifeq ($(BOARD_DEBUG_MEMLEAKS),true)
    LOCAL_SHARED_LIBRARIES += libheaptracker
endif

LOCAL_32_BIT_ONLY := true
include $(BUILD_SHARED_LIBRARY)
