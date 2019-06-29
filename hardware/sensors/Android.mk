# Copyright (C) 2008 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


LOCAL_PATH := $(call my-dir)

# HAL module implemenation, not prelinked, and stored in
# hw/<SENSORS_HARDWARE_MODULE_ID>.<ro.product.board>.so
include $(CLEAR_VARS)

LOCAL_MODULE := sensors.tenderloin

LOCAL_MODULE_RELATIVE_PATH := hw
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE_TAGS := optional

LOCAL_CFLAGS := -DLOG_TAG=\"Sensors\"
LOCAL_SRC_FILES := \
    sensors.c \
    nusensors.cpp \
    InputEventReader.cpp \
    SensorBase.cpp \
    lsm303dlh_acc.cpp \
    lsm303dlh_mag.cpp \
    LightSensor.cpp \
    MPLSensor.cpp

LOCAL_CFLAGS += -DCONFIG_MPU_SENSORS_MPU3050=1

LOCAL_C_INCLUDES += \
    $(LOCAL_PATH)/../mlsdk/platform/include \
    $(LOCAL_PATH)/../mlsdk/platform/include/linux \
    $(LOCAL_PATH)/../mlsdk/platform/linux \
    $(LOCAL_PATH)/../mlsdk/mllite \
    $(LOCAL_PATH)/../mlsdk/mldmp \
    $(LOCAL_PATH)/../mlsdk/external/aichi \
    $(LOCAL_PATH)/../mlsdk/external/akmd

LOCAL_SHARED_LIBRARIES := liblog libcutils libutils libdl libmllite libmlplatform
LOCAL_CPPFLAGS+=-DLINUX=1
LOCAL_LDFLAGS:=-rdynamic

include $(BUILD_SHARED_LIBRARY)
