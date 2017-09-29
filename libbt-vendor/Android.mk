#
# Copyright 2012 The Android Open Source Project
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
#
ifeq ($(BOARD_HAVE_BLUETOOTH_HCI),true)

LOCAL_PATH := $(call my-dir)
kernel_includes += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include

include $(CLEAR_VARS)
LOCAL_CPP_EXTENSION := .cc

LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
BDROID_DIR:= system/bt

LOCAL_SRC_FILES := \
        src/bt_vendor_hci.cc \
		src/conf.cc \
        src/hardware.cc \
		src/upio.cc \
		src/ubcsp.cc \
        src/userial_vendor.cc

LOCAL_C_INCLUDES += \
        $(LOCAL_PATH)/include \
        $(BDROID_DIR)/hci/include \
	$(kernel_includes)

LOCAL_SHARED_LIBRARIES := \
        libcutils \
        libbase \
        liblog

LOCAL_STATIC_LIBRARIES := libosi

ifeq ($(TARGET_BOOTLOADER_BOARD_NAME), tenderloin)
	LOCAL_CFLAGS := -DHW_TENDERLOIN
endif

LOCAL_MODULE := libbt-vendor
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_OWNER := hci
LOCAL_MODULE_PATH := $(TARGET_OUT_VENDOR_SHARED_LIBRARIES)

include $(LOCAL_PATH)/vnd_buildcfg.mk

include $(BUILD_SHARED_LIBRARY)

endif
