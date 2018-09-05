# Copyright (C) 2015 The Android Open-Source Project
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

LOCAL_PATH := $(call my-dir)

ifeq ($(TARGET_BOOTLOADER_BOARD_NAME),tenderloin)

INSTALLED_TOOLBOX_CONFIG := $(PRODUCT_OUT)/tptoolbox.cfg

$(INSTALLED_TOOLBOX_CONFIG): $(LOCAL_INSTALLED_MODULE)
	@echo "Creating toolbox config: $@"
	@echo 'TPTOOLBOX_CONFIG_VERSION 1' >> $@
	@echo 'FILETYPE ROM' >> $@
	@echo 'ROM_VERSION $(PLATFORM_VERSION)' >> $@
	@echo 'ROM_IS_DATAMEDIA 1' >> $@
	@echo 'ROM_BOOTIMAGE_NAME Android' >> $@
	@echo 'ROM_SYSTEM_SIZE 1312' >> $@

ALL_DEFAULT_INSTALLED_MODULES += $(INSTALLED_TOOLBOX_CONFIG)

INSTALLED_MOBOOT_CONFIG := $(PRODUCT_OUT)/moboot.default

$(INSTALLED_MOBOOT_CONFIG): $(LOCAL_INSTALLED_MODULE)
	@echo "Creating moboot config: $@"
	@echo 'Android' >> $@

ALL_DEFAULT_INSTALLED_MODULES += $(INSTALLED_MOBOOT_CONFIG)

include $(call all-makefiles-under,$(LOCAL_PATH))

endif
