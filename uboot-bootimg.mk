#
# Copyright (C) 2011 The Cyanogenmod Project
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

ifeq ($(BOARD_USES_UBOOT),true)
ifeq ($(BOARD_USES_UBOOT_MULTIIMAGE),true)

image_name := $(TARGET_DEVICE) $(PLATFORM_VERSION)
image_name_ramdisk := $(image_name) Ramdisk
image_name_boot := $(image_name) Android
image_name_recovery := $(image_name) Recovery

recovery_ramdisk := $(PRODUCT_OUT)/ramdisk-recovery.img
INSTALLED_RECOVERYIMAGE_TARGET := $(PRODUCT_OUT)/recovery.img

# mkimage args
mkimage_ramdisk_args := -A ARM -O Linux -T RAMDisk -C none -n "$(image_name_ramdisk)"
mkimage_boot_args := -A ARM -O Linux -T multi -C none -n "$(image_name_boot)"
mkimage_recovery_args := -A arm -T multi -C none -n "$(image_name_recovery)"

#
# Boot
#
UBOOT_RAMDISK_TARGET := $(BUILT_RAMDISK_TARGET:%.img=%.ub)

$(UBOOT_RAMDISK_TARGET): $(MKIMAGE) $(BUILT_RAMDISK_TARGET)
	$(call pretty,"Target boot ramdisk: $@")
	$(hide) $(MKIMAGE) $(mkimage_ramdisk_args) -d $(BUILT_RAMDISK_TARGET) $@
	@echo "Made boot ramdisk: $@"

BOOTIMAGE_EXTRA_DEPS := $(UBOOT_RAMDISK_TARGET)

$(INSTALLED_BOOTIMAGE_TARGET): $(MKIMAGE) $(INTERNAL_BOOTIMAGE_FILES) $(BOOTIMAGE_EXTRA_DEPS)
	$(call pretty,"Target boot image: $@")
	$(hide) $(MKIMAGE) $(mkimage_boot_args) -d $(INSTALLED_KERNEL_TARGET):$(UBOOT_RAMDISK_TARGET) $@
	@echo "Made boot image: $@"

#
# Recovery
#
uboot_recovery_ramdisk := $(recovery_ramdisk:%.img=%.ub)

$(uboot_recovery_ramdisk): $(MKIMAGE) $(recovery_ramdisk)
	$(call pretty,"Target recovery ramdisk: $@")
	$(hide) $(MKIMAGE) $(mkimage_ramdisk_args) -d $(recovery_ramdisk) $@
	@echo "Made recovery ramdisk: $@"

RECOVERYIMAGE_EXTRA_DEPS += $(uboot_recovery_ramdisk)

$(INSTALLED_RECOVERYIMAGE_TARGET): $(MKIMAGE) $(recovery_ramdisk) $(recovery_kernel) $(RECOVERYIMAGE_EXTRA_DEPS)
	$(call pretty,"Target recovery image: $@")
	$(hide) $(MKIMAGE) $(mkimage_recovery_args) -d $(strip $(recovery_kernel)):$(strip $(uboot_recovery_ramdisk)) $@
	@echo "Made recovery image: $@"

endif
endif
