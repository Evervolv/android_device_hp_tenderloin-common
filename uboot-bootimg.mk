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

#
# Bootimage
#
UBOOT_RAMDISK_NAME := $(TARGET_DEVICE) $(PLATFORM_VERSION) Ramdisk
UBOOT_RAMDISK_ARGS := -A ARM -O Linux -T RAMDisk -C none -n "$(UBOOT_RAMDISK_NAME)" -d $(BUILT_RAMDISK_TARGET)
UBOOT_RAMDISK_TARGET := $(BUILT_RAMDISK_TARGET:%.img=%.ub)

$(UBOOT_RAMDISK_TARGET): $(INSTALLED_RAMDISK_TARGET) $(MKIMAGE)
	$(MKIMAGE) $(UBOOT_RAMDISK_ARGS) $@
	@echo "Made boot ramdisk  $@"

INSTALLED_RAMDISK_TARGET := $(UBOOT_RAMDISK_TARGET)

INSTALLED_BOOTIMAGE_TARGET := $(PRODUCT_OUT)/boot.img

UBOOT_MULTIBOOT_NAME := $(TARGET_DEVICE) $(PLATFORM_VERSION) Multiboot
UBOOT_MULTIBOOT_ARGS := -A ARM -O Linux -T multi -C none -n "$(INTERNAL_MULTIBOOT_NAME)"

BOARD_UBOOT_ENTRY := $(strip $(BOARD_UBOOT_ENTRY))
ifdef BOARD_UBOOT_ENTRY
    UBOOT_MULTIBOOT_ARGS += -e $(BOARD_UBOOT_ENTRY)
endif

BOARD_UBOOT_LOAD := $(strip $(BOARD_UBOOT_LOAD))
ifdef BOARD_UBOOT_LOAD
    UBOOT_MULTIBOOT_ARGS += -a $(BOARD_UBOOT_LOAD)
endif

UBOOT_MULTIBOOT_ARGS += -d $(INSTALLED_KERNEL_TARGET):$(UBOOT_RAMDISK_TARGET)

ifeq ($(BOARD_USES_UBOOT_MULTIIMAGE),true)
$(INSTALLED_BOOTIMAGE_TARGET): $(MKIMAGE) $(INTERNAL_RAMDISK_FILES) $(UBOOT_RAMDISK_TARGET) $(INSTALLED_KERNEL_TARGET)
	$(MKIMAGE) $(UBOOT_MULTIBOOT_ARGS) $@
	@echo "Made multiboot uImage  $@"
else
$(INSTALLED_BOOTIMAGE_TARGET): $(BUILT_UBOOT_RAMDISK_TARGET)
	@echo "Made boot uImage  $@"
endif

#
# Recovery Image
#
recovery_ramdisk := $(PRODUCT_OUT)/ramdisk-recovery.img

UBOOT_RECOVERY_NAME := $(TARGET_DEVICE) Recovery Ramdisk
UBOOT_RECOVERY_ARGS := -A ARM -O Linux -T RAMDisk -C none -n "$(UBOOT_RECOVERY_NAME)" -d $(recovery_ramdisk)
UBOOT_RECOVERY_TARGET := $(recovery_ramdisk:%.img=%.ub)

$(UBOOT_RECOVERY_TARGET): $(MKIMAGE) $(recovery_ramdisk)
	$(MKIMAGE) $(UBOOT_RECOVERY_ARGS) $@
	@echo "Made recovery ramdisk  $@"

INSTALLED_RECOVERYIMAGE_TARGET := $(PRODUCT_OUT)/recovery.img

UBOOT_RECOVERY_MULTIBOOT_NAME := $(TARGET_DEVICE) Recovery Multiboot
UBOOT_RECOVERY_MULTIBOOT_ARGS := -A arm -T multi -C none -n "$(UBOOT_RECOVERY_MULTIBOOT_NAME)"

BOARD_UBOOT_ENTRY := $(strip $(BOARD_UBOOT_ENTRY))
ifdef BOARD_UBOOT_ENTRY
    UBOOT_RECOVERY_MULTIBOOT_ARGS += -e $(BOARD_UBOOT_ENTRY)
endif

UBOOT_RECOVERY_MULTIBOOT_ARGS += -d $(strip $(INSTALLED_KERNEL_TARGET)):$(strip $(UBOOT_RECOVERY_TARGET))

ifeq ($(BOARD_USES_UBOOT_MULTIIMAGE),true)
$(INSTALLED_RECOVERYIMAGE_TARGET): $(MKIMAGE) $(UBOOT_RECOVERY_TARGET) $(INSTALLED_KERNEL_TARGET)
	$(MKIMAGE) $(UBOOT_RECOVERY_MULTIBOOT_ARGS) $@
	@echo "Made multiboot recovery uImage  $@"
else #!BOARD_USES_UBOOT_MULTIIMAGE
    # If we are not on a multiimage platform lets zip the kernel with the ramdisk
    # for Rom Manager
$(INSTALLED_RECOVERYIMAGE_TARGET): $(UBOOT_RECOVERY_TARGET) $(INSTALLED_KERNEL_TARGET)
	$(hide) rm -f $@
	zip -qDj $@ $(UBOOT_RECOVERY_TARGET) $(INSTALLED_KERNEL_TARGET)
	@echo ----- Made recovery image \(zip\) -------- $@
	@echo "Made recovery uImage \(zip\) $@"
endif
