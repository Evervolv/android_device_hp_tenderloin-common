ifeq ($(TARGET_BOOTLOADER_BOARD_NAME),tenderloin)

LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE       := lvm.static
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_PATH  := $(TARGET_ROOT_OUT_SBIN)
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_SRC_FILES    := lvm.static
include $(BUILD_PREBUILT)

endif
