DEVICE_PACKAGE_OVERLAYS += device/hp/tenderloin-common/overlay

# Permissions
PRODUCT_COPY_FILES += \
    frameworks/native/data/etc/android.hardware.bluetooth.xml:system/etc/permissions/android.hardware.bluetooth.xml \
    frameworks/native/data/etc/android.hardware.bluetooth_le.xml:system/etc/permissions/android.hardware.bluetooth_le.xml

# Aapt
PRODUCT_CHARACTERISTICS := tablet
PRODUCT_AAPT_CONFIG := xlarge mdpi
PRODUCT_AAPT_PREF_CONFIG := mdpi

# Bootloader
PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/moboot_control.sh:system/bin/moboot_control.sh

# Ramdisk
PRODUCT_PACKAGES += \
    fstab.tenderloin \
    init.tenderloin.rc \
    init.tenderloin.usb.rc \
    ueventd.tenderloin.rc

# Audio
PRODUCT_PACKAGES += \
   audio.a2dp.default \
   audio.primary.tenderloin \
   audio.r_submix.default \
   libsrec_jni \
   libavextensions \
   libavmediaextentions

# Audio config
PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/configs/mixer_paths.xml:system/etc/mixer_paths.xml \
    device/hp/tenderloin-common/configs/audio_policy.conf:system/etc/audio_policy.conf

# Bluetooth
PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/bluetooth/bt_vendor.conf:/system/etc/bluetooth/bt_vendor.conf \
    device/hp/tenderloin-common/bluetooth/bluecore6.psr:/system/etc/bluetooth/bluecore6.psr

# Camera
#PRODUCT_PACKAGES += \
#   camera.msm8660

# Display
PRODUCT_PACKAGES += \
    copybit.msm8660 \
    gralloc.msm8660 \
    hwcomposer.msm8660 \
    libgenlock \
    memtrack.msm8660

# Filesystem management tools
PRODUCT_PACKAGES += \
    fsck.f2fs \
    resize2fs_static

# Init.d
PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/prebuilt/etc/init.d/10check_media_minor:system/etc/init.d/10check_media_minor

# Keylayouts
PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/prebuilt/usr/idc/HPTouchpad.idc:system/usr/idc/HPTouchpad.idc \
    device/hp/tenderloin-common/prebuilt/usr/keylayout/Generic.kl:system/usr/keylayout/Generic.kl \
    device/hp/tenderloin-common/prebuilt/usr/keylayout/pmic8058_pwrkey.kl:system/usr/keylayout/pmic8058_pwrkey.kl

# Lights
PRODUCT_PACKAGES += \
    lights.tenderloin

# Media
PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/configs/media_profiles.xml:system/etc/media_profiles.xml \
    device/hp/tenderloin-common/configs/media_codecs.xml:system/etc/media_codecs.xml \
    frameworks/av/media/libstagefright/data/media_codecs_google_audio.xml:system/etc/media_codecs_google_audio.xml \
    frameworks/av/media/libstagefright/data/media_codecs_google_telephony.xml:system/etc/media_codecs_google_telephony.xml \
    frameworks/av/media/libstagefright/data/media_codecs_google_video.xml:system/etc/media_codecs_google_video.xml

# OMX
PRODUCT_PACKAGES += \
    libOmxCore \
    libOmxVdec \
    libOmxVenc \
    libOmxAacEnc \
    libOmxAmrEnc \
    libOmxEvrcEnc \
    libOmxQcelp13Enc \
    libstagefrighthw

# Prebuilts
PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/prebuilt/tptoolbox.cfg:tptoolbox.cfg \
    device/hp/tenderloin-common/prebuilt/boot/moboot.splash.Evervolv.tga:moboot.splash.Evervolv.tga \
    device/hp/tenderloin-common/prebuilt/boot/moboot.default:moboot.default \
    device/hp/tenderloin-common/prebuilt/lvm/lvm.conf:root/lvm/lvm.conf \

# Power
PRODUCT_PACKAGES += \
    power.tenderloin

# Recovery
PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/releasetools/install-recovery.sh:$(PRODUCT_OUT)/ota_temp/SYSTEM/bin/install-recovery.sh

# Stlport
PRODUCT_PACKAGES += \
    libstlport

# Sensors
PRODUCT_PACKAGES += \
    sensors.tenderloin

# System properties
PRODUCT_PROPERTY_OVERRIDES += \
    ro.sf.lcd_density=160 \
    dalvik.vm.dex2oat-flags=--no-watch-dog \
    dalvik.vm.dex2oat-swap=false \
    dalvik.vm.dexopt-flags=m=y \
    dalvik.vm.image-dex2oat-filter=speed \
    dalvik.vm.lockprof.threshold=500 \
    ro.com.google.networklocation=1 \
    qcom.bluetooth.soc=rome

# Tools
PRODUCT_PACKAGES += \
    dosfsck \
    librs_jni \
    libmllite \
    libmlplatform \
    serial \
    ts_srv \
    ts_srv_set \
    rebootcmd \
    mkbootimg

# Wifi
PRODUCT_PACKAGES += \
    dhcpcd.conf \
    hostapd \
    hostapd_default.conf \
    libnetcmdiface \
    libwpa_client \
    wpa_supplicant \
    wpa_supplicant.conf

PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/configs/wpa_supplicant.conf:system/etc/wifi/wpa_supplicant.conf \
    device/hp/tenderloin-common/configs/wpa_supplicant_overlay.conf:system/etc/wifi/wpa_supplicant_overlay.conf

PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw2.0/data.patch.bin:/system/etc/firmware/ath6k/AR6003/hw2.0/data.patch.bin \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw2.0/bdata.SD31.bin:/system/etc/firmware/ath6k/AR6003/hw2.0/bdata.SD31.bin \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw2.0/athwlan.bin.z77:/system/etc/firmware/ath6k/AR6003/hw2.0/athwlan.bin.z77 \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw2.0/bdata.WB31.bin:/system/etc/firmware/ath6k/AR6003/hw2.0/bdata.WB31.bin \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw2.0/otp.bin.z77:/system/etc/firmware/ath6k/AR6003/hw2.0/otp.bin.z77 \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw2.0/bdata.SD32.bin:/system/etc/firmware/ath6k/AR6003/hw2.0/bdata.SD32.bin \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw2.1.1/endpointping.bin:/system/etc/firmware/ath6k/AR6003/hw2.1.1/endpointping.bin \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw2.1.1/data.patch.bin:/system/etc/firmware/ath6k/AR6003/hw2.1.1/data.patch.bin \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw2.1.1/bdata.SD31.bin:/system/etc/firmware/ath6k/AR6003/hw2.1.1/bdata.SD31.bin \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw2.1.1/bdata.WB31.bin:/system/etc/firmware/ath6k/AR6003/hw2.1.1/bdata.WB31.bin \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw2.1.1/athwlan.bin:/system/etc/firmware/ath6k/AR6003/hw2.1.1/athwlan.bin \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw2.1.1/bdata.SD32.bin:/system/etc/firmware/ath6k/AR6003/hw2.1.1/bdata.SD32.bin \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw2.1.1/otp.bin:/system/etc/firmware/ath6k/AR6003/hw2.1.1/otp.bin \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw2.1.1/fw-3.bin:/system/etc/firmware/ath6k/AR6003/hw2.1.1/fw-3.bin \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw1.0/data.patch.bin:/system/etc/firmware/ath6k/AR6003/hw1.0/data.patch.bin \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw1.0/bdata.SD31.bin:/system/etc/firmware/ath6k/AR6003/hw1.0/bdata.SD31.bin \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw1.0/athwlan.bin.z77:/system/etc/firmware/ath6k/AR6003/hw1.0/athwlan.bin.z77 \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw1.0/bdata.WB31.bin:/system/etc/firmware/ath6k/AR6003/hw1.0/bdata.WB31.bin \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw1.0/otp.bin.z77:/system/etc/firmware/ath6k/AR6003/hw1.0/otp.bin.z77 \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw1.0/bdata.SD32.bin:/system/etc/firmware/ath6k/AR6003/hw1.0/bdata.SD32.bin \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6002/eeprom.data:/system/etc/firmware/ath6k/AR6002/eeprom.data \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6002/data.patch.hw2_0.bin:/system/etc/firmware/ath6k/AR6002/data.patch.hw2_0.bin \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6002/athwlan.bin.z77:/system/etc/firmware/ath6k/AR6002/athwlan.bin.z77 \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6002/eeprom.bin:/system/etc/firmware/ath6k/AR6002/eeprom.bin \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/LICENSE.atheros_firmware:/system/etc/firmware/ath6k/LICENSE.atheros_firmware \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw2.0/bdata.SD32.bin:/system/etc/firmware/ath6k/AR6003/hw2.0/bdata.CUSTOM.bin \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw2.1.1/bdata.SD32.bin:/system/etc/firmware/ath6k/AR6003/hw2.1.1/bdata.CUSTOM.bin \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw1.0/bdata.SD32.bin:/system/etc/firmware/ath6k/AR6003/hw1.0/bdata.CUSTOM.bin \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw2.1.1/bdata.SD32.bin:/system/etc/firmware/ath6k/AR6003/hw2.1.1/bdata.bin
