DEVICE_PACKAGE_OVERLAYS += device/hp/tenderloin-common/overlay

PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/fstab.tenderloin:root/fstab.tenderloin

# Add touchcreen configuration file
PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/HPTouchpad.idc:system/usr/idc/HPTouchpad.idc

# Dualboot Magic
PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/moboot_control.sh:system/bin/moboot_control.sh

# Bluetooth
PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/bluetooth/bt_vendor.conf:/system/etc/bluetooth/bt_vendor.conf \
    device/hp/tenderloin-common/bluetooth/bluecore6.psr:/system/etc/bluetooth/bluecore6.psr

PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/prebuilt/wifi/wpa_supplicant.conf:system/etc/wifi/wpa_supplicant.conf \
    device/hp/tenderloin-common/prebuilt/wifi/wpa_supplicant_overlay.conf:system/etc/wifi/wpa_supplicant_overlay.conf  \
    device/hp/tenderloin-common/prebuilt/lvm/lvm.conf:root/lvm/lvm.conf \
    device/hp/tenderloin-common/prebuilt/boot/moboot.splash.Evervolv.tga:moboot.splash.Evervolv.tga \
    device/hp/tenderloin-common/prebuilt/tptoolbox.cfg:tptoolbox.cfg \
    device/hp/tenderloin-common/prebuilt/boot/moboot.default:moboot.default

# media minor check boot script
PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/prebuilt/etc/init.d/10check_media_minor:system/etc/init.d/10check_media_minor

# Codecs
PRODUCT_COPY_FILES += \
    frameworks/av/media/libstagefright/data/media_codecs_google_audio.xml:system/etc/media_codecs_google_audio.xml \
    frameworks/av/media/libstagefright/data/media_codecs_google_telephony.xml:system/etc/media_codecs_google_telephony.xml \
    frameworks/av/media/libstagefright/data/media_codecs_google_video.xml:system/etc/media_codecs_google_video.xml

# Keylayouts
PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/prebuilt/usr/keylayout/Generic.kl:system/usr/keylayout/Generic.kl \
    device/hp/tenderloin-common/prebuilt/usr/keylayout/pmic8058_pwrkey.kl:system/usr/keylayout/pmic8058_pwrkey.kl

#Wifi Firmware
# from kernel.org
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
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw1.0/bdata.SD32.bin:/system/etc/firmware/ath6k/AR6003/hw1.0/bdata.CUSTOM.bin

# Wifi Firmware
# Temporary hack for ATH6KL
PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/prebuilt/wifi/ath6k/AR6003/hw2.1.1/bdata.SD32.bin:/system/etc/firmware/ath6k/AR6003/hw2.1.1/bdata.bin

# Audio xml Files
PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/configs/mixer_paths.xml:system/etc/mixer_paths.xml \
    device/hp/tenderloin-common/configs/media_profiles.xml:system/etc/media_profiles.xml \
    device/hp/tenderloin-common/configs/media_codecs.xml:system/etc/media_codecs.xml 

# Audio Policy Configuration
PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/configs/audio_policy.conf:system/etc/audio_policy.conf

# To be inline with Build repo
PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/releasetools/install-recovery.sh:$(PRODUCT_OUT)/ota_temp/SYSTEM/bin/install-recovery.sh

# We have enough storage space to hold precise GC data
PRODUCT_TAGS += dalvik.gc.type-precise

# This is a tablet.
PRODUCT_CHARACTERISTICS := tablet
PRODUCT_AAPT_CONFIG := xlarge mdpi
PRODUCT_AAPT_PREF_CONFIG := mdpi

# BoringSSL compatability wrapper
PRODUCT_PACKAGES += \
    libboringssl-compat

# QCOM Hal
PRODUCT_PACKAGES += \
    copybit.msm8660 \
    hwcomposer.msm8660 \
    gralloc.msm8660 \
    liboverlay \
    libmemalloc \
    libtilerenderer \
    libgenlock \
    libQcomUI

# QCOM OMX
PRODUCT_PACKAGES += \
    libstagefrighthw \
    libstagefright_soft_ffmpegvdec \
    libOmxCore \
    libmm-omxcore \
    libdivxdrmdecrypt \
    libOmxVdec \
    libOmxVenc \
    libOmxAacEnc \
    libOmxAmrEnc

# Camera
PRODUCT_PACKAGES += \
   camera.msm8660

# F2FS filesystem
PRODUCT_PACKAGES += \
   mkfs.f2fs \
   fsck.f2fs \
   fibmap.f2fs \
   f2fstat*

# Audio
PRODUCT_PACKAGES += \
   audio.a2dp.default \
   audio.primary.tenderloin \
   audio.r_submix.default \
   libsrec_jni \
   libavextensions \
   libavmediaextentions

# Sensors, misc
PRODUCT_PACKAGES += \
    librs_jni \
    libnetcmdiface \
    libmllite \
    libmlplatform \
    power.tenderloin \
    sensors.tenderloin \
    lights.tenderloin \
    memtrack.msm8660 \
    ts_srv \
    ts_srv_set \
    dosfsck \
    serial \
    com.android.future.usb.accessory \
    rebootcmd

# Wifi
PRODUCT_PACKAGES += \
    dhcpcd.conf \
    libwpa_client \
    wpa_supplicant \
    wpa_supplicant.conf

PRODUCT_PACKAGES += \
	make_ext4fs

# Display
PRODUCT_PACKAGES += \
	libstlport \
	libstlport_static

# Property overrides.
PRODUCT_PROPERTY_OVERRIDES += \
    ro.sf.lcd_density=160 \
    ro.com.google.networklocation=1 \
    dalvik.vm.dex2oat-swap=false \
    dalvik.vm.lockprof.threshold=500 \
    dalvik.vm.dexopt-flags=m=y \
    dalvik.vm.dex2oat-flags=--no-watch-dog \
    dalvik.vm.image-dex2oat-filter=speed
