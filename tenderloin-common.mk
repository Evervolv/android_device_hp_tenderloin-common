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
    init.tenderloin.power.rc \
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
PRODUCT_PACKAGES += \
   camera.msm8660

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

# IO Scheduler
PRODUCT_PROPERTY_OVERRIDES += \
    sys.io.scheduler=bfq

# Keylayouts
PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/prebuilt/usr/idc/HPTouchpad.idc:system/usr/idc/HPTouchpad.idc \
    device/hp/tenderloin-common/prebuilt/usr/keylayout/Generic.kl:system/usr/keylayout/Generic.kl \
    device/hp/tenderloin-common/prebuilt/usr/keylayout/pmic8058_pwrkey.kl:system/usr/keylayout/pmic8058_pwrkey.kl

# Lights
PRODUCT_PACKAGES += \
    lights.tenderloin

# Low-RAM optimizations
PRODUCT_PROPERTY_OVERRIDES += \
    ro.config.low_ram=true \
    ro.config.max_starting_bg=8 \
    ro.sys.fw.bg_apps_limit=16 \
    ro.sys.fw.use_trim_settings=true \
    ro.sys.fw.empty_app_percent=50 \
    ro.sys.fw.trim_empty_percent=100 \
    ro.sys.fw.trim_cache_percent=100 \
    ro.sys.fw.trim_enable_memory=874512384 \
    ro.sys.fw.bservice_enable=true \
    ro.sys.fw.bservice_limit=5 \
    ro.sys.fw.bservice_age=5000

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
    dalvik.vm.image-dex2oat-filter=speed \
    ro.com.google.networklocation=1 \
    media.stagefright.legacyencoder=true \
    media.stagefright.less-secure=true \
    camera2.portability.force_api=1 \
    qcom.hw.aac.encoder=true \
    debug.composition.type=dyn \
    persist.hwc.mdpcomp.enable=false \
    ro.opengles.version=131072

# Tools
PRODUCT_PACKAGES += \
    dosfsck \
    librs_jni \
    libmllite \
    libmlplatform \
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
    libwifi-hal-ath6kl \
    wpa_supplicant.conf
