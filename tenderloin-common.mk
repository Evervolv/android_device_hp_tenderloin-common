DEVICE_PACKAGE_OVERLAYS += device/hp/tenderloin-common/overlay

# Permissions
PRODUCT_COPY_FILES += \
    frameworks/native/data/etc/android.hardware.bluetooth.xml:system/etc/permissions/android.hardware.bluetooth.xml \
    frameworks/native/data/etc/android.hardware.bluetooth_le.xml:system/etc/permissions/android.hardware.bluetooth_le.xml

# Aapt
PRODUCT_CHARACTERISTICS := tablet
PRODUCT_AAPT_CONFIG := xlarge mdpi
PRODUCT_AAPT_PREF_CONFIG := mdpi

#Art
PRODUCT_PROPERTY_OVERRIDES += \
    dalvik.vm.heapstartsize=8m \
    dalvik.vm.heapgrowthlimit=192m \
    dalvik.vm.heapsize=256m \
    dalvik.vm.heaptargetutilization=0.75 \
    dalvik.vm.heapminfree=512k \
    dalvik.vm.heapmaxfree=8m

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
   sound_trigger.primary.tenderloin \
   libsrec_jni \
   libavextensions \
   libavmediaextentions

PRODUCT_PACKAGES += \
    android.hardware.audio@2.0-impl \
    android.hardware.audio.effect@2.0-impl \
    android.hardware.soundtrigger@2.0-impl

# Audio config
PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/configs/mixer_paths.xml:system/etc/mixer_paths.xml \
    device/hp/tenderloin-common/configs/audio_policy.conf:system/etc/audio_policy.conf

# Bluetooth
PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/bluetooth/bt_vendor.conf:/system/etc/bluetooth/bt_vendor.conf \
    device/hp/tenderloin-common/bluetooth/bluecore6.psr:/system/etc/bluetooth/bluecore6.psr

PRODUCT_PACKAGES += \
    android.hardware.bluetooth@1.0-impl \
    libbt-vendor

# Camera
PRODUCT_PACKAGES += \
    android.hardware.camera.device@3.2-impl \
    android.hardware.camera.provider@2.4-impl \
    camera.msm8660

# Display
PRODUCT_PACKAGES += \
    copybit.msm8660 \
    gralloc.msm8660 \
    hwcomposer.msm8660 \
    libgenlock \
    memtrack.msm8660

PRODUCT_PROPERTY_OVERRIDES += debug.hwui.use_buffer_age=false

PRODUCT_PACKAGES += \
    android.hardware.graphics.allocator@2.0-impl \
    android.hardware.graphics.allocator@2.0-service \
    android.hardware.graphics.composer@2.1-impl \
    android.hardware.graphics.mapper@2.0-impl

# Drm
PRODUCT_PACKAGES += \
    android.hardware.drm@1.0-impl

# Filesystem management tools
PRODUCT_PACKAGES += \
    fsck.f2fs \
    resize2fs_static

#GNSS HAL
PRODUCT_PACKAGES += \
    android.hardware.gnss@1.0-impl

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

# Keymaster HAL
PRODUCT_PACKAGES += \
    android.hardware.keymaster@3.0-impl

# Lights
PRODUCT_PACKAGES += \
    lights.tenderloin \
    android.hardware.light@2.0-impl

# Low-RAM optimizations
PRODUCT_PROPERTY_OVERRIDES += \
    ro.config.low_ram=true \
    persist.sys.force_highendgfx=true \
    dalvik.vm.jit.codecachesize=0 \
    ro.config.max_starting_bg=4 \
    ro.sys.fw.bg_apps_limit=8 \
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

# Memtrack HAL
PRODUCT_PACKAGES += \
    android.hardware.memtrack@1.0-impl

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

# OMX properties
PRODUCT_PROPERTY_OVERRIDES += \
    persist.media.treble_omx=false

# Prebuilts
PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/prebuilt/tptoolbox.cfg:tptoolbox.cfg \
    device/hp/tenderloin-common/prebuilt/boot/moboot.splash.Evervolv.tga:moboot.splash.Evervolv.tga \
    device/hp/tenderloin-common/prebuilt/boot/moboot.default:moboot.default \
    device/hp/tenderloin-common/prebuilt/lvm/lvm.conf:root/lvm/lvm.conf \

# Power
PRODUCT_PACKAGES += \
    android.hardware.power@1.0-impl \
    power.tenderloin

# Recovery
PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/releasetools/install-recovery.sh:$(PRODUCT_OUT)/ota_temp/SYSTEM/bin/install-recovery.sh

# RenderScript HAL
PRODUCT_PACKAGES += \
    android.hardware.renderscript@1.0-impl

# Stlport
#PRODUCT_PACKAGES += \
#   libstlport

# Sensors
PRODUCT_PACKAGES += \
    sensors.tenderloin

# Sensor HAL
PRODUCT_PACKAGES += \
    android.hardware.sensors@1.0-impl

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
    ro.opengles.version=131072 \
    debug.sf.disable_backpressure=1

# Tools
PRODUCT_PACKAGES += \
    dosfsck \
    librs_jni \
    libmllite \
    libmlplatform \
    ts_srv \
    ts_srv_set \
    mkbootimg

# Vendor Interface Manifest
PRODUCT_COPY_FILES += \
    device/hp/tenderloin-common/manifest.xml:system/vendor/manifest.xml

# Vibrator
PRODUCT_PACKAGES += \
    android.hardware.vibrator@1.0-impl

# Wifi
PRODUCT_PACKAGES += \
    android.hardware.wifi@1.0-service \
    hostapd \
    hostapd_default.conf \
    libnetcmdiface \
    libwpa_client \
    wpa_supplicant \
    wificond \
    wifilogd \
    wpa_supplicant.conf
