#
# Copyright (C) 2013 The Android Open-Source Project
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

# Audio
PRODUCT_PROPERTY_OVERRIDES += \
    qcom.hw.aac.encoder=true

# Camera
PRODUCT_PROPERTY_OVERRIDES += \
    camera2.portability.force_api=1

# Dex2oat
PRODUCT_PROPERTY_OVERRIDES += \
    dalvik.vm.dex2oat-flags=--no-watch-dog \
    dalvik.vm.dex2oat-swap=false \
    dalvik.vm.image-dex2oat-filter=speed

# Display
PRODUCT_PROPERTY_OVERRIDES += \
    debug.composition.type=dyn \
    debug.hwui.use_buffer_age=false \
    debug.sf.disable_backpressure=1 \
    persist.hwc.mdpcomp.enable=false \
    ro.opengles.version=131072 \
    ro.sf.lcd_density=160

# GPS
PRODUCT_PROPERTY_OVERRIDES += \
    ro.com.google.networklocation=1

# Low ram
PRODUCT_PROPERTY_OVERRIDES += \
    persist.sys.force_highendgfx=true \
    dalvik.vm.jit.codecachesize=0 \
    ro.config.max_starting_bg=4 \
    ro.sys.fw.bg_apps_limit=8 \
    ro.sys.fw.use_trim_settings=true \
    ro.sys.fw.empty_app_percent=50 \
    ro.sys.fw.trim_empty_percent=100 \
    ro.sys.fw.trim_cache_percent=100 \
    ro.sys.fw.trim_enable_memory=874512384 \
    ro.vendor.qti.sys.fw.bservice_enable=true \
    ro.vendor.qti.sys.fw.bservice_limit=5 \
    ro.vendor.qti.sys.fw.bservice_age=5000

# Media
PRODUCT_PROPERTY_OVERRIDES += \
    media.stagefright.legacyencoder=true \
    media.stagefright.less-secure=true

# Scheduler
PRODUCT_PROPERTY_OVERRIDES += \
    sys.io.scheduler=bfq
