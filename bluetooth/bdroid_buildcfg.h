/*
 * Copyright (C) 2012 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _BDROID_BUILDCFG_H
#define _BDROID_BUILDCFG_H

#define BTM_DEF_LOCAL_NAME   "HP Touchpad"

// Networking, Capturing, Object Transfer
// Major Class: COMPUTER
// Minor Class: LAPTOP
#define BTA_DM_COD {0x1A, 0x01, 0x0C}

#define BTM_BYPASS_EXTRA_ACL_SETUP TRUE
#define LE_L2CAP_CFC_INCLUDED FALSE
#define REMOVE_EAGER_THREADS FALSE
#define TEST_APP_INTERFACE FALSE
#define KERNEL_MISSING_CLOCK_BOOTTIME_ALARM TRUE
#define BTM_SCO_ENHANCED_SYNC_DISABLED TRUE
#define LEGACY_BT TRUE
#endif
