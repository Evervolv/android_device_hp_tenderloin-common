/*
 * This is a userspace power management driver for the digitizer in the HP
 * Touchpad to turn the digitizer on and off.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 *
 * Copyright (c) 2012 CyanogenMod Touchpad Project.
 *
 *
 */

#ifndef ANDROID_TOUCHSCREEN_DIGITIZER_H
#define ANDROID_TOUCHSCREEN_DIGITIZER_H

// Maximum number of times to retry powering on the digitizer
#define MAX_DIGITIZER_RETRY 3

void digitizer_power(int enable);
void digitizer_init(void);

#endif // ANDROID_TOUCHSCREEN_DIGITIZER_H
