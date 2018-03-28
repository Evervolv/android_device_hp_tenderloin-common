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

#ifndef ANDROID_TOUCHSCREEN_CONFIG_H
#define ANDROID_TOUCHSCREEN_CONFIG_H

// This value determines when a large distance change between one touch
// and another will be reported as 2 separate touches instead of a swipe.
// This distance is in pixels.
#define MAX_DELTA 130

// If we exceed MAX_DELTA, we'll check the previous touch point to see if
// it was moving fairly far.  If the previous touch moved far enough and is
// within the same direction / angle, we'll allow it to be a swipe.
// This is the distance theshold that the previous touch must have traveled.
// This value is in pixels.
#define MIN_PREV_DELTA 40

// This is the angle, plus or minus that the previous direction must have
// been traveling.  This angle is an arctangent. (atan2)
#define MAX_DELTA_ANGLE 0.25

// Any touch above this threshold is immediately reported to the system
#define TOUCH_INITIAL_THRESHOLD 32
#define TOUCH_INITIAL_THRESHOLD_S 32

// Previous touches that have already been reported will continue to be
// reported so long as they stay above this threshold
#define TOUCH_CONTINUE_THRESHOLD 26
#define TOUCH_CONTINUE_THRESHOLD_S 16

// New touches above this threshold but below TOUCH_INITIAL_THRESHOLD will not
// be reported unless the touch continues to appear.  This is designed to
// filter out brief, low threshold touches that may not be valid.
#define TOUCH_DELAY_THRESHOLD 28
#define TOUCH_DELAY_THRESHOLD_S 24

// Delay before a touch above TOUCH_DELAY_THRESHOLD but below
// TOUCH_INITIAL_THRESHOLD will be reported.  We will wait and see if this
// touch continues to show up in future buffers before reporting the event.
#define TOUCH_DELAY 5
#define TOUCH_DELAY_S 2

// Threshold for end of a large area. This value needs to be set low enough
// to filter out large touch areas and tends to be related to other touch
// thresholds.
#define LARGE_AREA_UNPRESS 22 //TOUCH_CONTINUE_THRESHOLD
#define LARGE_AREA_FRINGE 5 // Threshold for large area fringe

// Enables filtering of a single touch to make it easier to long press.
// Keeps the initial touch point the same so long as it stays within
// the radius (note it's not really a radius and is actually a square)
#define DEBOUNCE_RADIUS 10 // Radius for debounce in pixels

// Enables filtering after swiping to prevent the slight jitter that
// sometimes happens while holding your finger still.  The radius is
// really a square. We don't start debouncing a hover unless the touch point
// stays within the radius for the number of cycles defined by
// HOVER_DEBOUNCE_DELAY
#define HOVER_DEBOUNCE_RADIUS 2 // Radius for hover debounce in pixels
#define HOVER_DEBOUNCE_DELAY 30 // Count of delay before we start debouncing

// This is used to help calculate ABS_TOUCH_MAJOR
// This is roughly the value of 1024 / 40 or 768 / 30
#define PIXELS_PER_POINT 25

// This enables slots for the type B multi-touch protocol.
// The kernel must support slots (ABS_MT_SLOT). The TouchPad kernel doesn't 
// seem to handle liftoffs with protocol B properly so leave it off for now.
#define USE_B_PROTOCOL 0

#endif // ANDROID_TOUCHSCREEN_CONFIG_H
