/*
 * Copyright (C) 2008 The Android Open Source Project
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

#include <hardware/sensors.h>

#include "nusensors.h"

/*****************************************************************************/

/*
 * The SENSORS Module
 */

/*
 * the AK8973 has a 8-bit ADC but the firmware seems to average 16 samples,
 * or at least makes its calibration on 12-bits values. This increases the
 * resolution by 4 bits.
 */

static const struct sensor_t sSensorList[] = {
        { "LSM303DLH 3-axis Accelerometer",
                "ST Microelectronics",
                1, SENSORS_HANDLE_BASE+ID_A,
                SENSOR_TYPE_ACCELEROMETER, 2.0f*GRAVITY_EARTH, (2.0f*GRAVITY_EARTH)/2048.0f, 0.5f, 10000/*10ms*/, 0, 0, 0, 0, 0, 0, { } },
        { "LSM303DLH Magnetometer",
                "ST Microelectronics",
                1, SENSORS_HANDLE_BASE+ID_M,
                SENSOR_TYPE_MAGNETIC_FIELD, 130.0f, 0.05f, 0.5f, 10000/*10ms*/, 0, 0, 0, 0, 0, 0, { } },
        { "ISL29023 Light sensor",
                "Intersil",
                1, SENSORS_HANDLE_BASE+ID_L,
                SENSOR_TYPE_LIGHT, 3000.0f, 1.0f, 0.75f, 0, 0, 0, 0, 0, 0, 0, { } },
        { "MPL Gyroscope",
                "Invensense",
                1, SENSORS_HANDLE_BASE+ID_GY,
                SENSOR_TYPE_GYROSCOPE, 2000.0f*RAD_P_DEG, 32.8f*RAD_P_DEG, 0.5f, 10000/*10ms*/, 0, 0, 0, 0, 0, 0, { } },
        { "MPL Temperature sensor",
                "Invensense",
                1, SENSORS_HANDLE_BASE+ID_T,
                SENSOR_TYPE_AMBIENT_TEMPERATURE, 80.0f, 0.5f, 0.0f, 10000/*10ms*/, 0, 0, 0, 0, 0, 0, { } },
};

static int open_sensors(const struct hw_module_t* module, const char* name,
        struct hw_device_t** device);

static int sensors__get_sensors_list(struct sensors_module_t* module,
        struct sensor_t const** list)
{
    *list = sSensorList;
    return ARRAY_SIZE(sSensorList);
}

#if defined(SENSORS_DEVICE_API_VERSION_1_4)
static int sensors__set_operation_mode(unsigned int mode)
{
    (void)mode;
    return 0;
}
#endif

static struct hw_module_methods_t sensors_module_methods = {
    .open = open_sensors
};

struct sensors_module_t HAL_MODULE_INFO_SYM = {
        common: {
                tag: HARDWARE_MODULE_TAG,
                version_major: 1,
                version_minor: 0,
                id: SENSORS_HARDWARE_MODULE_ID,
                name: "Hewlet Packard Sensors Module",
                author: "Erik Hardesty",
                methods: &sensors_module_methods,
                dso: NULL,
                reserved: {0}
         },
         get_sensors_list: sensors__get_sensors_list,
#if defined(SENSORS_DEVICE_API_VERSION_1_4)
         set_operation_mode: sensors__set_operation_mode,
#endif
};

/*****************************************************************************/

static int open_sensors(const struct hw_module_t* module, const char* name,
        struct hw_device_t** device)
{
    return init_nusensors(module, device);
}
