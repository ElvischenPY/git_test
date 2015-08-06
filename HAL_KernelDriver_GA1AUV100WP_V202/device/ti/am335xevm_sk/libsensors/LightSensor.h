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

#ifndef ANDROID_LIGHT_SENSOR_H
#define ANDROID_LIGHT_SENSOR_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include "sensors.h"
#include "SensorBase.h"
#include "InputEventReader.h"

#define LIGHT_PARAM_MAX_RANGE 0xFFFF
#define LIGHT_PARAM_RESOLUTION 0.001f 
#define LIGHT_PARAM_PAWER 0.06f 
#define LIGHT_PARAM_MIN_DELAY 250 

/* with panel or without panel */
//#define WITH_PANEL          

/* Coefficient of illuminance calculation formula 
 * ALFA, BETA : range from 100 to 9999 
 * CALC_OFFSET: more than or eaual to 1000
 * because preventing overflow and preserving calculation accuracy
 */
/* with panel */
#ifdef WITH_PANEL
#define ALFA1          0.1067
#define ALFA2          0.2819
#define ALFA3          0.1031
#define BETA1          0
#define BETA2          0.3504
#define BETA3          0.1121
#define RATIO_FIRST_BOUND   0.50
#define RATIO_SECOND_BOUND  0.75
#define RATIO_THIRD_BOUND   0.92
#else
/* without panel */
#define ALFA1          0.1067
#define ALFA2          0.2819
#define ALFA3          0.1031
#define BETA1          0
#define BETA2          0.3504
#define BETA3          0.1121
#define RATIO_FIRST_BOUND   0.5
#define RATIO_SECOND_BOUND  0.75
#define RATIO_THIRD_BOUND   0.92
#endif

/* coefficient for adjusting the difference in RANGE */
#define GAMMA_LOW_LUX_MODE    1
#define GAMMA_HIGH_LUX_MODE  32      /*   128/4 = 32   */

#define MAX_LUX_VALUE      100000
#define OVER_FLOW_COUNT     65000
#define ZERO_LUX_TH             2

/* ALS mode change */
#define LOW_LUX_MODE    ( 0 )
#define HIGH_LUX_MODE   ( 1 )

/*****************************************************************************/

struct input_event;

class LightSensor : public SensorBase {
    int mEnabled;
    InputEventCircularReader mInputReader;
    sensors_event_t mPendingEvent;
    bool mHasPendingEvent;
    char input_sysfs_path[PATH_MAX];
    int input_sysfs_path_len;

    int setInitialState();

    int          data_als[2];
    float        als_lux_prev ;
    float        lux;
    int          als_mode;
    virtual void als_get_lux( int *data_als, int *als_mode, float *als_lux_prev, float *lux );
  
public:
            LightSensor();
    virtual ~LightSensor();
    virtual int readEvents(sensors_event_t* data, int count);
    virtual bool hasPendingEvents() const;
    virtual int setDelay(int32_t handle, int64_t ns);
    virtual int enable(int32_t handle, int enabled);
};

/*****************************************************************************/

#endif  // ANDROID_LIGHT_SENSOR_H
