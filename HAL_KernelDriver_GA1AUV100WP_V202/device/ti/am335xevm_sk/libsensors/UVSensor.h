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

#ifndef ANDROID_UV_SENSOR_H
#define ANDROID_UV_SENSOR_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include "sensors.h"
#include "SensorBase.h"
#include "InputEventReader.h"

#define UV_PARAM_MAX_RANGE  0xFFFF
#define UV_PARAM_RESOLUTION 0.001f 
#define UV_PARAM_PAWER      0.06f 
#define UV_PARAM_MIN_DELAY  250 

/* with panel or without panel */
//#define WITH_PANEL

/* with panel */
#ifdef WITH_PANEL
#define UV_ALFA1				0.00004216 //4.216 * 10^-5
#define UV_BETA1				0.00004216
#define RATIO_NOUV_BOUND		1.00
#else
/* without panel */
#define UV_ALFA1				0.00004216
#define UV_BETA1				0.00004216
#define RATIO_NOUV_BOUND		1.00
#endif

/* coefficient for adjusting the difference in RANGE */
#define GAMMA_LOW_UV_MODE		 1
#define GAMMA_HIGH_UV_MODE	32			/*   128/4 = 32   */

#define MAX_UV_VALUE			 9999		/* value when the sensor data overflow */
#define OVER_FLOW_COUNT	 	65000		/* less tnan < 2^(resolution) -1 */
#define ZERO_UV_TH				2

/* UV mode change */
#define LOW_UV_MODE			( 0 )
#define HIGH_UV_MODE		( 1 )

/*****************************************************************************/

struct input_event;

class UVSensor : public SensorBase {
    int mEnabled;
    InputEventCircularReader mInputReader;
    sensors_event_t mPendingEvent;
    bool mHasPendingEvent;
    char input_sysfs_path[PATH_MAX];
    int input_sysfs_path_len;

    int setInitialState();

	int 				data_uv[2];
	float				uv_prev ;
	float    			uv_val;
	int 				uv_mode;
	virtual void 		uv_get_val( int *data_uv, int *uv_mode, float *uv_prev, float *uv_val );
public:
            UVSensor();
    virtual ~UVSensor();
    virtual int readEvents(sensors_event_t* data, int count);
    virtual bool hasPendingEvents() const;
    virtual int setDelay(int32_t handle, int64_t ns);
    virtual int enable(int32_t handle, int enabled);
};

/*****************************************************************************/

#endif  // ANDROID_UV_SENSOR_H
