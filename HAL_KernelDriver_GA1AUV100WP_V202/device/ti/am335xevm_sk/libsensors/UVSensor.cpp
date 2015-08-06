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

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>
#include <cutils/log.h>

#include "sensors.h"
#include "UVSensor.h"

#define UV_ENABLE 		"uv_enable"
#define UV_DATA_NUM 	2

/*****************************************************************************/

UVSensor::UVSensor()
    : SensorBase(NULL, "uv_sensor"),
      mEnabled(0),
      mInputReader(4),
      mHasPendingEvent(false)
{
//    PG_LOG("UVSensor: UVSensor()\n");
    ALOGE("UVSensor (%s)", input_name);

    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = ID_UV;
    mPendingEvent.type = SENSOR_TYPE_UV;
    memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));

    if (data_fd) {
        strcpy(input_sysfs_path, "/sys/class/input/");
        strcat(input_sysfs_path, input_name);
        strcat(input_sysfs_path, "/device/");
        input_sysfs_path_len = strlen(input_sysfs_path);
        enable(0, 1);
    }
}

UVSensor::~UVSensor() {
    if (mEnabled) {
        enable(0, 0);
    }
}

void UVSensor::uv_get_val( int *data_uv, int *uv_mode, float *uv_prev, float *uv_val )
{
	//float 	uv_val ;
	float 	ratio ;
	int 	gamma=1 ;
	
	/* calculate the ratio */	
	if( data_uv[0] == 0 )
	{
		ratio = 1 ;
	}
	else
	{
		ratio =  (float)data_uv[1]  / data_uv[0] ;
	}

	/* coefficient for adjusting the difference in RANGE */
	if( *uv_mode == HIGH_UV_MODE )
		gamma = GAMMA_HIGH_UV_MODE ;
	else 
		gamma = GAMMA_LOW_UV_MODE ;

	/* calculate lux without adjusting parameters */
	if(ratio<RATIO_NOUV_BOUND)
	{
		*uv_val = gamma * (UV_ALFA1 * data_uv[0] - UV_BETA1 * data_uv[1]) ;
	}
  else
  {
    *uv_val = 0 ;
  }

	/* Exceptional process and precaution for calculation overflow */
	if ( ( data_uv[0] < ZERO_UV_TH ) &&
				 ( *uv_mode == LOW_UV_MODE ) )
	{
		*uv_val = 0 ;
	}
	else if ( ( data_uv[0] > OVER_FLOW_COUNT ) &&
				 ( *uv_mode == HIGH_UV_MODE ) )
	{
		*uv_val = MAX_UV_VALUE ;
	}
	else if ( ratio > RATIO_NOUV_BOUND )
	{
		*uv_val = 0 ;
	}
	
	*uv_prev = *uv_val ;
	
}
	
	
int UVSensor::setDelay(int32_t handle, int64_t ns)
{
    int fd;
    strcpy(&input_sysfs_path[input_sysfs_path_len], "uv_delay");
    fd = open(input_sysfs_path, O_RDWR);
    if (fd >= 0) {
        char buf[80];
        sprintf(buf, "%lld", ns);
        write(fd, buf, strlen(buf)+1);
        close(fd);
        return 0;
    }
    return -1;
}

int UVSensor::enable(int32_t handle, int en)
{
    int flags = en ? 1 : 0;
    if (flags != mEnabled) {
        int fd;
        strcpy(&input_sysfs_path[input_sysfs_path_len], UV_ENABLE);
        fd = open(input_sysfs_path, O_RDWR);
        ALOGE("open (%d)", fd);
        ALOGE("open (%s)", input_sysfs_path);
        if (fd >= 0) {
            char buf[2];
            int err;
            buf[1] = 0;
            if (flags) {
                buf[0] = '1';
            } else {
                buf[0] = '0';
            }
            err = write(fd, buf, sizeof(buf));
            ALOGE("write (%d)", err);
            close(fd);
            mEnabled = flags;
            return 0;
        }
        return -1;
    }
    return 0;
}

bool UVSensor::hasPendingEvents() const {
    return mHasPendingEvent;
}

int UVSensor::readEvents(sensors_event_t* data, int count)
{
    if (count < 1)
        return -EINVAL;

    if (mHasPendingEvent) {
        mHasPendingEvent = false;
        mPendingEvent.timestamp = getTimestamp();
        *data = mPendingEvent;
        return mEnabled ? 1 : 0;
    }

    ssize_t n = mInputReader.fill(data_fd);
    if (n < 0)
        return n;

    int numEventReceived = 0;
    input_event const* event;

    while (count && mInputReader.readEvent(&event)) {
        int type = event->type;
        if (type == EV_ABS) {
            if (event->code == EVENT_TYPE_UV) {
              unsigned int dataIdx = (0x00010000 & event->value) >> 16;
              uv_mode = (0xFFFF0000 & event->value) >> 17;
              if(dataIdx >= UV_DATA_NUM){
                //ALOGW("data index over! idx:%d report: %d\n", dataIdx, event->value);
              }else{
                data_uv[dataIdx] = (0x0000FFFF & event->value);
                //ALOGW("[1-2]data_uv[0]:%d, data_uv[1]:%d dataIdx:%d report\n",data_uv[0], data_uv[1], dataIdx);
              }
            }
        } else if (type == EV_SYN) {
            uv_get_val(data_uv, &uv_mode, &uv_prev, &uv_val);
            //ALOGW("[9] EV_SYN lux:%f, lux_prev:%f report\n",lux, uv_prev);
            mPendingEvent.timestamp = timevalToNano(event->time);
            mPendingEvent.light = uv_val;
            if (mEnabled) {
                *data++ = mPendingEvent;
                count--;
                numEventReceived++;
            }
        } else {
//            EV_LOG("LightSensor: unknown event (type=%d, code=%d)",
//                    type, event->code);
        }
        mInputReader.next();
    }

    return numEventReceived;
}
