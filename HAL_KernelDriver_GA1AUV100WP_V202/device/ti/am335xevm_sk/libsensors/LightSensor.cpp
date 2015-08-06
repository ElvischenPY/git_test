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
#include "LightSensor.h"
//#include "ProxGestureSensorBase.h"


#define ALS_ENABLE     "als_enable"
#define ALS_DATA_NUM   2

/*****************************************************************************/

LightSensor::LightSensor()
    : SensorBase(NULL, "light_sensor"),
      mEnabled(0),
      mInputReader(4),
      mHasPendingEvent(false)
{
//    PG_LOG("LightSensor: LightSensor()\n");
    ALOGE("LightSensor (%s)", input_name);

    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = ID_L;
    mPendingEvent.type = SENSOR_TYPE_LIGHT;
    memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));

    if (data_fd) {
        strcpy(input_sysfs_path, "/sys/class/input/");
        strcat(input_sysfs_path, input_name);
        strcat(input_sysfs_path, "/device/");
        input_sysfs_path_len = strlen(input_sysfs_path);
        enable(0, 1);
    }
}

LightSensor::~LightSensor() {
    if (mEnabled) {
        enable(0, 0);
    }
}

void LightSensor::als_get_lux( int *data_als, int *als_mode, float *als_lux_prev, float *lux )
{
  //float   lux ;
  float ratio ;
  int   gamma=1 ;

  /* calculate the ratio */  
  if( data_als[0] == 0 )
  {
    ratio = 1 ;
  }
  else
  {
    ratio =  (float)data_als[1]/data_als[0] ;
  }

  /* coefficient for adjusting the difference in RANGE */
  if( *als_mode == HIGH_LUX_MODE )
    gamma = GAMMA_HIGH_LUX_MODE;
  else 
    gamma = GAMMA_LOW_LUX_MODE;

  /* calculate lux without adjusting parameters */
  if(ratio<=RATIO_FIRST_BOUND)
  {
    *lux = gamma * (ALFA1 * data_als[0] - BETA1 * data_als[1]) ;
  }
  else if (ratio<=RATIO_SECOND_BOUND)
  {
    *lux = gamma * (ALFA2 * data_als[0] - BETA2 * data_als[1]) ;
  }
  else if (ratio<=RATIO_THIRD_BOUND)
  {
    *lux = gamma * (ALFA3 * data_als[0] - BETA3 * data_als[1]) ;
  }

  /* Exceptional process and precaution for calculation overflow */
  if ( ( data_als[0] < ZERO_LUX_TH ) &&
         ( *als_mode == LOW_LUX_MODE ) )
  {/* ZERO output condition */
    *lux = 0 ;
  }
  else if ( ( data_als[0] > OVER_FLOW_COUNT ) &&
         ( *als_mode == HIGH_LUX_MODE ) )
  {/* raw data overflow */
    *lux = MAX_LUX_VALUE ;
  }
  else if ( ratio>RATIO_THIRD_BOUND )
  {/* Illegal data */
    *lux = *als_lux_prev;
  }

  *als_lux_prev = *lux ;

}


int LightSensor::setDelay(int32_t handle, int64_t ns)
{
    int fd;
    strcpy(&input_sysfs_path[input_sysfs_path_len], "als_delay");
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

int LightSensor::enable(int32_t handle, int en)
{
    int flags = en ? 1 : 0;
    if (flags != mEnabled) {
        int fd;
        strcpy(&input_sysfs_path[input_sysfs_path_len], ALS_ENABLE);
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

bool LightSensor::hasPendingEvents() const {
    return mHasPendingEvent;
}

int LightSensor::readEvents(sensors_event_t* data, int count)
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
          if (event->code == EVENT_TYPE_LIGHT) {
            unsigned int dataIdx = (0x00010000 & event->value) >> 16;
            als_mode = (0xFFFF0000 & event->value) >> 17;
            if(dataIdx >= ALS_DATA_NUM){
              //ALOGW("data index over! idx:%d report: %d\n", dataIdx, event->value);
            }else{
              data_als[dataIdx] = (0x0000FFFF & event->value);
              //ALOGW("[1-2]data_als[0]:%d, data_als[1]:%d dataIdx:%d report\n",data_als[0], data_als[1], dataIdx);
            }
          }
        } else if (type == EV_SYN) {
          als_get_lux(data_als, &als_mode, &als_lux_prev, &lux);
          //ALOGW("[9] EV_SYN lux:%f, lux_prev:%f , i:%d report\n",lux, als_lux_prev, i);
          mPendingEvent.timestamp = timevalToNano(event->time);
          mPendingEvent.light = lux;
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
