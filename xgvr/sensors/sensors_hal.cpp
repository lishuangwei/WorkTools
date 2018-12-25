/*
 * Copyright (C) 2015 The Android Open Source Project
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
#define LOG_TAG "3glasses-sensors-hal"
// #define LOG_NDEBUG  1
#include <utils/Log.h>

#include "sensors_hal.h"

#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include <linux/input.h>
#include <linux/hidraw.h>
#include <sys/ioctl.h>

#include "sensors_service.h"

#include "sensors.h"

uint64_t _get_timestamp();

SensorContext::SensorContext(const hw_module_t* module) {
    memset(&device, 0, sizeof(device));

    device.common.tag = HARDWARE_DEVICE_TAG;
    device.common.version = SENSORS_DEVICE_API_VERSION_1_3;
    device.common.module = const_cast<hw_module_t*>(module);
    device.common.close = CloseWrapper;
    device.activate = ActivateWrapper;
    device.setDelay = SetDelayWrapper;
    device.poll = PollEventsWrapper;
    device.batch = BatchWrapper;
    device.flush = FlushWrapper;

    // sensors[SENSORS_SERVICE_ID_ACC] = new Accelerometer();
    // sensors[SENSORS_SERVICE_ID_GYR] = new Gyroscope();
    // sensors[SENSORS_SERVICE_ID_MAG] = new Magnetometer();
    // sensors[SENSORS_SERVICE_ID_RV] = new Fusion();

    batch_period_us = 66667;
    batch_timeout_us = 100000;
}

SensorContext::~SensorContext() {
    // for (int i = 0; i < AvailableSensors::kNumSensors; i++) {
    //   if (sensors[i]) delete sensors[i];
    // }
}

int SensorContext::activate(int handle, int enabled) {
    ALOGD("activate handle(%d) %s.", handle, enabled ? "enable" : "disable");
    sensors_service_shm_data_t *shm_data;

    sp<IBinder> binder = defaultServiceManager()->getService(String16(SENSOR_SERVICE_NAME));
    sp<ISharedBuffer> service = ISharedBuffer::asInterface(binder);
    sp<IMemory> buffer = service->getBuffer();
    shm_data = (sensors_service_shm_data_t*)buffer->pointer();
    if (enabled) {
        if (handle == SENSORS_SERVICE_ID_ACC) {
            ALOGD("activate SENSORS_SERVICE_ID_ACC.");
            shm_data->sensor_hal_enable_flag |= SENSORS_SERVICE_SENSOR_FLAG_ACC;
        }
        if (handle == SENSORS_SERVICE_ID_GYR) {
            ALOGD("activate SENSORS_SERVICE_ID_GYR.");
            shm_data->sensor_hal_enable_flag |= SENSORS_SERVICE_SENSOR_FLAG_GYR;
        }
        if (handle == SENSORS_SERVICE_ID_MAG) {
            ALOGD("activate SENSORS_SERVICE_ID_MAG.");
            shm_data->sensor_hal_enable_flag |= SENSORS_SERVICE_SENSOR_FLAG_MAG;
        }
        if (handle == SENSORS_SERVICE_ID_RV) {
            ALOGD("activate SENSORS_SERVICE_ID_RV.");
            shm_data->sensor_hal_enable_flag |= SENSORS_SERVICE_SENSOR_FLAG_RV;
        }
        if (handle == SENSORS_SERVICE_ID_PROXMITY) {
            ALOGD("activate SENSORS_SERVICE_ID_PROXMITY.");
            shm_data->sensor_hal_enable_flag |= SENSORS_SERVICE_SENSOR_FLAG_P;
        }
    } else {
        if (handle == SENSORS_SERVICE_ID_ACC) {
            ALOGD("deactivate SENSORS_SERVICE_ID_ACC.");
            shm_data->sensor_hal_enable_flag &= ~SENSORS_SERVICE_SENSOR_FLAG_ACC;
        }
        if (handle == SENSORS_SERVICE_ID_GYR) {
            ALOGD("deactivate SENSORS_SERVICE_ID_GYR.");
            shm_data->sensor_hal_enable_flag &= ~SENSORS_SERVICE_SENSOR_FLAG_GYR;
        }
        if (handle == SENSORS_SERVICE_ID_MAG) {
            ALOGD("deactivate SENSORS_SERVICE_ID_MAG.");
            shm_data->sensor_hal_enable_flag &= ~SENSORS_SERVICE_SENSOR_FLAG_MAG;
        }
        if (handle == SENSORS_SERVICE_ID_RV) {
            ALOGD("deactivate SENSORS_SERVICE_ID_RV.");
            shm_data->sensor_hal_enable_flag &= ~SENSORS_SERVICE_SENSOR_FLAG_RV;
        }
        if (handle == SENSORS_SERVICE_ID_PROXMITY) {
            ALOGD("deactivate SENSORS_SERVICE_ID_PROXMITY.");
            shm_data->sensor_hal_enable_flag &= ~SENSORS_SERVICE_SENSOR_FLAG_P;
        }
    }
    is_sensor_activate[handle] = enabled;
    ::sched_yield();

    return 0;
}

int SensorContext::setDelay(int handle, int64_t ns) {
    ALOGD("setDelay handle(%d) ns(%d).", handle, (int)ns);
    return 0;
}

uint64_t _get_timestamp() {
    struct timespec t;

    t.tv_sec = t.tv_nsec = 0;
    clock_gettime(CLOCK_BOOTTIME, &t);

    return  (uint64_t(t.tv_sec) * 1000000000LL + t.tv_nsec);
}

int SensorContext::pollEvents(sensors_event_t* data, int count) {
    // ALOGD("pollEvents (%d).", count);
#if 0
    time_t t = time(NULL);
    struct tm* now = localtime(&t);
    uint64_t timestamp = _get_timestamp();
    int total_count = count;

    if (is_hid_open == false) {
        mHidFd = ::open("/dev/hidraw0", O_RDWR);
        if (mHidFd <= 0) {
            ALOGE("Open hidraw0 fail!!");
            // ::close(mHidFd);

            // Make sure HAL not going to endless polling
            sleep(1);
            return 0;
        } else {
            ::close(mHidFd);
        }
    }

    while (count) {
        if (count && is_sensor_activate[SENSORS_SERVICE_ID_ACC]) {
            data->version = sizeof(sensors_event_t);
            data->sensor = SENSORS_SERVICE_ID_ACC;
            data->type = SENSOR_TYPE_ACCELEROMETER;
            data->acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;
            data->timestamp = _get_timestamp();
            data->data[0] = (float)now->tm_sec / 60.0f;
            data->data[1] = 0.1f;
            data->data[2] = 0.2f;
            data ++;
            count--;
        }

        if (count && is_sensor_activate[SENSORS_SERVICE_ID_GYR]) {
            data->version = sizeof(sensors_event_t);
            data->sensor = SENSORS_SERVICE_ID_GYR;
            data->type = SENSOR_TYPE_GYROSCOPE;
            data->acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;
            data->timestamp = _get_timestamp();
            data->data[0] = (float)now->tm_sec / 60.0f;
            data->data[1] = 0.3f;
            data->data[2] = 0.4f;
            data ++;
            count--;
        }

        if (count && is_sensor_activate[SENSORS_SERVICE_ID_RV]) {
            data->version = sizeof(sensors_event_t);
            data->sensor = SENSORS_SERVICE_ID_RV;
            data->type = SENSOR_TYPE_ROTATION_VECTOR;
            data->acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;
            data->timestamp = _get_timestamp();
            data->data[0] = (float)now->tm_sec / 60.0f;
            data->data[1] = 0.4f;
            data->data[2] = 0.6f;
            data->data[3] = 0.7f;
            data ++;
            count--;
        }
        usleep(1000);
    }
    return total_count;
#else
    static int pos = -1;
    int valid_head;
    int total_count;
    uint32_t penalty_wait_us = 950;
    sensors_service_shm_data_t *shm_data = NULL;

    // clock_t begin = ::clock();

    sp<IBinder> binder = defaultServiceManager()->getService(String16(SENSOR_SERVICE_NAME));
    sp<ISharedBuffer> service = ISharedBuffer::asInterface(binder);
    sp<IMemory> buffer = service->getBuffer();
    shm_data = (sensors_service_shm_data_t*)buffer->pointer();

    // clock_t end = ::clock();
    // double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    // ALOGD("shm_data(%p). count(%d)", shm_data, count);
    // ALOGD("elapsed_secs(%lf)", elapsed_secs);

    while ((valid_head = shm_data->hal_ev_valid_head) < 0) {
        ALOGD("wait for low level init..");
        usleep(1000000);
    }

    total_count = 0;
    clock_t start_time = clock();
    clock_t curr_time;
    while (total_count < count) {
        if (is_sensor_activate[SENSORS_SERVICE_ID_ACC]
                || is_sensor_activate[SENSORS_SERVICE_ID_MAG]
                || is_sensor_activate[SENSORS_SERVICE_ID_GYR]
                || is_sensor_activate[SENSORS_SERVICE_ID_RV]
                || is_sensor_activate[SENSORS_SERVICE_ID_PROXMITY]
           ) {
            int i;
            sensors_event_t *ev;
            if ((valid_head = shm_data->hal_ev_valid_head) != pos) {

                if (valid_head >= pos) {
                    // printf("start m1 loop(%d %d).\n", pos, valid_head);
                } else {
                    // printf("start m2 loop(%d %d).\n", pos, valid_head + SENSORS_SERVICE_SHM_HAL_DATA_COUNT);
                    valid_head = valid_head + SENSORS_SERVICE_SHM_HAL_DATA_COUNT;
                }
                for (i = pos + 1; i <= valid_head; i++) {
                    ev = &shm_data->hal_ev[i % SENSORS_SERVICE_SHM_HAL_DATA_COUNT];
                    memcpy(data, ev, sizeof(sensors_event_t));
                    data++;
                    total_count++;
                    if (total_count >= count) {
                        break;
                    }
                }
                pos = (i - 1)  % SENSORS_SERVICE_SHM_HAL_DATA_COUNT;
                penalty_wait_us = 950;
            } else {
                // no new data arrived.
                usleep(penalty_wait_us);
                // ALOGD("[test] timeout, penalty_wait_us(%d).", penalty_wait_us);
                penalty_wait_us = penalty_wait_us < 1000000 ? penalty_wait_us * 1.1 : penalty_wait_us;
                if (is_sensor_activate[SENSORS_SERVICE_ID_PROXMITY]) {
                    if (total_count) {
                        // ALOGD("[test] timeout, total_count(%d).", total_count);
                        break;
                    }
                }
            }
        }
        ::sched_yield();
    }

    // ALOGD("return total_count(%d)..", total_count);
    return total_count;
#endif
}

int SensorContext::batch(int handle, int flags,
                         int64_t period_ns, int64_t timeout) {
    ALOGD("batch handle(%d) flags(%d) period_ns(%d) timeout(%d).", handle, flags, (int)period_ns, (int)timeout);
    batch_period_us = period_ns / 1000;
    batch_timeout_us = timeout / 1000;
    return 0;
}

int SensorContext::flush(int handle) {
    ALOGD("flush handle(%d).", handle);
    return 0;
}

// static
int SensorContext::CloseWrapper(hw_device_t* dev) {
    SensorContext* sensor_context = reinterpret_cast<SensorContext*>(dev);
    if (sensor_context) {
        delete sensor_context;
    }
    return 0;
}

// static
int SensorContext::ActivateWrapper(sensors_poll_device_t* dev,
                                   int handle, int enabled) {
    return reinterpret_cast<SensorContext*>(dev)->activate(handle, enabled);
}

// static
int SensorContext::SetDelayWrapper(sensors_poll_device_t* dev,
                                   int handle, int64_t ns) {
    return reinterpret_cast<SensorContext*>(dev)->setDelay(handle, ns);
}

// static
int SensorContext::PollEventsWrapper(sensors_poll_device_t* dev,
                                     sensors_event_t* data, int count) {
    return reinterpret_cast<SensorContext*>(dev)->pollEvents(data, count);
}

// static
int SensorContext::BatchWrapper(sensors_poll_device_1_t* dev, int handle,
                                int flags, int64_t period_ns, int64_t timeout) {
    ALOGD("BatchWrapper");
    return reinterpret_cast<SensorContext*>(dev)->batch(handle, flags, period_ns,
            timeout);
}

// static
int SensorContext::FlushWrapper(sensors_poll_device_1_t* dev,
                                int handle) {
    ALOGD("FlushWrapper");
    return reinterpret_cast<SensorContext*>(dev)->flush(handle);
}

static int open_sensors(const struct hw_module_t* module,
                        const char* id, struct hw_device_t** device) {
    SensorContext* ctx = new SensorContext(module);
    *device = &ctx->device.common;

    return 0;
}

static struct hw_module_methods_t sensors_module_methods = {
    .open = open_sensors,
};

static struct sensor_t kSensorList[] = {
    {
        .name = "3Glasses Accelerometer",
        .vendor = "3Glasses",
        .version = 1,
        .handle = SENSORS_SERVICE_ID_ACC,
        .type = SENSOR_TYPE_ACCELEROMETER,
        .maxRange = GRAVITY_EARTH * 8.0f,
        .resolution = GRAVITY_EARTH * 8.0f / 32768.0f,
        .power = 0.0f,
        .minDelay = 1000,
        .fifoReservedEventCount = 0,
        .fifoMaxEventCount = 0,
        .stringType = SENSOR_STRING_TYPE_ACCELEROMETER,
        .requiredPermission = "",
        .maxDelay = 0,
        .flags = SENSOR_FLAG_CONTINUOUS_MODE,
        .reserved = {},
    },
    {
        .name = "3Glasses Gyroscope",
        .vendor = "3Glasses",
        .version = 1,
        .handle = SENSORS_SERVICE_ID_GYR,
        .type = SENSOR_TYPE_GYROSCOPE,
        .maxRange = 1000.0f * M_PI / 180.0f,
        .resolution = 1000.0f * M_PI / (180.0f * 32768.0f),
        .power = 0.0f,
        .minDelay = 1000,
        .fifoReservedEventCount = 0,
        .fifoMaxEventCount = 0,
        .stringType = SENSOR_STRING_TYPE_GYROSCOPE,
        .requiredPermission = "",
        .maxDelay = 0,
        .flags = SENSOR_FLAG_CONTINUOUS_MODE,
        .reserved = {},
    },
    {
        .name = "3Glasses Magnetic Field",
        .vendor = "3Glasses",
        .version = 1,
        .handle = SENSORS_SERVICE_ID_MAG,
        .type = SENSOR_TYPE_GEOMAGNETIC_FIELD,
        .maxRange = GRAVITY_EARTH * 8.0f,
        .resolution = GRAVITY_EARTH * 8.0f / 32768.0f,
        .power = 0.0f,
        .minDelay = 1000,
        .fifoReservedEventCount = 0,
        .fifoMaxEventCount = 0,
        .stringType = SENSOR_STRING_TYPE_MAGNETIC_FIELD,
        .requiredPermission = "",
        .maxDelay = 0,
        .flags = SENSOR_FLAG_CONTINUOUS_MODE,
        .reserved = {},
    },
    {
        .name = "3Glasses Fusion",
        .vendor = "3Glasses",
        .version = 1,
        .handle = SENSORS_SERVICE_ID_RV,
        .type = SENSOR_TYPE_ROTATION_VECTOR,
        .maxRange = 1.0f,
        .resolution = 1.0f,
        .power = 0.0f,
        .minDelay = 1000,
        .fifoReservedEventCount = 0,
        .fifoMaxEventCount = 0,
        .stringType = SENSOR_STRING_TYPE_ROTATION_VECTOR,
        .requiredPermission = "",
        .maxDelay = 0,
        .flags = SENSOR_FLAG_CONTINUOUS_MODE,
        .reserved = {},
    },
    {
        .name = "3Glasses Proximity Sensor",
        .vendor = "3Glasses",
        .version = 1,
        .handle = SENSORS_SERVICE_ID_PROXMITY,
        .type = SENSOR_TYPE_PROXIMITY,
        .maxRange = 5.0f,                                  // maxRange (cm)
        .resolution = 1.0f,                                // resolution (cm)
        .power = 0.0f,
        .minDelay = 1000,
        .fifoReservedEventCount = 0,
        .fifoMaxEventCount = 0,
        .stringType = SENSOR_STRING_TYPE_PROXIMITY,
        .requiredPermission = "",
        .maxDelay = 200000000,
        .flags = SENSOR_FLAG_WAKE_UP | SENSOR_FLAG_ON_CHANGE_MODE,
        .reserved = {},
    }
};

static int get_sensors_list(struct sensors_module_t* module,
                            struct sensor_t const** list) {
    if (!list) return 0;
    *list = kSensorList;
    return sizeof(kSensorList) / sizeof(kSensorList[0]);
}

struct sensors_module_t HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .version_major = 1,
        .version_minor = 0,
        .id = SENSORS_HARDWARE_MODULE_ID,
        .name = "3Glasses Sensor Module",
        .author = "3Glasses",
        .methods = &sensors_module_methods,
        .dso = NULL,
        .reserved = {0},
    },
    .get_sensors_list = get_sensors_list,
    .set_operation_mode = NULL
};
