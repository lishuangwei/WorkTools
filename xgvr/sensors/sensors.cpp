
#define LOG_TAG "3glasses-sensors"
// #defined LOG_NDEBUG  1
#include <utils/Log.h>

#include "sensors.h"

#include <stdlib.h>
#include <time.h>

#include <hardware/sensors.h>

SensorBase::SensorBase() {}
SensorBase::~SensorBase() {}

int SensorBase::activate(int handle, int enabled) {
  ALOGD("activate");
  return 0;
}

int SensorBase::setDelay(int handle, int64_t ns) {
  return 0;
}

int SensorBase::batch(int handle, int flags,
                      int64_t period_ns, int64_t timeout) {
  return 0;
}

int SensorBase::flush(int handle) {
  return 0;
}


Accelerometer::Accelerometer() {}
Accelerometer::~Accelerometer() {}

int Accelerometer::pollEvents(sensors_event_t* data, int count) {
  ALOGD("Accelerometer::pollEvents");
#if 1
  sleep(1);
  // Returns fake random values.
  data->acceleration.x = static_cast<float>(random() % kMaxRange);
  data->acceleration.y = static_cast<float>(random() % kMaxRange);
  data->acceleration.z = static_cast<float>(random() % kMaxRange);
  return 1;
#else
  return -1;
#endif
}


Gyroscope::Gyroscope() {}
Gyroscope::~Gyroscope() {}

int Gyroscope::pollEvents(sensors_event_t* data, int count) {
  ALOGD("Gyroscope::pollEvents");
#if 1
  sleep(1);
  if (data) {
    // For this custom sensor, we will return the hour of the local time.
    time_t t = time(NULL);
    struct tm* now = localtime(&t);
    if (now) {
      // Any field can be used to return data for a custom sensor.
      // See definition of struct sensors_event_t in hardware/sensors.h for the
      // available fields.
      data->data[0] = now->tm_hour;
      return 1;
    }
  }
  // Note: poll() return value is the number of events being returned. We use
  // -1 to signal an error.
  return -1;
#else
  return -1;
#endif
}




Fusion::Fusion() {}
Fusion::~Fusion() {}

int Fusion::pollEvents(sensors_event_t* data, int count) {
  ALOGD("Fusion::pollEvents");
#if 1
  sleep(1);
  if (data) {
    // For this custom sensor, we will return the hour of the local time.
    time_t t = time(NULL);
    struct tm* now = localtime(&t);
    if (now) {
      // Any field can be used to return data for a custom sensor.
      // See definition of struct sensors_event_t in hardware/sensors.h for the
      // available fields.
      data->data[0] = now->tm_hour;
      return 1;
    }
  }
  // Note: poll() return value is the number of events being returned. We use
  // -1 to signal an error.
  return -1;
#else
  return -1;
#endif
}
