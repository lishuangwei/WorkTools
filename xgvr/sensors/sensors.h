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

#ifndef _SENSORS_H_
#define _SENSORS_H_

#include <stdint.h>

struct sensors_event_t;

class SensorBase {
public:
  SensorBase();
  virtual ~SensorBase();

  virtual int activate(int handle, int enabled);
  virtual int setDelay(int handle, int64_t ns);
  virtual int pollEvents(sensors_event_t* data, int count) = 0;
  virtual int batch(int handle, int flags, int64_t period_ns, int64_t timeout);
  virtual int flush(int handle);
};

class Accelerometer : public SensorBase {
public:
  static const int kMaxRange = 1000;

  Accelerometer();
  ~Accelerometer() override;

  int pollEvents(sensors_event_t* data, int count) override;
};

class Gyroscope : public SensorBase {
public:
  Gyroscope();
  ~Gyroscope() override;

  int pollEvents(sensors_event_t* data, int count) override;
};

class Fusion : public SensorBase {
public:
  Fusion();
  ~Fusion() override;

  int pollEvents(sensors_event_t* data, int count) override;
};

#endif  /* _SENSORS_H_ */
