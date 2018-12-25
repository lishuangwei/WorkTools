#ifndef _SENSORS_SERVICE_H_
#define _SENSORS_SERVICE_H_

#include <hardware/hardware.h>
#include <hardware/sensors.h>

#include <binder/MemoryHeapBase.h>
#include <binder/MemoryBase.h>
#include <binder/IMemory.h>
#include <binder/IServiceManager.h>
#include <utils/String16.h>

#include "QVRServiceExternalSensors.h"

#include "ISharedBuffer.h"

typedef enum {
	PLATFORM_S1,
	PLATFORM_D2C,
	PLATFORM_DMICRO,
	_PLATFORM_NULL,
} platform_t;


#define SENSOR_SERVICE_NAME 	"sensors_service_3glasses"
#define SENSOR_SHM_NAME 		"sensor_event_shm"

#define SENSOR_SERVICE_VERSION_MAJOR	0
#define SENSOR_SERVICE_VERSION_MINOR	8

#define SENSORS_SERVICE_SHM_HAL_DATA_COUNT	64
#define SENSORS_SERVICE_SHM_QVR_DATA_COUNT	64
typedef struct {
	platform_t platform;
	volatile int hal_ev_valid_head;
	volatile int qvr_ev_valid_head;
	volatile uint32_t sensor_hal_enable_flag;
	volatile uint32_t sensor_qvr_enable_flag;
	sensors_event_t hal_ev[SENSORS_SERVICE_SHM_HAL_DATA_COUNT];
	sensor_sample_t qvr_ev[SENSORS_SERVICE_SHM_QVR_DATA_COUNT];
} sensors_service_shm_data_t;
#define SENSOR_SHM_SIZE sizeof(sensors_service_shm_data_t)

enum
{
	SENSORS_SERVICE_ID_ACC = 0,
	SENSORS_SERVICE_ID_GYR,
	SENSORS_SERVICE_ID_MAG,
	SENSORS_SERVICE_ID_RV,
	SENSORS_SERVICE_ID_PROXMITY,
	_SENSORS_SERVICE_ID_MAX,
};

#define SENSORS_SERVICE_SENSOR_FLAG_ACC		(1 << SENSORS_SERVICE_ID_ACC)
#define SENSORS_SERVICE_SENSOR_FLAG_GYR		(1 << SENSORS_SERVICE_ID_GYR)
#define SENSORS_SERVICE_SENSOR_FLAG_MAG		(1 << SENSORS_SERVICE_ID_MAG)
#define SENSORS_SERVICE_SENSOR_FLAG_RV		(1 << SENSORS_SERVICE_ID_RV)
#define SENSORS_SERVICE_SENSOR_FLAG_P		(1 << SENSORS_SERVICE_ID_PROXMITY)

#define SENSOR_SERVICE_HID_GET_DATA_DELAY_US	950

#endif /* _SENSORS_SERVICE_H_ */
