#define LOG_TAG "3glasses-sensors-qvr"
// #define LOG_NDEBUG  1
#include <utils/Log.h>

#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include <linux/input.h>
#include <linux/hidraw.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <pthread.h>

#include <hardware/hardware.h>
#include <hardware/sensors.h>

#include <sys/types.h>

#include "QVRServiceExternalSensors.h"

#include "sensors_service.h"

#define QVR_PIN_CPU_CORE     1
#define QVR_SCHED_FIFO_CORE  2

typedef struct {
	data_ready_callback_fn data_ready_cb;
	handle_error_callback_fn handle_error_cb;
	void* pCtx;
} qvr_thread_param_t;

pthread_t qvr_thread;
qvr_thread_param_t *qvr_thread_param;
volatile bool stop;
volatile bool running;

int qvr_external_init(void) {
	ALOGD("**** qvr_external_init");
	return 0;
}

int qvr_external_deinit(void) {
	ALOGD("**** qvr_external_deinit");
	return 0;
}

// set fixed cpu
int _qvr_set_thread_affinity(int coreId)
//-----------------------------------------------------------------------------
{
	long rv = 0;
	int affinity_mask = 0;
	pid_t tid = gettid();

	rv = syscall(__NR_sched_getaffinity, tid, sizeof(affinity_mask), &affinity_mask);
	ALOGI("Current Affinity (Thread 0x%x): 0x%x, rv: %d", (int)tid, affinity_mask, rv);

	int mask = 1 << coreId;
	rv = syscall(__NR_sched_setaffinity, tid, sizeof(mask), &mask);

	affinity_mask = 0;
	rv = syscall(__NR_sched_getaffinity, tid, sizeof(affinity_mask), &affinity_mask);
	ALOGI("    New Affinity (Thread 0x%x): 0x%x, rv: %d", (int)tid, affinity_mask, rv);

	return (rv == 0 ? 1 : 0);
}

// thread set fifo
void qvr_set_sched_fifo(void)
{
	struct sched_param param = {
		.sched_priority = 98,     /*level range  0-99*/
	};

	if (sched_setscheduler(gettid(), SCHED_FIFO, &param) < 0) {
		ALOGE("set sched_fifo failed");
	}
	_qvr_set_thread_affinity(QVR_SCHED_FIFO_CORE);
}

void *qvr_external_thread(qvr_thread_param_t *p) {
	int rv;
	static int pos = -1;
	int valid_head;
	int total_count;
	sensors_service_shm_data_t *shm_data;

	sp<IBinder> binder = defaultServiceManager()->getService(String16(SENSOR_SERVICE_NAME));
	sp<ISharedBuffer> service = ISharedBuffer::asInterface(binder);
	sp<IMemory> buffer = service->getBuffer();
	shm_data = (sensors_service_shm_data_t*)buffer->pointer();

#if QVR_PIN_CPU_CORE
	qvr_set_sched_fifo();
#endif

	ALOGD("shm_data(%p).", shm_data);
	running = true;

	shm_data->sensor_qvr_enable_flag |= (SENSORS_SERVICE_SENSOR_FLAG_ACC | SENSORS_SERVICE_SENSOR_FLAG_GYR);

	while (!stop && ((valid_head = shm_data->qvr_ev_valid_head) < 0)) {
#if 0
		::sched_yield();
#else
		ALOGD("wait for low level init.");
		usleep(1000000);
#endif
	}

	while (!stop) {
		int i;
		sensor_sample_t *ev;

		while (!stop && ((valid_head = shm_data->qvr_ev_valid_head) == pos)) {
#if 0
			::sched_yield();
#else
			// ALOGD("wait for new data.. shm_data->qvr_ev_valid_head(%d)\n", shm_data->qvr_ev_valid_head);
			usleep(500);
#endif
		}

		if (valid_head >= pos) {
			// printf("start m1 loop(%d %d).\n", pos, valid_head);
		} else {
			// printf("start m2 loop(%d %d).\n", pos, valid_head + SENSORS_SERVICE_SHM_QVR_DATA_COUNT);
			valid_head = valid_head + SENSORS_SERVICE_SHM_QVR_DATA_COUNT;
		}
		for (i = pos + 1; i <= valid_head; i++) {
#if 0
			sensor_sample_t event_data;
			ev = &shm_data->qvr_ev[i % SENSORS_SERVICE_SHM_QVR_DATA_COUNT];
			// printf("pos(%d) ev->type(%d) valid_head(%d)\n", pos, ev->type, valid_head);

			event_data.ts = ev->ts;
			event_data.sample[0] = ev->sample[0];
			event_data.sample[1] = ev->sample[1];
			event_data.sample[2] = ev->sample[2];
			p->data_ready_cb(p->pCtx, &event_data);
#else
			ev = &shm_data->qvr_ev[i % SENSORS_SERVICE_SHM_QVR_DATA_COUNT];
			p->data_ready_cb(p->pCtx, ev);
#endif
			// ALOGD("data ready..\n");
		}
		pos = (i - 1)  % SENSORS_SERVICE_SHM_QVR_DATA_COUNT;
	}

	free(p);
	running = false;
	ALOGD("thread exit!");
	return NULL;
}

int qvr_external_start(data_ready_callback_fn data_ready_cb,
                       handle_error_callback_fn handle_error_cb, void* pCtx) {
	ALOGD("**** qvr_external_start");
	ALOGD("**** data_ready_cb: %p", data_ready_cb);

	qvr_thread_param = (qvr_thread_param_t *) malloc(sizeof(qvr_thread_param_t));
	qvr_thread_param->data_ready_cb = data_ready_cb;
	qvr_thread_param->handle_error_cb = handle_error_cb;
	qvr_thread_param->pCtx = pCtx;
	// pthread_attr_t attr;
	// pthread_attr_init(&attr);

	// wait for thread stoped.
	while (running);
	stop = false;
	pthread_create(&qvr_thread, NULL, (void *(*)(void*))qvr_external_thread, qvr_thread_param);
	return 0;
}

int qvr_external_stop(void) {
	ALOGD("**** qvr_external_stop");
	sensors_service_shm_data_t *shm_data;

	sp<IBinder> binder = defaultServiceManager()->getService(String16(SENSOR_SERVICE_NAME));
	sp<ISharedBuffer> service = ISharedBuffer::asInterface(binder);
	sp<IMemory> buffer = service->getBuffer();
	shm_data = (sensors_service_shm_data_t*)buffer->pointer();
	stop = true;

	ALOGD("**** qvr_external_stop, wait for join thread...");
	pthread_join(qvr_thread, NULL);
	shm_data->sensor_qvr_enable_flag &= ~(SENSORS_SERVICE_SENSOR_FLAG_ACC | SENSORS_SERVICE_SENSOR_FLAG_GYR);
	ALOGD("**** qvr_external_stop, done.");
	return 0;
}

int qvr_external_get_sensor_rate(sensor_type_e type, int* rate) {
	ALOGD("**** qvr_external_get_sensor_rate");
	switch (type) {
	case ACCEL:
	case GYRO:
		*rate = 1000;
		break;
	}
	return 0;
}

int qvr_external_get_sensor_bias(sensor_type_e type, float * bias) {
	ALOGD("**** qvr_external_get_sensor_bias");
	return 0;
}

qvr_external_sensor_ops_t qvr_external_get_sensor_ops =
{
	.Init = qvr_external_init,
	.Deinit = qvr_external_deinit,
	.Start = qvr_external_start,
	.Stop = qvr_external_stop,
	.GetSensorRate = qvr_external_get_sensor_rate,
	.GetSensorBias = qvr_external_get_sensor_bias,
};

qvr_external_sensors_t qvr_external_sensor = {
	.api_version = QVRSERVICEEXTERNALSENSOR_API_VERSION_1,
	.ops = &qvr_external_get_sensor_ops,
};

qvr_external_sensors_t* getInstance(void) {
	return &qvr_external_sensor;
}
