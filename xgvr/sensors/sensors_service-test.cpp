#define LOG_TAG "3glasses-sensors-service_test"
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
#include <unistd.h>

#include <hardware/hardware.h>
#include <hardware/sensors.h>

#include "sensors_service.h"

#include <sys/types.h>
#define SENSORS_SERVICE_SHM_DATA_EV_COUNT 64 

void _normalize(float *in, float *out) {
	float m = sqrt(in[3] * in[3] + in[0] * in[0] + in[1] * in[1] + in[2] * in[2]);
	out[3] = in[3] / m;
	out[0] = in[0] / m;
	out[1] = in[1] / m;
	out[2] = in[2] / m;
}

void _mul(float *in, double *mul, float *out) {
	float imt[4];
	imt[0] = in[0] * mul[3] + in[1] * mul[2] - in[2] * mul[1] + in[3] * mul[0];
	imt[1] = -in[0] * mul[2] + in[1] * mul[3] + in[2] * mul[0] + in[3] * mul[1];
	imt[2] = in[0] * mul[1] - in[1] * mul[0] + in[2] * mul[3] + in[3] * mul[2];
	imt[3] = -in[0] * mul[0] - in[1] * mul[1] - in[2] * mul[2] + in[3] * mul[3];
	_normalize(imt, out);
}

int main(int argc, char* argv[]) {
	int rv;
	int fd;

	sp<IBinder> binder = defaultServiceManager()->getService(String16(SENSOR_SERVICE_NAME));
	if (binder == NULL)
	{
		printf("Failed to get service: %s.\n", SENSOR_SERVICE_NAME);
		return -1;
	}

	sp<ISharedBuffer> service = ISharedBuffer::asInterface(binder);
	if (service == NULL)
	{
		return -2;
	}

	sp<IMemory> buffer = service->getBuffer();
	if (buffer == NULL)
	{
		return -3;
	}

	sensors_service_shm_data_t* shm_data = (sensors_service_shm_data_t*)buffer->pointer();
	if (shm_data == NULL)
	{
		return -4;
	}

	printf("buffer->pointer(): %p\n", buffer->pointer());
	printf("shm_data->valid_head: %d\n", shm_data->hal_ev_valid_head);
	static int pos = -1;
	int valid_head;

	shm_data->sensor_hal_enable_flag = SENSORS_SERVICE_SENSOR_FLAG_ACC | SENSORS_SERVICE_SENSOR_FLAG_GYR;
	while ((valid_head = shm_data->hal_ev_valid_head) < 0) {
#if 1
		::sched_yield();
#else
		printf("wait for new data\n");
		usleep(100000);
#endif
	}

	while (1) {
		int i;
		sensors_event_t *ev;

		while ((valid_head = shm_data->hal_ev_valid_head) == pos) {
			// printf("wait for ready. pos(%d) valid_head(%d) shm_data->valid_head(%d)\n", pos, valid_head, shm_data->valid_head);
			usleep(10000);
		}
		if (valid_head >= pos) {
			printf("start m1 loop(%d %d).\n", pos, valid_head);
		} else {
			printf("start m2 loop(%d %d).\n", pos, valid_head + SENSORS_SERVICE_SHM_DATA_EV_COUNT);
			valid_head = valid_head + SENSORS_SERVICE_SHM_DATA_EV_COUNT;
		}
		for (i = pos + 1; i <= valid_head; i++) {
                        ev = &shm_data->hal_ev[i % SENSORS_SERVICE_SHM_DATA_EV_COUNT];
			// printf("pos(%d) ev->type(%d) valid_head(%d)\n", pos, ev->type, valid_head);
			if (ev->type == SENSOR_TYPE_ACCELEROMETER) {
				printf("%d: AccX(%f) AccY(%f) AccZ(%f)\n", i % SENSORS_SERVICE_SHM_DATA_EV_COUNT, ev->data[0], ev->data[1], ev->data[2]);
			} else if (ev->type == SENSOR_TYPE_MAGNETIC_FIELD) {
				printf("%d: MagX(%f) MagY(%f) MagZ(%f)\n", i % SENSORS_SERVICE_SHM_DATA_EV_COUNT, ev->data[0], ev->data[1], ev->data[2]);
			} else if (ev->type == SENSOR_TYPE_GYROSCOPE) {
				printf("%d: GyroX(%f) GyroY(%f) GyroZ(%f)\n", i % SENSORS_SERVICE_SHM_DATA_EV_COUNT, ev->data[0], ev->data[1], ev->data[2]);
			} else if (ev->type == SENSOR_TYPE_ROTATION_VECTOR) {
				printf("%d: x(%f) y(%f) z(%f) w(%f)\n", i % SENSORS_SERVICE_SHM_DATA_EV_COUNT, ev->data[0], ev->data[1], ev->data[2], ev->data[3]);
			}
		}
		pos = (i - 1)  % SENSORS_SERVICE_SHM_DATA_EV_COUNT;
		// usleep(500000);
	}
#if 0
	CircleBuff_Cblk_t *cblk = (CircleBuff_Cblk_t *)buffer->pointer(); //步骤四：获取映射的首地址

	fd = open(FIFO_NAME_HAL, O_RDONLY | O_NDELAY);
	printf("I'm reader: %d\n", fd);

	while (1) {
		if (fd == -1) {
			fd = open(FIFO_NAME_HAL, O_RDONLY);
			ALOGD("fd open: %d.", fd);
		}
		if (fd < 0) {
			ALOGD("Wait for fd ready.");
			sleep(1);
			continue;
		}

		sensors_event_t data;
		int rv = read(fd, &data, sizeof(sensors_event_t));
		if (rv == 0 || rv == -1) {
			// try again.
			continue;
		}
		ALOGE("fd read rv(%d).", rv);
		if (rv != sizeof(sensors_event_t)) {
			ALOGE("Data truncated!!");
			continue;
		}

#if 0
		for (int k = 0; k < 64; k++) {
			printf("%02x ", raw_data[k]);
		}
		printf("\n");
#else
		printf("data.type: %d\n", data.type);
#endif
	}
#endif
}
