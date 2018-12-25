#define LOG_TAG "3glasses-sensors-service"
// #define LOG_NDEBUG  1
#include <utils/Log.h>

#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <linux/uinput.h>
#include <linux/hidraw.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/syscall.h>

#include <cutils/properties.h>

#include <hardware/hardware.h>
#include <hardware/sensors.h>

#include <sys/types.h>
#include <sys/mman.h>

#include <binder/MemoryHeapBase.h>
#include <binder/MemoryBase.h>
#include <binder/IServiceManager.h>
#include <binder/IPCThreadState.h>

#include "hidapi.h"
#include "sensors_service.h"

#define  PIN_CPU_CORE  		1
#define  SCHED_FIFO_CORE  	1

enum {
	SERVICE_STATUS_INIT,
	SERVICE_STATUS_HID_READ,
};

typedef struct {
	platform_t platform;
	char platform_string[20];
	uint16_t vid;
	uint16_t pid;
} hmd_usb_t;

typedef struct {
	union {
		struct {
			uint8_t type;
			uint8_t panel_status;
			uint16_t timestamp;
			uint8_t temperature;
			uint8_t ipd;
			float quat[4];
			float acc[3];
			float gyr[3];
			float mag[3];
			uint8_t touch[2];
			uint8_t als;
			uint8_t button;
		} data;
		uint8_t raw[64];
	} u;
} xgvr_msg_t;

hid_device *hid_handle;
int fifo_hal_fd = -1;
int fifo_qvr_fd = -1;
int service_status = SERVICE_STATUS_INIT;

hmd_usb_t hmd_usb[] = {
	{
		.platform = PLATFORM_S1,
		.platform_string = "s1",
		.vid = 0x2b1c,
		.pid = 0x0001,
	},
	{
		.platform = PLATFORM_D2C,
		.platform_string = "D3V1",
		.vid = 0x2b1c,
		.pid = 0x0200,
	},
	{
		.platform = PLATFORM_D2C,
		.platform_string = "D3V2",
		.vid = 0x2b1c,
		.pid = 0x0201,
	},
	{
		.platform = PLATFORM_D2C,
		.platform_string = "D3C",
		.vid = 0x2b1c,
		.pid = 0x0202,
	},
	{
		.platform = PLATFORM_D2C,
		.platform_string = "D2C",
		.vid = 0x2b1c,
		.pid = 0x0203,
	},
	{
		.platform = PLATFORM_DMICRO,
		.platform_string = "DMICRO",
		.vid = 0x2b1c,
		.pid = 0x0204,
	},
};

uint64_t _get_timestamp() {
	struct timespec t;

	t.tv_sec = t.tv_nsec = 0;
	clock_gettime(CLOCK_BOOTTIME, &t);

	return  (uint64_t(t.tv_sec) * 1000000000LL + t.tv_nsec);
}

class SharedBufferService : public BnSharedBuffer
{
public:
	SharedBufferService()
	{
		sp<MemoryHeapBase> heap = new MemoryHeapBase(sizeof(sensors_service_shm_data_t), 0, SENSOR_SHM_NAME);
		if (heap != NULL)
		{
			mMemory = new MemoryBase(heap, 0, sizeof(sensors_service_shm_data_t));

			int32_t* data = (int32_t*)mMemory->pointer();
			if (data != NULL)
			{
				*data = 0;
			}
		}
	}

	virtual ~SharedBufferService()
	{
		mMemory = NULL;
	}

public:
	static void instantiate()
	{
		defaultServiceManager()->addService(String16(SENSOR_SERVICE_NAME), new SharedBufferService());
	}

	virtual sp<IMemory> getBuffer()
	{
		return mMemory;
	}

private:
	sp<MemoryBase> mMemory;
};

void _common_quat_normalize(float *in, float *out) {
	float m = sqrt(in[3] * in[3] + in[0] * in[0] + in[1] * in[1] + in[2] * in[2]);
	out[3] = in[3] / m;
	out[0] = in[0] / m;
	out[1] = in[1] / m;
	out[2] = in[2] / m;
}

void _common_quat_mul(float *in, double *mul, float *out) {
	float imt[4];
	imt[0] = in[0] * mul[3] + in[1] * mul[2] - in[2] * mul[1] + in[3] * mul[0];
	imt[1] = -in[0] * mul[2] + in[1] * mul[3] + in[2] * mul[0] + in[3] * mul[1];
	imt[2] = in[0] * mul[1] - in[1] * mul[0] + in[2] * mul[3] + in[3] * mul[2];
	imt[3] = -in[0] * mul[0] - in[1] * mul[1] - in[2] * mul[2] + in[3] * mul[3];
	_common_quat_normalize(imt, out);
}

void _3glasses_unpack_sensor_data(const uint8_t* buffer, float* x, float* y, float* z)
{
	struct { int n : 21; } s;
	int X, Y, Z;

	X = s.n = (buffer[0] << 13) | (buffer[1] << 5) | ((buffer[2] & 0xF8) >> 3);
	Y = s.n = ((buffer[2] & 0x07) << 18) | (buffer[3] << 10) | (buffer[4] << 2) |
	          ((buffer[5] & 0xC0) >> 6);
	Z = s.n = ((buffer[5] & 0x3F) << 15) | (buffer[6] << 7) | (buffer[7] >> 1);

	*x = (float)X;
	*y = (float)Y;
	*z = (float)Z;
}

// set fixed cpu
int _common_set_thread_affinity(int coreId)
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
void _common_set_sched_fifo(void)
{
	struct sched_param param = {
		.sched_priority = 98,     /*level range  0-99*/
	};

	if (sched_setscheduler(gettid(), SCHED_FIFO, &param) < 0) {
		ALOGE("set sched_fifo failed");
	}
	_common_set_thread_affinity(SCHED_FIFO_CORE);
}

void _common_emit_keycode(int fd, int type, int code, int val)
{
	struct input_event ie;

	ie.type = type;
	ie.code = code;
	ie.value = val;
	/* timestamp values below are ignored */
	ie.time.tv_sec = 0;
	ie.time.tv_usec = 0;

	write(fd, &ie, sizeof(ie));
}

int main(int argc, char* argv[]) {
	int rv;
	int fd;
	sensors_service_shm_data_t* shm_data;
	static clock_t last_time = clock();
	uint8_t old_key1_status, key1_status;
	uint8_t old_key2_status, key2_status;
	uint8_t old_distance;
	uint8_t buf[128];
	int head;
	float AccX, AccY, AccZ, AccW, RotationX, RotationY, RotationZ, RotationW, GyroX, GyroY, GyroZ, MagX, MagY, MagZ;
	uint8_t distance;
	uint64_t timestamp;
	int hal_ev_pos;
	int qvr_ev_pos;
	uint8_t enbale_flag;
	int bytes_read;
	struct uinput_user_dev uidev;
	int i;

	ALOGD("3Glasses Sensor Service Version(%d.%d) Start.", SENSOR_SERVICE_VERSION_MAJOR, SENSOR_SERVICE_VERSION_MINOR);

	SharedBufferService::instantiate();
	ProcessState::self()->startThreadPool();

	ALOGD("sizeof(sensors_service_shm_data_t): %d.", sizeof(sensors_service_shm_data_t));

	sp<IBinder> binder = defaultServiceManager()->getService(String16(SENSOR_SERVICE_NAME));
	if (binder == NULL)
	{
		ALOGE("Failed to get service: %s.\n", SENSOR_SERVICE_NAME);
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

	shm_data = (sensors_service_shm_data_t*)buffer->pointer();
	if (shm_data == NULL)
	{
		return -4;
	}


#if PIN_CPU_CORE
	_common_set_sched_fifo();
#endif

	ALOGD("shm_data: %p.", shm_data);

	hal_ev_pos = shm_data->hal_ev_valid_head = -1;
	qvr_ev_pos = shm_data->qvr_ev_valid_head = -1;
	shm_data->sensor_hal_enable_flag = 0;
	shm_data->sensor_qvr_enable_flag = 0;

	rv = hid_init();
	if (rv != 0) {
		ALOGE("hid_init fail!! rv(%d)", rv);
	}

	int uinput_fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);

	/*
	 * The ioctls below will enable the device that is about to be
	 * created, to pass key events, in this case the space key.
	 */
	ioctl(uinput_fd, UI_SET_EVBIT, EV_KEY);
	ioctl(uinput_fd, UI_SET_KEYBIT, KEY_ENTER);
	ioctl(uinput_fd, UI_SET_KEYBIT, KEY_BACK);

	memset(&uidev, 0, sizeof(uidev));
	uidev.id.bustype = BUS_USB;
	uidev.id.vendor = 0x0; /* sample vendor */
	uidev.id.product = 0x0; /* sample product */
	snprintf(uidev.name, UINPUT_MAX_NAME_SIZE, "3lgasses-vkey");
	write(uinput_fd, &uidev, sizeof(uidev));
	ioctl(uinput_fd, UI_DEV_CREATE);

	while (1) {
#if 0 // debug, fake data.
		sensors_event_t *ev;
		uint64_t timestamp;
		int pos = shm_data->valid_head;

		timestamp = _get_timestamp();

		pos = (pos + 1) % SENSORS_SERVICE_SHM_DATA_EV_COUNT;
		ev = &shm_data->hal_ev[pos];
		ev->version = sizeof(sensors_event_t);
		ev->sensor = SENSORS_SERVICE_ID_ACC;
		ev->type = SENSOR_TYPE_ACCELEROMETER;
		ev->acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;
		ev->timestamp = timestamp;
		ev->data[0] = 0.1f;
		ev->data[1] = 0.2f;
		ev->data[2] = 0.3f;
		shm_data->valid_head = pos;

#if PLATFORM_S1
		pos = (pos + 1) % SENSORS_SERVICE_SHM_DATA_EV_COUNT;
		ev = &shm_data->hal_ev[pos];
		ev->version = sizeof(sensors_event_t);
		ev->sensor = SENSORS_SERVICE_ID_MAG;
		ev->type = SENSOR_TYPE_MAGNETIC_FIELD;
		ev->acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;
		ev->timestamp = timestamp;
		ev->data[0] = 0.1f;
		ev->data[1] = 0.2f;
		ev->data[2] = 0.3f;
		shm_data->valid_head = pos;
#endif

		pos = (pos + 1) % SENSORS_SERVICE_SHM_DATA_EV_COUNT;
		ev = &shm_data->hal_ev[pos];
		ev->version = sizeof(sensors_event_t);
		ev->sensor = SENSORS_SERVICE_ID_GYR;
		ev->type = SENSOR_TYPE_GYROSCOPE;
		ev->acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;
		ev->timestamp = timestamp;
		ev->data[0] = 0.1f;
		ev->data[1] = 0.2f;
		ev->data[2] = 0.3f;
		shm_data->valid_head = pos;

#if PLATFORM_D2C
		pos = (pos + 1) % SENSORS_SERVICE_SHM_DATA_EV_COUNT;
		ev = &shm_data->hal_ev[pos];
		ev->version = sizeof(sensors_event_t);
		ev->sensor = SENSORS_SERVICE_ID_RV;
		ev->type = SENSOR_TYPE_ROTATION_VECTOR;
		ev->acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;
		ev->timestamp = timestamp;
		ev->data[0] = 0.1f;
		ev->data[1] = 0.2f;
		ev->data[2] = 0.3f;
		ev->data[3] = 0.9f;
		shm_data->valid_head = pos;
#endif

		usleep(1000000 / 500);
#else
		switch (service_status) {
		case SERVICE_STATUS_INIT:
		{
			sleep(1);
			hid_handle = NULL;

			// try to open HID
			for (i = 0; i < sizeof(hmd_usb) / sizeof(hmd_usb_t); i++) {
				hid_handle = hid_open(hmd_usb[i].vid, hmd_usb[i].pid, NULL);
				if (hid_handle) {
					shm_data->platform = hmd_usb[i].platform;
					property_set("persist.3box.hmdtype", hmd_usb[i].platform_string);
					ALOGD("hid(%x:%x) hid_handle(%x) %s opened.", hmd_usb[i].vid, hmd_usb[i].pid, hid_handle, hmd_usb[i].platform_string);
					break;
				}
			}

			if (!hid_handle) {
				property_set("persist.3box.hmdtype", "none");
				ALOGD("no HMD exist.");
				continue;
			}

			service_status = SERVICE_STATUS_HID_READ;

			old_key1_status = 0;
			old_key2_status = 0;
			old_distance = -1;
		}
		break;
		case SERVICE_STATUS_HID_READ:
		{
#if 0
			enbale_flag = 0xffff;
#else
			// ALOGD("sensor_hal_enable_flag: %08x, sensor_qvr_enable_flag: %08x\n", shm_data->sensor_hal_enable_flag, shm_data->sensor_qvr_enable_flag);
			enbale_flag = shm_data->sensor_hal_enable_flag | shm_data->sensor_qvr_enable_flag;
			// if (!enbale_flag) {
			// 	ALOGD("wait for trigger");
			// 	sleep(1);
			// 	continue;
			// }
#endif
			bytes_read = hid_read(hid_handle, buf, sizeof(buf));
			if (bytes_read < 0) {
				ALOGE("read bytes from hidraw failed...");
				hid_handle = NULL;
				service_status = SERVICE_STATUS_INIT;
				break;
			} else {
				// ALOGD("bytes_read: %d", bytes_read);
			}
#if 0
			ALOGD("read bytes_read(%d).", bytes_read);
			for (int k = 0; k < 64; k++) {
				printf("%02x ", buf[k]);
			}
			printf("\n");
#endif
			timestamp = _get_timestamp();

			if (shm_data->platform == PLATFORM_S1) {
				if (bytes_read < 64) {
					ALOGE("Data truncated!! bytes_read(%d)", bytes_read);
					sleep(1);
					continue;
				}
				float a[3], g[3];
				_3glasses_unpack_sensor_data(&buf[24], &a[0], &a[1], &a[2]);
				a[0] = a[0];
				a[1] = a[1];
				a[2] = a[2];
				AccX = a[1] / 2048 * GRAVITY_EARTH;
				AccY = -a[0] / 2048 * GRAVITY_EARTH;
				AccZ = a[2] / 2048 * GRAVITY_EARTH;

#if 0
				MagX = buf[56] | (buf[57] << 16);
				MagY = buf[58] | (buf[59] << 16);
				MagZ = buf[60] | (buf[61] << 16);
#endif
				_3glasses_unpack_sensor_data(&buf[32], &g[0], &g[1], &g[2]);
				GyroX = g[1] / 16.4 * M_PI / 180.0;
				GyroY = -g[0] / 16.4 * M_PI / 180.0;
				GyroZ = g[2] / 16.4 * M_PI / 180.0;

				if (buf[1] & 0x80) {
					distance = 0;
				} else {
					distance = 5; // assume 5cm
				}

			} else if (shm_data->platform == PLATFORM_D2C) {
				if (bytes_read < sizeof(xgvr_msg_t)) {
					ALOGE("Data truncated!! bytes_read(%d)", bytes_read);
					sleep(1);
					continue;
				}
				float a[3], g[3];
				float _quat[4], inter_quat[4];
				double mul[4] = { 0, 0.70710548251, 0, 0.70710548251 };

				xgvr_msg_t *msg = (xgvr_msg_t *)buf;
#if 0			// v1.x
				a[0] = msg->u.data.acc[0];
				a[1] = msg->u.data.acc[1];
				a[2] = msg->u.data.acc[2];
				AccX = a[1];
				AccY = a[0];
				AccZ = a[2];

				g[0] = msg->u.data.gyr[0];
				g[1] = msg->u.data.gyr[1];
				g[2] = msg->u.data.gyr[2];
				GyroX = g[1];
				GyroY = g[0];
				GyroZ = g[2];
#else			// v2.x
				a[0] = msg->u.data.acc[0];
				a[1] = msg->u.data.acc[1];
				a[2] = msg->u.data.acc[2];
				AccX = a[0];
				AccY = a[1];
				AccZ = a[2];

				g[0] = msg->u.data.gyr[0];
				g[1] = msg->u.data.gyr[1];
				g[2] = msg->u.data.gyr[2];
				GyroX = g[0];
				GyroY = g[1];
				GyroZ = g[2];
#endif

				_quat[0] = msg->u.data.quat[0];
				_quat[1] = msg->u.data.quat[1];
				_quat[2] = msg->u.data.quat[2];
				_quat[3] = msg->u.data.quat[3];
#if 0
				_common_quat_mul(_quat, mul, inter_quat);
				RotationX = inter_quat[0];
				RotationY = inter_quat[1];
				RotationZ = inter_quat[2];
				RotationW = inter_quat[3];
#else
				RotationX = _quat[0];
				RotationY = _quat[1];
				RotationZ = _quat[2];
				RotationW = _quat[3];
#endif
				key1_status = (msg->u.data.button & (1 << 0)) ? 1 : 0;
				key2_status = (msg->u.data.button & (1 << 1)) ? 1 : 0;

				if (msg->u.data.als & 0x1) {
					distance = 0;
				} else {
					distance = 5; // assume 5cm
				}
			} else if (shm_data->platform == PLATFORM_DMICRO) {
				if (bytes_read < sizeof(xgvr_msg_t)) {
					ALOGE("Data truncated!! bytes_read(%d)", bytes_read);
					sleep(1);
					continue;
				}
				float a[3], g[3];
				float _quat[4], inter_quat[4];
				double mul[4] = {0, -0.70710678118, 0, 0.70710678118};

				xgvr_msg_t *msg = (xgvr_msg_t *)buf;
				a[0] = msg->u.data.acc[0];
				a[1] = msg->u.data.acc[1];
				a[2] = msg->u.data.acc[2];
				AccX = -a[0];
				AccY = -a[1];
				AccZ = a[2];

				g[0] = msg->u.data.gyr[0];
				g[1] = msg->u.data.gyr[1];
				g[2] = msg->u.data.gyr[2];
				GyroX = -g[0];
				GyroY = -g[1];
				GyroZ = g[2];

				_quat[0] = msg->u.data.quat[0];
				_quat[1] = msg->u.data.quat[1];
				_quat[2] = msg->u.data.quat[2];
				_quat[3] = msg->u.data.quat[3];
#if 0
				_common_quat_mul(_quat, mul, inter_quat);
				RotationX = inter_quat[0];
				RotationY = inter_quat[1];
				RotationZ = inter_quat[2];
				RotationW = inter_quat[3];
#else
				RotationX = _quat[0];
				RotationY = _quat[1];
				RotationZ = _quat[2];
				RotationW = _quat[3];
#endif
				key1_status = (msg->u.data.button & (1 << 0)) ? 1 : 0;
				key2_status = (msg->u.data.button & (1 << 1)) ? 1 : 0;

				if (msg->u.data.als & 0x1) {
					distance = 0;
				} else {
					distance = 5; // assume 5cm
				}
			}
			// ALOGD("AccX(%f) AccY(%f) AccZ(%f)\n", AccX, AccY, AccZ);
			// ALOGD("GyroX(%f) GyroY(%f) GyroZ(%f)\n", GyroX, GyroY, GyroZ);
			// ALOGD("MagX(%f) MagY(%f) MagZ(%f)\n", MagX, MagY, MagZ);
			// ALOGD("proximity distance(%d)\n", distance);

			if (enbale_flag & SENSORS_SERVICE_SENSOR_FLAG_ACC) {
				if (shm_data->sensor_qvr_enable_flag & SENSORS_SERVICE_SENSOR_FLAG_ACC) {
					sensor_sample_t *ev;
					qvr_ev_pos = (qvr_ev_pos + 1) % SENSORS_SERVICE_SHM_QVR_DATA_COUNT;
					ev = &shm_data->qvr_ev[qvr_ev_pos];
					ev->type = ACCEL;
					ev->ts = timestamp;
					ev->sample[0] = AccX;
					ev->sample[1] = AccY;
					ev->sample[2] = AccZ;
				} else if (shm_data->sensor_hal_enable_flag & SENSORS_SERVICE_SENSOR_FLAG_ACC) {
					sensors_event_t *ev;
					hal_ev_pos = (hal_ev_pos + 1) % SENSORS_SERVICE_SHM_HAL_DATA_COUNT;
					ev = &shm_data->hal_ev[hal_ev_pos];
					ev->version = sizeof(sensors_event_t);
					ev->sensor = SENSORS_SERVICE_ID_ACC;
					ev->type = SENSOR_TYPE_ACCELEROMETER;
					ev->acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;
					ev->timestamp = timestamp;
					ev->data[0] = AccX;
					ev->data[1] = AccY;
					ev->data[2] = AccZ;
				}
			}
#if 0
			if (enbale_flag & SENSORS_SERVICE_SENSOR_FLAG_MAG) {
				sensors_event_t *ev;
				hal_ev_pos = (hal_ev_pos + 1) % SENSORS_SERVICE_SHM_HAL_DATA_COUNT;
				ev = &shm_data->hal_ev[hal_ev_pos];
				ev->version = sizeof(sensors_event_t);
				ev->sensor = SENSORS_SERVICE_ID_MAG;
				ev->type = SENSOR_TYPE_MAGNETIC_FIELD;
				ev->acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;
				ev->timestamp = timestamp;
				ev->data[0] = MagX;
				ev->data[1] = MagY;
				ev->data[2] = MagZ;
			}
#endif
			if (enbale_flag & SENSORS_SERVICE_SENSOR_FLAG_GYR) {
				if (shm_data->sensor_qvr_enable_flag & SENSORS_SERVICE_SENSOR_FLAG_GYR) {
					sensor_sample_t *ev;
					qvr_ev_pos = (qvr_ev_pos + 1) % SENSORS_SERVICE_SHM_QVR_DATA_COUNT;
					ev = &shm_data->qvr_ev[qvr_ev_pos];
					ev->type = GYRO;
					ev->ts = timestamp;
					ev->sample[0] = GyroX;
					ev->sample[1] = GyroY;
					ev->sample[2] = GyroZ;
				} else if (shm_data->sensor_hal_enable_flag & SENSORS_SERVICE_SENSOR_FLAG_GYR) {
					sensors_event_t *ev;
					hal_ev_pos = (hal_ev_pos + 1) % SENSORS_SERVICE_SHM_HAL_DATA_COUNT;
					ev = &shm_data->hal_ev[hal_ev_pos];
					ev->version = sizeof(sensors_event_t);
					ev->sensor = SENSORS_SERVICE_ID_GYR;
					ev->type = SENSOR_TYPE_GYROSCOPE;
					ev->acceleration.status = SENSOR_STATUS_ACCURACY_HIGH; // DUNCAN_TODO
					ev->timestamp = timestamp;
					ev->data[0] = GyroX;
					ev->data[1] = GyroY;
					ev->data[2] = GyroZ;
				}
			}
			if (shm_data->sensor_qvr_enable_flag & SENSORS_SERVICE_SENSOR_FLAG_RV) {
				;
			} else if (shm_data->sensor_hal_enable_flag & SENSORS_SERVICE_SENSOR_FLAG_GYR) {
				sensors_event_t *ev;
				hal_ev_pos = (hal_ev_pos + 1) % SENSORS_SERVICE_SHM_HAL_DATA_COUNT;
				ev = &shm_data->hal_ev[hal_ev_pos];
				ev->version = sizeof(sensors_event_t);
				ev->sensor = SENSORS_SERVICE_ID_RV;
				ev->type = SENSOR_TYPE_ROTATION_VECTOR;
				ev->acceleration.status = SENSOR_STATUS_ACCURACY_HIGH; // DUNCAN_TODO
				ev->timestamp = timestamp;
				ev->data[0] = RotationX;
				ev->data[1] = RotationY;
				ev->data[2] = RotationZ;
				ev->data[3] = RotationW;
			}
			if (enbale_flag & SENSORS_SERVICE_SENSOR_FLAG_P) {
				if (distance != old_distance) {
					sensors_event_t *ev;
					old_distance = distance;
					hal_ev_pos = (hal_ev_pos + 1) % SENSORS_SERVICE_SHM_HAL_DATA_COUNT;
					ev = &shm_data->hal_ev[hal_ev_pos];
					ev->version = sizeof(sensors_event_t);
					ev->sensor = SENSORS_SERVICE_ID_PROXMITY;
					ev->type = SENSOR_TYPE_PROXIMITY; // DUNCAN_TODO
					ev->acceleration.status = SENSOR_STATUS_ACCURACY_HIGH; // DUNCAN_TODO
					ev->timestamp = timestamp;
					ev->data[0] = distance;
				}
			}
			shm_data->hal_ev_valid_head = hal_ev_pos;
			shm_data->qvr_ev_valid_head = qvr_ev_pos;
			sched_yield();
			// ALOGD("hal_ev_valid_head(%d) qvr_ev_valid_head(%d)\n", shm_data->hal_ev_valid_head, shm_data->qvr_ev_valid_head);

#if 1
			if (key1_status != old_key1_status) {
				ALOGD("key1_status(%d)\n", key1_status);
				if (key1_status) {
					_common_emit_keycode(uinput_fd, EV_KEY, KEY_ENTER, 1);
				} else {
					_common_emit_keycode(uinput_fd, EV_KEY, KEY_ENTER, 0);
				}
				_common_emit_keycode(uinput_fd, EV_SYN, SYN_REPORT, 0);
				old_key1_status = key1_status;
			}
			if (key2_status != old_key2_status) {
				ALOGD("key2_status(%d)\n", key2_status);
				if (key2_status) {
					_common_emit_keycode(uinput_fd, EV_KEY, KEY_BACK, 1);
				} else {
					_common_emit_keycode(uinput_fd, EV_KEY, KEY_BACK, 0);
				}
				_common_emit_keycode(uinput_fd, EV_SYN, SYN_REPORT, 0);
				old_key2_status = key2_status;
			}
#endif

#if 1
			// force delay to avoid system crach
			usleep(SENSOR_SERVICE_HID_GET_DATA_DELAY_US);
#endif
		}
		break;
		}
#endif
	}
	ioctl(uinput_fd, UI_DEV_DESTROY);

	ALOGD("EXIT!!");
	hid_exit();
	IPCThreadState::self()->joinThreadPool();
}
