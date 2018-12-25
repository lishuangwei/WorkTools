LOCAL_PATH := $(call my-dir)

# $(TARGET_DEVICE)

# Example Sensors HAL implementation.
include $(CLEAR_VARS)
LOCAL_MODULE := sensors.3glasses
# LOCAL_MODULE_RELATIVE_PATH := hw
# LOCAL_MODULE_TAGS := optional
LOCAL_CFLAGS := -Wno-unused-parameter
LOCAL_C_INCLUDES += \
	vr-api/LINUX/android/vendor/qcom/proprietary/qvr/inc
LOCAL_SRC_FILES := \
  sensors_hal.cpp \
  ISharedBuffer.cpp
LOCAL_SHARED_LIBRARIES := liblog libbinder libutils
LOCAL_MODULE_OWNER := 3glasses
LOCAL_PROPRIETARY_MODULE := true
include $(BUILD_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := sensors-qvr_3glasses
LOCAL_CFLAGS := -Wno-unused-parameter
LOCAL_C_INCLUDES += \
	vr-api/LINUX/android/vendor/qcom/proprietary/qvr/inc
LOCAL_SRC_FILES := \
  sensors_qvr.cpp \
  ISharedBuffer.cpp
LOCAL_SHARED_LIBRARIES := liblog libbinder libutils
include $(BUILD_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := sensors-hal-example-app
LOCAL_SRC_FILES := hal-example-app.cpp
LOCAL_CFLAGS := -Wno-unused-parameter
LOCAL_C_INCLUDES += \
	vr-api/LINUX/android/vendor/qcom/proprietary/qvr/inc
LOCAL_SHARED_LIBRARIES := libhardware
include $(BUILD_EXECUTABLE)

include $(CLEAR_VARS)
LOCAL_MODULE := sensors-service_3glasses
LOCAL_CFLAGS := -Wno-unused-parameter
LOCAL_C_INCLUDES += \
	vr-api/LINUX/android/vendor/qcom/proprietary/qvr/inc \
	$(LOCAL_PATH)/libusb
LOCAL_SRC_FILES := \
  sensors_service.cpp \
  ISharedBuffer.cpp \
  hid.cpp
LOCAL_SHARED_LIBRARIES := liblog libbinder libutils libusb1.0 libcutils
include $(BUILD_EXECUTABLE)

 include $(CLEAR_VARS)
 LOCAL_MODULE := sensors-service_3glasses-test
 LOCAL_CFLAGS := -Wno-unused-parameter
 LOCAL_C_INCLUDES += \
 	vr-api/LINUX/android/vendor/qcom/proprietary/qvr/inc
 LOCAL_SRC_FILES := \
   sensors_service-test.cpp \
   ISharedBuffer.cpp
 LOCAL_SHARED_LIBRARIES := liblog libbinder libutils
 include $(BUILD_EXECUTABLE)

include $(LOCAL_PATH)/libusb/libusb.mk
