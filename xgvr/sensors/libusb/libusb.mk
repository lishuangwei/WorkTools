# Android build config for libusb
# Copyright © 2012-2013 RealVNC Ltd. <toby.gray@realvnc.com>
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
#
include $(CLEAR_VARS)

LIBUSB_ROOT_REL:= libusb
LIBUSB_ROOT_ABS:= $(LOCAL_PATH)/libusb

LOCAL_SRC_FILES := \
  $(LIBUSB_ROOT_REL)/core.c \
  $(LIBUSB_ROOT_REL)/descriptor.c \
  $(LIBUSB_ROOT_REL)/io.c \
  $(LIBUSB_ROOT_REL)/sync.c \
  $(LIBUSB_ROOT_REL)/os/linux_usbfs.c \
  $(LIBUSB_ROOT_REL)/os/threads_posix.c

LOCAL_C_INCLUDES += \
  $(LOCAL_PATH)/libusb \
  $(LIBUSB_ROOT_ABS)/libusb \
  $(LIBUSB_ROOT_ABS)/libusb/os

LOCAL_EXPORT_C_INCLUDES := \
  $(LIBUSB_ROOT_ABS)/libusb

LOCAL_LDLIBS := -llog
LOCAL_CFLAGS := -DLIBUSB_DESCRIBE=\"1.0.9-28-g7634714\"
LOCAL_MODULE := libusb1.0

include $(BUILD_SHARED_LIBRARY)