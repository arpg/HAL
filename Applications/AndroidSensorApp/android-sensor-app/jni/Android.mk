LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE  :=  hal
LOCAL_SRC_FILES := /Users/jack/src/rslam/abuild/CoreDev/HAL/HAL/libs/armeabi-v7a/libhal.so
LOCAL_CPPFLAGS += -std=c++11
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE  := andcam
LOCAL_SRC_FILES := /Users/jack/src/rslam/abuild/CoreDev/HAL/Applications/AndroidSensorApp/AndCam/libs/armeabi-v7a/libAndCam.so
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := boostthread
LOCAL_SRC_FILES := /Users/jack/src/android-dev/Toolchain/usr/lib/libboost_thread.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := node
LOCAL_SRC_FILES := /Users/jack/src/rslam/abuild/CoreDev/Node/libs/armeabi-v7a/libnode.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := pbmsgs
LOCAL_SRC_FILES := /Users/jack/src/rslam/abuild/CoreDev/HAL/PbMsgs/libs/armeabi-v7a/libpbmsgs.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := calibu
LOCAL_SRC_FILES := /Users/jack/src/rslam/abuild/CoreDev/Calibu/src/libs/armeabi-v7a/libcalibu.so
include $(PREBUILT_SHARED_LIBRARY)

#include $(CLEAR_VARS)
#LOCAL_MODULE := rightstdc
#LOCAL_SRC_FILES := /Users/jack/src/android-dev/Toolchain/arm-linux-androideabi/lib/armv7-a/libstdc++
#include $(PREBUILD_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencvjava
LOCAL_SRC_FILES := /Users/jack/src/android-dev/Toolchain/usr/lib/libopencv_java.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := gnushared
LOCAL_SRC_FILES := /Users/jack/src/android-dev/Toolchain/arm-linux-androideabi/lib/armv7-a/libgnustl_shared.so 
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := miniglog
LOCAL_SRC_FILES := /Users/jack/src/rslam/abuild/CoreDev/miniglog/libs/armeabi-v7a/libminiglog.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := AndNodeCam
LOCAL_SRC_FILES := AndNodeCam.cpp
LOCAL_C_INCLUDES := /Users/jack/src/rslam/rslam/CoreDev/HAL
LOCAL_C_INCLUDES += /Users/jack/src/rslam/rslam/CoreDev/HAL/PbMsgs
LOCAL_C_INCLUDES += /Users/jack/src/rslam/rslam/CoreDev/
LOCAL_C_INCLUDES += /Users/jack/src/rslam/abuild/CoreDev/Node
LOCAL_C_INCLUDES += /Users/jack/src/rslam/abuild/CoreDev/HAL/
LOCAL_C_INCLUDES += /Users/jack/src/rslam/rslam/CoreDev/miniglog
LOCAL_C_INCLUDES += /Users/jack/src/rslam/abuild
LOCAL_C_INCLUDES += /Users/jack/src/android-dev/Toolchain/user/include
LOCAL_C_INCLUDES += /Users/jack/src/android-dev/Toolchain/include/c++/4.8
LOCAL_C_INCLUDES += /Users/jack/src/android-dev/Toolchain/include/c++/4.8/arm-linux-androideabi
LOCAL_SHARED_LIBRARIES := hal
LOCAL_SHARED_LIBRARIES += node
LOCAL_SHARED_LIBRARIES += gnushared
LOCAL_SHARED_LIBRARIES += opencvjava
LOCAL_SHARED_LIBRARIES += miniglog
LOCAL_SHARED_LIBRARIES += pbmsgs
LOCAL_SHARED_LIBRARIES += calibu
LOCAL_SHARED_LIBRARIES += andcam
LOCAL_STATIC_LIBRARIES := boostthread
#LOCAL_STATIC_LIBRARIES += rightstdc
LOCAL_LDLIBS := -llog
LOCAL_CFLAGS := -D__GXX_EXPERIMENTAL_CXX0X__
LOCAL_CPPFLAGS  := -g -std=c++11
include $(BUILD_SHARED_LIBRARY)
