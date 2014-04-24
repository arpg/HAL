project(android-sensor-app)
cmake_minimum_required(VERSION 2.8)

include(def_apk)

find_package(HAL)
find_package(Node)
find_package(PbMsgs)
find_package(MINIGLOG)
find_package(OpenCV2)

include_directories(
  ${MINIGLOG_INCLUDE_DIRS}
  ${HAL_INCLUDE_DIRS}
  ${PbMsgs_INCLUDE_DIRS}
  ${Node_INCLUDE_DIRS}
  )

def_apk(AndroidSensorApp
  DEPENDS hal calibu pbmsgs node miniglog
  LINK_LIBS ${OpenCV2_JAVA_LIBRARY}
  JNI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/jni"
  JNI_SOURCES NativeSensorInterface.cpp
  JNI_LIB NativeSensorInterface
  CONDITIONS OpenCV2_FOUND
  )