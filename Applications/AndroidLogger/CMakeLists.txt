project(AndroidLogger)
cmake_minimum_required(VERSION 2.8)

include(def_apk)

find_library(GLES_LIBRARY NAMES GLESv2)

def_apk(AndroidLogger
  JNI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/jni"
  JNI_SOURCES NativeSensorInterface.cpp NativeOpenGLRenderer.cpp
  JNI_LIB ARPGNativeInterface
  LINK_LIBS ${GLES_LIBRARY}
  )