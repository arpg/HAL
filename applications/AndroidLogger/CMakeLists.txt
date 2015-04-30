project(AndroidLogger)
cmake_minimum_required(VERSION 2.8)

include(def_apk)

find_library(GLES_LIBRARY NAMES GLESv2)

find_package(GLog QUIET)
find_package(OpenCV2)

include_directories(${Messages_INCLUDE_DIRS} ${GLog_INCLUDE_DIRS})

if(GLOG_FOUND AND OpenCV2_FOUND)
  def_apk(AndroidLogger
    JNI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/jni"
    JNI_SOURCES NativeSensorInterface.cpp NativeOpenGLRenderer.cpp
    JNI_LIB ARPGNativeInterface
    LINK_LIBS ${GLES_LIBRARY} ${OpenCV2_JAVA_LIBRARY}
    DEPENDS messages miniglog
    )
endif()
