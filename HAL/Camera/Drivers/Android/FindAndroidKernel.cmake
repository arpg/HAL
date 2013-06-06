# FindAndroidKernel.cmake
#   AndroidKernel_FOUND
#   AndroidKernel_INCLUDE_DIRS

message(STATUS "PREFIX: ${CMAKE_PREFIX_PATH}")

SET(AndroidKernel_POSSIBLE_ROOT_DIRS
    "$ENV{AndroidKernel_ROOT_DIR}"                      # *NIX: custom install location (like ROS)
        /usr/local                                      # Linux: default dir by CMake
        /usr                                            # Linux
        /opt/local                                      # OS X: default MacPorts location
        ${CMAKE_INSTALL_PREFIX}                         # Android toolchain
)

FIND_PATH(AndroidKernel_AV_INCLUDE_DIR
          NAMES camera/ICamera.h gestures/IGestureDevice.h media/IMediaPlayer.h
          PATHS ${AndroidKernel_POSSIBLE_ROOT_DIRS})
FIND_PATH(AndroidKernel_NATIVE_INCLUDE_DIR
          NAMES android/window.h ui/ANativeObjectBase.h
          PATHS ${AndroidKernel_POSSIBLE_ROOT_DIRS})
FIND_PATH(AndroidKernel_HARDWARE_INCLUDE_DIR
          NAMES hardware/camera.h hardware/lights.h
          PATHS ${AndroidKernel_POSSIBLE_ROOT_DIRS})
FIND_PATH(AndroidKernel_CORE_INCLUDE_DIR
          NAMES android/log.h ctest/ctest.h system/camera.h
          PATHS ${AndroidKernel_POSSIBLE_ROOT_DIRS})


SET(AndroidKernel_INCLUDE_DIRS
    ${AndroidKernel_AV_INCLUDE_DIR}
    ${AndroidKernel_NATIVE_INCLUDE_DIR}
    ${AndroidKernel_HARDWARE_INCLUDE_DIR}
    ${AndroidKernel_CORE_INCLUDE_DIR}
    )

SET(Raisin_FOUND ON)

FOREACH(NAME ${AndroidKernel_INCLUDE_DIRS})
    IF(NOT EXISTS ${NAME})
        SET(AndroidKernel_FOUND OFF)
    ENDIF(NOT EXISTS ${NAME})
ENDFOREACH(NAME)

MARK_AS_ADVANCED(FORCE
                 AndroidKernel_ROOT_DIR
                 AndroidKernel_AV_INCLUDE_DIR
                 AndroidKernel_NATIVE_INCLUDE_DIR
                 AndroidKernel_HARDWARE_INCLUDE_DIR
                 AndroidKernel_CORE_INCLUDE_DIR
                 )

IF(AndroidKernel_FOUND)
   IF(NOT AndroidKernel_FIND_QUIETLY)
      MESSAGE(STATUS "Found Android Kernel: ${AndroidKernel_INCLUDE_DIRS}")
   ENDIF (NOT AndroidKernel_FIND_QUIETLY)
ELSE(AndroidKernel_FOUND)
   IF(AndroidKernel_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find AndroidKernel. Please specify it's location with the AnroidKernel_ROOT_DIR env. variable.")
   ENDIF(AndroidKernel_FIND_REQUIRED)
ENDIF(AndroidKernel_FOUND)
