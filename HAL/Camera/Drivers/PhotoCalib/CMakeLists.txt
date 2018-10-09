set(BUILD_PhotoCalib OFF CACHE BOOL "Build the PhotoCalib Camera driver")

if(BUILD_PhotoCalib)

  message(STATUS "HAL: building 'PhotoCalib' camera driver.")

  add_to_hal_sources(
    PhotoCalibDriver.h
    PhotoCalibDriver.cpp
    PhotoCalibFactory.cpp
  )

endif()