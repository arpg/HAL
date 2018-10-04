message(STATUS "HAL: building 'AutoExposure' camera driver")

add_to_hal_sources(
  AutoExposureDriver.h
  AutoExposureDriver.cpp
  AutoExposureFactory.cpp
)