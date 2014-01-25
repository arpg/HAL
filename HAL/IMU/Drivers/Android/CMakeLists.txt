# build the library
if(ANDROID)
  find_package(AndroidKernel QUIET)

  if(AndroidKernel_FOUND)
    set(should_build ON)
  else()
    set(should_build OFF)
  endif()
  set(BUILD_ANDROID_IMU ${should_build} CACHE BOOL "Build an IMU driver for recent Android devices")
  if(BUILD_ANDROID_IMU)
    add_to_hal_sources(
      AndroidIMUDriver.h
      AndroidIMUDriver.cpp
      AndroidIMUFactory.cpp)
    add_to_hal_include_dirs(${AndroidKernel_INCLUDE_DIRS})
    add_to_hal_libraries(${AndroidKernel_LIBRARIES})
  endif()
endif()