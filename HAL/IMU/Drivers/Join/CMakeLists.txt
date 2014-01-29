set(BUILD_JoinIMU ON CACHE BOOL "Toggle building the Join IMU driver")
if(BUILD_JoinIMU)
  add_to_hal_sources(JoinDriver.h JoinDriver.cpp JoinFactory.cpp)
endif()
