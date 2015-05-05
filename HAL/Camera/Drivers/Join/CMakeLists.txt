set(BUILD_JoinCamera ON CACHE BOOL "Toggle building the Join Camera driver")
if(BUILD_JoinCamera)
  add_to_hal_sources( JoinCameraDriver.h JoinCameraDriver.cpp JoinCameraFactory.cpp )
endif()
