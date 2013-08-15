set( BUILD_NinjaIMU false CACHE BOOL force )
if( BUILD_NinjaIMU )
    add_to_hal_sources( NinjaIMUDriver.h NinjaIMUDriver.cpp NinjaIMUFactory.cpp FtdiDriver.h )
endif()
