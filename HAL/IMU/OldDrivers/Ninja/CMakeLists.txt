set( BUILD_Ninja false CACHE BOOL force )
if( BUILD_Ninja )
    add_to_hal_sources( NinjaIMUDriver.h NinjaIMUDriver.cpp NinjaIMUFactory.cpp FtdiListener.h )
endif()
