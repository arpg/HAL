set( BUILD_Velodyne true CACHE BOOL force )
if( BUILD_Velodyne )
    add_to_hal_sources( VelodyneDriver.h VelodyneDriver.cpp VelodyneFactory.cpp )
endif()
