set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Kinect ${CMAKE_MODULE_PATH} )

find_package( OpenNI QUIET )
if( OPENNI_FOUND )
    add_definitions( -DUSE_KINECT )
    add_to_hal_sources(KinectDriver.cpp KinectDriver.h )
endif()
