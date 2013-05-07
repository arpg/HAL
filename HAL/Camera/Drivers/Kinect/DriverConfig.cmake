# Here you need to define:
# 1) the class name
# 2) the driver name users will use
# 3) BUILD_DRIVER to true IFF the driver should be built

set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Kinect ${CMAKE_MODULE_PATH} )
find_package( OpenNI REQUIRED )
if( OPENNI_FOUND )
    set( DRIVER_CLASS "KinectDriver" )
    set( DRIVER_HEADER "KinectDriver.h" )
    set( DRIVER_NAME "Kinect" )
    set( BUILD_DRIVER True )
endif()