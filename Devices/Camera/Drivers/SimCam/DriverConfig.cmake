# Here you need to define:
# 1) the class name
# 2) the driver name users will use
# 3) BUILD_DRIVER to true IFF the driver should be built

set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/NodeCam ${CMAKE_MODULE_PATH} )
find_package( ZeroMQ REQUIRED )
if( ZeroMQ_FOUND )
    set( DRIVER_CLASS "NodeCamDriver" )
    set( DRIVER_HEADER "NodeCamDriver.h" )
    set( DRIVER_NAME "NodeCam" )
    set( BUILD_DRIVER True )
endif()

