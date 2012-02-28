# Here you need to define:
# 1) the class name
# 2) the driver name users will use
# 3) BUILD_DRIVER to true IFF the driver should be built

set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/SimCam ${CMAKE_MODULE_PATH} )
find_package( OGRE REQUIRED )
if( OGRE_FOUND )
    set( DRIVER_CLASS "SimCamDriver" )
    set( DRIVER_HEADER "SimCamDriver.h" )
    set( DRIVER_NAME "SimCam" )
    set( BUILD_DRIVER True )
endif()

