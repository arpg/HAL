# Here you need to define:
# 1) the class name
# 2) the driver name users will use
# 3) BUILD_DRIVER to true IFF the driver should be built

set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/FireFly ${CMAKE_MODULE_PATH} )
find_package( DC1394_2 REQUIRED )
if( DC1394_2_FOUND )
    set( DRIVER_CLASS "FireFlyDriver" )
    set( DRIVER_HEADER "FireFlyDriver.h" )
    set( DRIVER_NAME "FireFly" )
    set( BUILD_DRIVER True )
endif()

