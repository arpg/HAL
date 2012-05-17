# Here you need to define:
# 1) the class name
# 2) the driver name users will use
# 3) BUILD_DRIVER to true IFF the driver should be built

set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/AlliedVision ${CMAKE_MODULE_PATH} )
find_package( PVAPI REQUIRED )
if( PVAPI_FOUND )
    set( DRIVER_CLASS "AlliedVisionDriver" )
    set( DRIVER_HEADER "AlliedVisionDriver.h" )
    set( DRIVER_NAME "AlliedVision" )
    set( BUILD_DRIVER True )
endif()
