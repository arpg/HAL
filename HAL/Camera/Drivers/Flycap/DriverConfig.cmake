# Here you need to define:
# 1) the class name
# 2) the driver name users will use
# 3) BUILD_DRIVER to true IFF the driver should be built

set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Flycap ${CMAKE_MODULE_PATH} )
find_package( Flycap REQUIRED )
if( FLYCAP_FOUND )
    set( DRIVER_CLASS "FlycapDriver" )
    set( DRIVER_HEADER "FlycapDriver.h" )
    set( DRIVER_NAME "Flycap" )
    set( BUILD_DRIVER True )
endif()

