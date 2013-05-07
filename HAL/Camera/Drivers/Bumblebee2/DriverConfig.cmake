# Here you need to define:
# 1) the class name
# 2) the driver name users will use
# 3) BUILD_DRIVER to true IFF the driver should be built

set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Bumblebee2 ${CMAKE_MODULE_PATH} )
find_package( DC1394_2 REQUIRED )
if( DC1394_2_FOUND )
    set( DRIVER_CLASS "Bumblebe2Driver" )
    set( DRIVER_HEADER "Bumblebe2Driver.h" )
    set( DRIVER_NAME "Bumblebee2" )
    set( BUILD_DRIVER True )
endif()

