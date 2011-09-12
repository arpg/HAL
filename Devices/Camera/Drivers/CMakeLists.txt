add_library( CameraDriverRegsitery CameraDriverRegistery.cpp )

# Build file reader driver
add_subdirectory( FileReader )
#add_dependencies( CameraDriverRegsitery FileReaderDriver )

# Build Bumblebee 2 driver
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Bumblebee2 ${CMAKE_MODULE_PATH} )
find_package( DC1394_2 REQUIRED )
if( DC1394_2_FOUND )
    add_definitions( -DUSE_BUMBLEBEE_2 )
    add_subdirectory( Bumblebee2 )
endif()

# Build the camera driver registery library
add_to_rpg_libraries( CameraDriverRegsitery )


