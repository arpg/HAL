# Build file reader driver
add_subdirectory( FileReader )
#add_dependencies( CameraDriverRegsitery FileReaderDriver )

# Build webcam driver
add_subdirectory( Webcam )

# Build Bumblebee 2 driver (you also need to make sure to edit DriverList.h)
#set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Bumblebee2 ${CMAKE_MODULE_PATH} )
# find_package( DC1394_2 REQUIRED )
## message( STATUS "GOT ${DC1394_2_LIBRARIES}" )
#if( DC1394_2_FOUND )
#    add_subdirectory( Bumblebee2 )
#    add_definitions( -DUSE_BUMBLEBEE_2 )
#endif()

add_library( CameraDriverRegsitery CameraDriverRegistery.cpp DriverList.h )

# Build the camera driver registery library
add_to_rpg_libraries( CameraDriverRegsitery )


