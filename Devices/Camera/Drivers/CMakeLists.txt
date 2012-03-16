# Build file reader driver
add_subdirectory( FileReader )
#add_dependencies( CameraDriverRegsitery FileReaderDriver )

# Build webcam driver
add_subdirectory( Webcam )

# Build Bumblebee 2 driver (you also need to make sure to edit DriverList.h)
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Bumblebee2 ${CMAKE_MODULE_PATH} )
find_package( DC1394_2 QUIET )
#message( STATUS "GOT ${DC1394_2_LIBRARIES}" )
if( DC1394_2_FOUND )
    add_subdirectory( Bumblebee2 )
    add_definitions( -DUSE_BUMBLEBEE_2 )
endif()

# Build Kinect driver if OpenNI is installed
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Kinect ${CMAKE_MODULE_PATH} )
find_package( OpenNI QUIET )
if( OPENNI_FOUND )
    add_subdirectory( Kinect )
endif()

# Build SimCam driver
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/SimCam ${CMAKE_MODULE_PATH} )
#find_package( OGRE QUIET )
#message( STATUS "GOT ${OGRE_LIBRARIES}" )
#if( OGRE_FOUND )
    add_subdirectory( SimCam )
#endif()


add_library( CameraDriverRegistery CameraDriverRegistery.cpp DriverList.h )

# Build the camera driver registery library
add_to_rpg_libraries( CameraDriverRegistery )