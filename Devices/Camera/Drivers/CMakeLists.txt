# Build file reader driver
add_subdirectory( FileReader )
#add_dependencies( CameraDriverRegsitery FileReaderDriver )

# Build webcam driver
add_subdirectory( Webcam )

# Build HDMI driver
add_subdirectory( HDMI )

# Build Epiphan DVI2PCI driver
add_subdirectory( Dvi2Pci )

# Build Bumblebee 2 driver (you also need to make sure to edit DriverList.h)
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Bumblebee2 ${CMAKE_MODULE_PATH} )
find_package( DC1394_2 QUIET )
#message( STATUS "GOT ${DC1394_2_LIBRARIES}" )
if( DC1394_2_FOUND )
    add_subdirectory( Bumblebee2 )
    add_definitions( -DUSE_BUMBLEBEE_2 )
endif()

# Build Flycap driver
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Flycap ${CMAKE_MODULE_PATH} )
find_package( FLYCAP QUIET )
#message( STATUS "GOT ${FLYCAP_LIBRARIES}" )
if( FLYCAP_FOUND )
    add_subdirectory( Flycap )
    add_definitions( -DUSE_FLYCAP )
endif()


# Build FireFly driver
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/FireFly ${CMAKE_MODULE_PATH} )
find_package( DC1394_2 QUIET )
#message( STATUS "GOT ${DC1394_2_LIBRARIES}" )
if( DC1394_2_FOUND )
    add_subdirectory( FireFly )
    add_definitions( -DUSE_FIREFLY )
endif()

# Build Kinect driver if OpenNI is installed
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Kinect ${CMAKE_MODULE_PATH} )
find_package( OpenNI QUIET )
if( OPENNI_FOUND )
    if(CMAKE_COMPILER_IS_GNUCXX)
        # Ignore warnings generated from third party includes
        set_source_files_properties( CameraDriverRegistery.cpp PROPERTIES COMPILE_FLAGS " -Wno-unknown-pragmas -Wno-reorder" )
    endif(CMAKE_COMPILER_IS_GNUCXX)
    add_subdirectory( Kinect )
    add_definitions( -DUSE_KINECT )
endif()

# Build NodeCam driver
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/NodeCam ${CMAKE_MODULE_PATH} )
find_package( ZeroMQ QUIET )
if( ZeroMQ_FOUND )
    find_package( Protobuf QUIET )
    if( PROTOBUF_FOUND )
        add_subdirectory( NodeCam )
        add_definitions( -DUSE_NODECAM )
    endif()
endif()

# Build AlliedVision driver (you also need to make sure to edit DriverList.h)
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/AlliedVision ${CMAKE_MODULE_PATH} )
find_package( PVAPI QUIET )
if( PVAPI_FOUND )
    add_subdirectory( AlliedVision )
    add_definitions( -DUSE_ALLIEDVISION )
endif()

add_library( CameraDriverRegistery CameraDriverRegistery.cpp DriverList.h )

# Build the camera driver registery library
add_to_rpg_libraries( CameraDriverRegistery )
