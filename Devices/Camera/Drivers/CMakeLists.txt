# Build file reader driver
set( Build_FileReader true CACHE BOOL force )
if( Build_FileReader )
    add_subdirectory( FileReader )
    add_definitions( -DUSE_FILEREADER )
endif()

# Build webcam driver
set( Build_Webcam true CACHE BOOL force )
if( Build_Webcam )
    add_subdirectory( Webcam )
    add_definitions( -DUSE_WEBCAM )
endif()

# Build HDMI driver
set( Build_HDMI false CACHE BOOL force )
if( Build_HDMI )
    add_subdirectory( HDMI )
    add_definitions( -DUSE_HDMI )
endif()

# Build Epiphan DVI2PCI driver
set( Build_DVI2PCI false CACHE BOOL force )
if( Build_DVI2PCI )
    add_subdirectory( Dvi2Pci )
    add_definitions( -DUSE_DVI2PCI )
endif()

# Build Bumblebee 2 driver (you also need to make sure to edit DriverList.h)
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Bumblebee2 ${CMAKE_MODULE_PATH} )
find_package( DC1394_2 QUIET )
if( DC1394_2_FOUND )
    set( Build_Bumblebee true CACHE BOOL force )
    if( Build_Bumblebee )
        add_subdirectory( Bumblebee2 )
        add_definitions( -DUSE_BUMBLEBEE_2 )
    endif()
endif()

# Build Flycap driver
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Flycap ${CMAKE_MODULE_PATH} )
find_package( FLYCAP QUIET )
if( FLYCAP_FOUND )
    set( Build_Flycap true CACHE BOOL force )
    if( Build_Flycap )
        add_subdirectory( Flycap )
        add_definitions( -DUSE_FLYCAP )
    endif()
endif()


# Build FireFly driver
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/FireFly ${CMAKE_MODULE_PATH} )
find_package( DC1394_2 QUIET )
if( DC1394_2_FOUND )
    set( Build_FireFly true CACHE BOOL force )
    if( Build_FireFly )
        add_subdirectory( FireFly )
        add_definitions( -DUSE_FIREFLY )
    endif()
endif()

# Build Kinect driver if OpenNI is installed
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Kinect ${CMAKE_MODULE_PATH} )
find_package( OpenNI QUIET )
if( OPENNI_FOUND )
    if(CMAKE_COMPILER_IS_GNUCXX)
        # Ignore warnings generated from third party includes
        set_source_files_properties( CameraDriverRegistery.cpp PROPERTIES COMPILE_FLAGS " -Wno-unknown-pragmas -Wno-reorder" )
    endif(CMAKE_COMPILER_IS_GNUCXX)
    set( Build_Kinect true CACHE BOOL force )
    if( Build_Kinect )
        add_subdirectory( Kinect )
        add_definitions( -DUSE_KINECT )
    endif()
endif()

# Build NodeCam driver
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/NodeCam ${CMAKE_MODULE_PATH} )
find_package( ZeroMQ QUIET )
if( ZeroMQ_FOUND )
    find_package( Protobuf QUIET )
    if( PROTOBUF_FOUND )
        set( Build_NodeCam false CACHE BOOL force )
        if( Build_NodeCam )
            add_subdirectory( NodeCam )
            add_definitions( -DUSE_NODECAM )
        endif()
    endif()
endif()

# Build AlliedVision driver (you also need to make sure to edit DriverList.h)
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/AlliedVision ${CMAKE_MODULE_PATH} )
find_package( PVAPI QUIET )
if( PVAPI_FOUND )
    set( Build_AlliedVision false CACHE BOOL force )
    if( Build_AlliedVision )
        add_subdirectory( AlliedVision )
        add_definitions( -DUSE_ALLIEDVISION )
    endif()
endif()

add_library( CameraDriverRegistery CameraDriverRegistery.cpp DriverList.h )

# Build the camera driver registery library
add_to_rpg_libraries( CameraDriverRegistery )
