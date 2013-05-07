# Build MicroStrain IMU driver
set( Build_MicroStrain true CACHE BOOL force )
if( Build_MicroStrain )
    add_subdirectory( MicroStrain )
    add_definitions( -DUSE_MICROSTRAIN )
endif()

# Build IMULog driver
set( Build_IMULog true CACHE BOOL force )
if( Build_IMULog )
    add_subdirectory( IMULog )
    add_definitions( -DUSE_IMULOG )
endif()

# Build Phidgets Driver
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Phidgets ${CMAKE_MODULE_PATH})
find_package( PHIDGETS QUIET )
if( PHIDGETS_FOUND )
    set( Build_Phidgets true CACHE BOOL force )
    if( Build_Phidgets )
        include_directories( ${PHIDGETS_INCLUDE_DIR} )
        add_subdirectory( Phidgets )
        add_definitions( -DUSE_PHIDGETS )
    endif()
endif()

add_library( IMUDriverRegistery IMUDriverRegistery.cpp DriverList.h )

# Build the IMU driver registery library
add_to_rpg_libraries( IMUDriverRegistery )
