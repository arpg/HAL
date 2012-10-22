# Build MicroStrain IMU driver
set( Build_MicroStrain true CACHE BOOL force )
if( Build_MicroStrain )
    add_subdirectory( MicroStrain )
    add_definitions( -DUSE_MICROSTRAIN )
endif()

add_library( IMUDriverRegistery IMUDriverRegistery.cpp DriverList.h )

# Build the IMU driver registery library
add_to_rpg_libraries( IMUDriverRegistery )
