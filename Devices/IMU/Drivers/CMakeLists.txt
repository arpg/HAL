# Build MicroStrain IMU driver
add_subdirectory( MicroStrain )

add_library( IMUDriverRegistery IMUDriverRegistery.cpp DriverList.h )

# Build the IMU driver registery library
add_to_rpg_libraries( IMUDriverRegistery )
