# Build SimulatedCarDriver
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/SimulatedCarDriver ${CMAKE_MODULE_PATH} )
add_subdirectory( SimulatedCarDriver )

add_library( CarDriverRegistery CarDriverRegistery.cpp DriverList.h )

# Build the car driver registery library
add_to_hal_libraries( CarDriverRegistery )
