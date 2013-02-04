set( IMULOG_DRIVER_SOURCES IMULogDriver.cpp ../../../VirtualDevice.cpp )
set( IMULOG_DRIVER_HEADERS IMULogDriver.h )
add_library( IMULogDriver ${IMULOG_DRIVER_SOURCES} ${IMULOG_DRIVER_HEADERS} )
target_link_libraries( IMULogDriver ${IMULOG_LIBRARIES} )

list( APPEND RPG_LIBRARIES ${IMULOG_LIBRARIES} )
set( RPG_LIBRARIES ${RPG_LIBRARIES} CACHE INTERNAL "" FORCE )

# 2) Tell RPG about this library:
add_to_rpg_include_dirs( ${CMAKE_CURRENT_SOURCE_DIR} )
add_to_rpg_libraries( IMULogDriver )
add_to_rpg_link_directories( ${CMAKE_CURRENT_BINARY_DIR} )

# 3) And make sure we are able to install it correctly:
install( TARGETS IMULogDriver ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install( FILES ${IMULOG_DRIVER_HEADERS}
         DESTINATION
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/IMU/Drivers/IMULog )
