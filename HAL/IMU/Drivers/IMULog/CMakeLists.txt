set( IMULOG_DRIVER_SOURCES IMULogDriver.cpp ../../../VirtualDevice.cpp )
set( IMULOG_DRIVER_HEADERS IMULogDriver.h )
add_library( IMULogDriver ${IMULOG_DRIVER_SOURCES} ${IMULOG_DRIVER_HEADERS} )
target_link_libraries( IMULogDriver ${IMULOG_LIBRARIES} )

list( APPEND HAL_LIBRARIES ${IMULOG_LIBRARIES} )
set( HAL_LIBRARIES ${HAL_LIBRARIES} CACHE INTERNAL "" FORCE )

# 2) Tell HAL about this library:
add_to_hal_include_dirs( ${CMAKE_CURRENT_SOURCE_DIR} )
add_to_hal_libraries( IMULogDriver )

# 3) And make sure we are able to install it correctly:
install( TARGETS IMULogDriver
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

install( FILES ${IMULOG_DRIVER_HEADERS}
         DESTINATION
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/IMU/Drivers/IMULog )
