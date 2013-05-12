# 1) Build the library
set( DEVICES_SOURCES DeviceTime.cpp)
set( DEVICES_HEADERS DeviceTime.h DeviceFactory.h DeviceRegistry.h )

add_library( Devices ${DEVICES_SOURCES} ${DEVICES_HEADERS} )

# 2) Tell HAL about this library:
add_to_hal_libraries( Devices )

# 3) And make sure we are able to install it correctly:
install( TARGETS Devices
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

install( FILES ${FILEREADER_DRIVER_HEADERS}
         DESTINATION
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/FileReader )
