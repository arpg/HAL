# 1) Build the library
set( FILEREADER_DRIVER_SOURCES FileReaderDriver.cpp ../../../VirtualDevice.cpp )
set( FILEREADER_DRIVER_HEADERS FileReaderDriver.h )
add_library( FileReaderDriver ${FILEREADER_DRIVER_SOURCES} ${FILEREADER_DRIVER_HEADERS} )

# 2) Tell HAL about this library:
add_to_hal_include_dirs( ${CMAKE_CURRENT_SOURCE_DIR} )
add_to_hal_libraries( FileReaderDriver )
add_to_hal_link_directories( ${CMAKE_CURRENT_BINARY_DIR} )

# 3) And make sure we are able to install it correctly:
install( TARGETS FileReaderDriver
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

install( FILES ${FILEREADER_DRIVER_HEADERS}
         DESTINATION
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/FileReader )
