if(0)

    message( STATUS "HAL: building 'ToyotaReader' camera driver.")

# 1) Build the library
set( TOYOTAREADER_DRIVER_SOURCES ToyotaReaderDriver.cpp )
set( TOYOTAREADER_DRIVER_HEADERS ToyotaReaderDriver.h )
add_library( ToyotaReaderDriver ${TOYOTAREADER_DRIVER_SOURCES} ${TOYOTAREADER_DRIVER_HEADERS} )

# 2) Tell HAL about this library:
add_to_hal_libraries( ToyotaReaderDriver )

# 3) And make sure we are able to install it correctly:
install( TARGETS ToyotaReaderDriver
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

install( FILES ${TOYOTAREADER_DRIVER_HEADERS}
         DESTINATION
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/ToyotaReader )
endif()
