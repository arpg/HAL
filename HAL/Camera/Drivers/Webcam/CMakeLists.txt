if(0)
# 1) Build the library
set( WEBCAM_DRIVER_SOURCES WebcamDriver.cpp )
set( WEBCAM_DRIVER_HEADERS WebcamDriver.h )
add_library( WebcamDriver ${WEBCAM_DRIVER_SOURCES} ${WEBCAM_DRIVER_HEADERS} )

# 2) Tell HAL about this library:
add_to_hal_libraries( WebcamDriver )

# 3) And make sure we are able to install it correctly:
install( TARGETS WebcamDriver
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

install( FILES ${WEBCAM_DRIVER_HEADERS}
         DESTINATION
     ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/Webcam )
endif()
