# 1) Build the library
set( WEBCAM_DRIVER_SOURCES WebcamDriver.cpp )
set( WEBCAM_DRIVER_HEADERS WebcamDriver.h )
add_library( WebcamDriver ${WEBCAM_DRIVER_SOURCES} ${WEBCAM_DRIVER_HEADERS} )

# 2) Tell RPG about this library:
add_to_rpg_include_dirs( ${CMAKE_CURRENT_SOURCE_DIR} )
add_to_rpg_libraries( WebcamDriver )
add_to_rpg_link_directories( ${CMAKE_CURRENT_BINARY_DIR} )

# 3) And make sure we are able to install it correctly:
install( TARGETS WebcamDriver
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

install( FILES ${WEBCAM_DRIVER_HEADERS}
         DESTINATION 
	 ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/Webcam )
