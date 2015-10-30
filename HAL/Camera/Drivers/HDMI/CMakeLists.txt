if(0)

 
    message( STATUS "HAL: building 'HDMI' camera driver.")

# 1) Build the library
include_directories( Blackmagic )
set( HDMI_DRIVER_SOURCES HDMIDriver.cpp Blackmagic/DeckLinkAPIDispatch.cpp CaptureDelegate.cpp)
set( HDMI_DRIVER_HEADERS HDMIDriver.h CaptureDelegate.h)
add_library( HDMIDriver ${HDMI_DRIVER_SOURCES} ${HDMI_DRIVER_HEADERS} )

# 2) Tell RPG about this library:
add_to_hal_libraries( HDMIDriver )

# 3) And make sure we are able to install it correctly:
install( TARGETS HDMIDriver
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

install( FILES ${HDMI_DRIVER_HEADERS}
         DESTINATION
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/HDMI )
endif()
