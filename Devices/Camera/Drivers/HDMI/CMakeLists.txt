# 1) Build the library
include_directories( Blackmagic )
set( HDMI_DRIVER_SOURCES HDMIDriver.cpp Blackmagic/DeckLinkAPIDispatch.cpp )
set( HDMI_DRIVER_HEADERS HDMIDriver.h )
add_library( HDMIDriver ${HDMI_DRIVER_SOURCES} ${HDMI_DRIVER_HEADERS} )

# 2) Tell RPG about this library:
add_to_rpg_include_dirs( ${CMAKE_CURRENT_BINARY_DIR} )
add_to_rpg_libraries( HDMIDriver )
add_to_rpg_link_directories( ${CMAKE_CURRENT_BINARY_DIR} )

# 3) And make sure we are able to install it correctly:
install( TARGETS HDMIDriver ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install( FILES ${HDMI_DRIVER_HEADERS}
         DESTINATION
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/HDMI )
