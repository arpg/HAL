# 1) Build the library
set( BB2_DRIVER_SOURCES Bumblebee2Driver.cpp )
set( BB2_DRIVER_HEADERS Bumblebee2Driver.h )
add_library( bumblebee2driver ${BB2_DRIVER_SOURCES} ${B2_DRIVER_HEADERS} )

# 2) Tell RPG about this library:
add_to_rpg_include_dirs( ${DC1394_2_INCLUDE_DIR} ${CMAKE_CURRENT_BINARY_DIR} )
add_to_rpg_libraries( bumblebee2driver ${DC1394_2_LIBRARIES} )
add_to_rpg_link_directireis( ${CMAKE_CURRENT_BINARY_DIR} )

# 3) And make sure we are able to install it correctly:
install( TARGETS bumblebee2driver ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install( FILES ${BB2_DRIVER_HEADERS}
         DESTINATION 
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/Bumblebee2 )
