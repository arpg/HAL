# 1) Build the library
set( BB2_DRIVER_SOURCES Bumblebee2Driver.cpp )
set( BB2_DRIVER_HEADERS Bumblebee2Driver.h )
add_library( Bumblebee2Driver ${BB2_DRIVER_SOURCES} ${B2_DRIVER_HEADERS} )

# 2) Tell RPG about this library:
add_to_rpg_include_dirs( ${DC1394_2_INCLUDE_DIR} ${CMAKE_CURRENT_BINARY_DIR} )
message( STATUS "2 GOT ${DC1394_2_LIBRARIES}" )
add_to_rpg_libraries( Bumblebee2Driver ${DC1394_2_LIBRARIES} )
add_to_rpg_link_directories( ${CMAKE_CURRENT_BINARY_DIR} )

# 3) And make sure we are able to install it correctly:
install( TARGETS Bumblebee2Driver ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install( FILES ${BB2_DRIVER_HEADERS}
         DESTINATION 
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/Bumblebee2 )
