# 1) Build the library:
set( BB2_DRIVER_SOURCES Bumblebee2Driver.cpp )
set( BB2_DRIVER_HEADERS Bumblebee2Driver.h )
add_library( Bumblebee2Driver ${BB2_DRIVER_SOURCES} ${BB2_DRIVER_HEADERS} )

# 2) Tell RPG about this library:
add_to_rpg_include_dirs( ${DC1394_2_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} )
add_to_rpg_libraries( Bumblebee2Driver )

list( APPEND RPG_LIBRARIES ${DC1394_2_LIBRARIES} )
set( RPG_LIBRARIES ${RPG_LIBRARIES} CACHE INTERNAL "" FORCE )

add_to_rpg_link_directories( ${CMAKE_CURRENT_BINARY_DIR} )

#MESSAGE( STATUS "here ${RPG_LIBRARIES}" )

# 3) And make sure we are able to install it correctly:
install( TARGETS Bumblebee2Driver
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

install( FILES ${BB2_DRIVER_HEADERS}
         DESTINATION
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/Bumblebee2 )
