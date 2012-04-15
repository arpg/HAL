# 1) Build the library:
set( FF_DRIVER_SOURCES FireFlyDriver.cpp )
set( FF_DRIVER_HEADERS FireFlyDriver.h )
add_library( FireFlyDriver ${FF_DRIVER_SOURCES} ${FF_DRIVER_HEADERS} )

# 2) Tell RPG about this library:
add_to_rpg_include_dirs( ${DC1394_2_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} )
add_to_rpg_libraries( FireFlyDriver )

list( APPEND RPG_LIBRARIES ${DC1394_2_LIBRARIES} )
set( RPG_LIBRARIES ${RPG_LIBRARIES} CACHE INTERNAL "" FORCE )

add_to_rpg_link_directories( ${CMAKE_CURRENT_BINARY_DIR} )

#MESSAGE( STATUS "here ${RPG_LIBRARIES}" )

# 3) And make sure we are able to install it correctly:
install( TARGETS FireFlyDriver ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install( FILES ${FF_DRIVER_HEADERS}
         DESTINATION
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/FireFly )