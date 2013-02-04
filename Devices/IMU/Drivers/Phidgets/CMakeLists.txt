include_directories(${PHIDGETS_INCLUDE_DIR})
set( PHIDGETS_DRIVER_SOURCES PhidgetsDriver.cpp )
set( PHIDGETS_DRIVER_HEADERS PhidgetsDriver.h )
add_library( PhidgetsDriver ${PHIDGETS_DRIVER_SOURCES} ${PHIDGETS_DRIVER_HEADERS} )
target_link_libraries( PhidgetsDriver ${PHIDGETS_LIBRARIES} )

list( APPEND RPG_LIBRARIES ${PHIDGETS_LIBRARIES} )
set( RPG_LIBRARIES ${RPG_LIBRARIES} CACHE INTERNAL "" FORCE )

# 2) Tell RPG about this library:
add_to_rpg_include_dirs( ${PHIDGETS_INCLUDE_DIR} )
add_to_rpg_include_dirs( ${CMAKE_CURRENT_SOURCE_DIR} )
add_to_rpg_libraries( PhidgetsDriver )
add_to_rpg_link_directories( ${CMAKE_CURRENT_BINARY_DIR} )

# 3) And make sure we are able to install it correctly:
install( TARGETS PhidgetsDriver ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install( FILES ${PHIDGETS_DRIVER_HEADERS}
         DESTINATION
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/IMU/Drivers/Phidgets )
