# 1) Build the library:
set( FLY_DRIVER_SOURCES FlycapDriver.cpp )
set( FLY_DRIVER_HEADERS FlycapDriver.h )
add_library( FlycapDriver ${FLY_DRIVER_SOURCES} ${FLY_DRIVER_HEADERS} )

# 2) Tell RPG about this library:
add_to_rpg_include_dirs( ${FLYCAP_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} )
add_to_rpg_libraries( FlycapDriver )

list( APPEND RPG_LIBRARIES ${FLYCAP_LIBRARIES} )
set( RPG_LIBRARIES ${RPG_LIBRARIES} CACHE INTERNAL "" FORCE )

add_to_rpg_link_directories( ${CMAKE_CURRENT_BINARY_DIR} )

#MESSAGE( STATUS "here ${RPG_LIBRARIES}" )

# 3) And make sure we are able to install it correctly:
install( TARGETS FlycapDriver ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install( FILES ${FLY_DRIVER_HEADERS}
         DESTINATION
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/Flycap )
