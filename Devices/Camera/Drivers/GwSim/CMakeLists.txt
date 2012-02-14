# 1) Build the library:
set( DRIVER_SOURCES GwSimDriver.cpp )
set( DRIVER_HEADERS GwSimDriver.h )
add_library( GwSimDriver ${DRIVER_SOURCES} ${DRIVER_HEADERS} )

# 2) Tell RPG about this library:
add_to_rpg_include_dirs( ${OGRE_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} )
add_to_rpg_libraries( GwSimDriver )

list( APPEND RPG_LIBRARIES ${OGRE_LIBRARIES} )
set( RPG_LIBRARIES ${RPG_LIBRARIES} CACHE INTERNAL "" FORCE )

add_to_rpg_link_directories( ${CMAKE_CURRENT_BINARY_DIR} )
 
#MESSAGE( STATUS "here ${RPG_LIBRARIES}" )

# 3) And make sure we are able to install it correctly:
install( TARGETS GwSimDriver ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install( FILES ${DRIVER_HEADERS}
         DESTINATION 
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/GwSim )
