# 1) Build the library
set( ROBOT_HEADERS Robot.h )
set( ROBOT_SOURCES Robot.cpp )

add_library( Robot ${ROBOT_SOURCES} ${ROBOT_HEADERS} )

# 2) Tell RPG about this library:
add_to_rpg_include_dirs( ${CMAKE_CURRENT_BINARY_DIR} )
add_to_rpg_libraries( Robot )
add_to_rpg_link_directories( ${CMAKE_CURRENT_BINARY_DIR} )

# 3) And make sure we are able to install it correctly:
install( TARGETS Robot ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install( FILES ${ROBOT_HEADERS}
         DESTINATION
	 ${CMAKE_INSTALL_PREFIX}/include/RPG )
