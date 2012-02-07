# 1) Build the library
set( HERMES_SOURCES Hermes.cpp )
set( HERMES_HEADERS Hermes.h )
add_library( Hermes ${HERMES_SOURCES} ${HERMES_HEADERS} )

# 2) Tell RPG about this library:
add_to_rpg_include_dirs( ${ZeroMQ_INCLUDE_DIR}
			 ${PROTOBUF_INCLUDE_DIRS}
			 ${CMAKE_CURRENT_BINARY_DIR} )
add_to_rpg_libraries( Hermes )

list( APPEND RPG_LIBRARIES ${ZeroMQ_LIBRARY} ${PROTOBUF_LIBRARY} )
set( RPG_LIBRARIES ${RPG_LIBRARIES} CACHE INTERNAL "" FORCE )

add_to_rpg_link_directories( ${CMAKE_CURRENT_BINARY_DIR} )

# 3) And make sure we are able to install it correctly:
install( TARGETS Hermes ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install( FILES ${HERMES_HEADERS}
         DESTINATION 
	 ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Hermes )
