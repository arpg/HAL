# 1) Build the library:
set( AV_DRIVER_SOURCES AlliedVisionDriver.cpp )
set( AV_DRIVER_HEADERS AlliedVisionDriver.h )
add_library( AlliedVisionDriver ${AV_DRIVER_SOURCES} ${AV_DRIVER_HEADERS} )

include_directories( AlliedVisionDriver ${PVAPI_INCLUDE_DIR} )
target_link_libraries(AlliedVisionDriver ${PVAPI_LIBRARIES} )

# 2) Tell RPG about this library:
add_to_rpg_libraries( AlliedVisionDriver )

list( APPEND RPG_LIBRARIES "${PVAPI_LIBRARIES}" )
set( RPG_LIBRARIES ${RPG_LIBRARIES} CACHE INTERNAL "" FORCE )

add_to_rpg_link_directories( ${CMAKE_CURRENT_BINARY_DIR} )

# 3) And make sure we are able to install it correctly:
install( TARGETS AlliedVisionDriver ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
