# 1) Build the library
set( TOYOTAREADER_DRIVER_SOURCES ToyotaReaderDriver.cpp )
set( TOYOTAREADER_DRIVER_HEADERS ToyotaReaderDriver.h )
add_library( ToyotaReaderDriver ${TOYOTAREADER_DRIVER_SOURCES} ${TOYOTAREADER_DRIVER_HEADERS} )

# 2) Tell RPG about this library:
add_to_rpg_include_dirs( ${CMAKE_CURRENT_SOURCE_DIR} )
add_to_rpg_libraries( ToyotaReaderDriver )
add_to_rpg_link_directories( ${CMAKE_CURRENT_BINARY_DIR} )

# 3) And make sure we are able to install it correctly:
install( TARGETS ToyotaReaderDriver ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install( FILES ${TOYOTAREADER_DRIVER_HEADERS}
         DESTINATION 
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/ToyotaReader )
