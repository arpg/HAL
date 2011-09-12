# 1) Build the library
set( FILEREADER_DRIVER_SOURCES FileReaderDriver.cpp )
set( FILEREADER_DRIVER_HEADERS FileReaderDriver.h )
add_library( FileReaderDriver ${FILEREADER_DRIVER_SOURCES} ${FILEREADER_DRIVER_HEADERS} )

# 2) Tell RPG about this library:
add_to_rpg_include_dirs( ${CMAKE_CURRENT_BINARY_DIR} )
add_to_rpg_libraries( FileReaderDriver )
add_to_rpg_link_directireis( ${CMAKE_CURRENT_BINARY_DIR} )

# 3) And make sure we are able to install it correctly:
install( TARGETS FileReaderDriver ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install( FILES ${FILEREADER_DRIVER_HEADERS}
         DESTINATION 
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/FileReader )
