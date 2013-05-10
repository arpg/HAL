# 1) Build the library:
set( FF_DRIVER_SOURCES FireFlyDriver.cpp )
set( FF_DRIVER_HEADERS FireFlyDriver.h )
add_library( FireFlyDriver ${FF_DRIVER_SOURCES} ${FF_DRIVER_HEADERS} )

# 2) Tell HAL about this library:
add_to_hal_include_dirs( ${DC1394_2_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} )
add_to_hal_libraries( FireFlyDriver )

list( APPEND HAL_LIBRARIES ${DC1394_2_LIBRARIES} )
set( HAL_LIBRARIES ${HAL_LIBRARIES} CACHE INTERNAL "" FORCE )

add_to_hal_link_directories( ${CMAKE_CURRENT_BINARY_DIR} )

# 3) And make sure we are able to install it correctly:
install( TARGETS FireFlyDriver
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

install( FILES ${FF_DRIVER_HEADERS}
         DESTINATION
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/FireFly )
