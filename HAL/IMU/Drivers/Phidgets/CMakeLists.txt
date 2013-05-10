include_directories(${PHIDGETS_INCLUDE_DIR})
set( PHIDGETS_DRIVER_SOURCES PhidgetsDriver.cpp )
set( PHIDGETS_DRIVER_HEADERS PhidgetsDriver.h )
add_library( PhidgetsDriver ${PHIDGETS_DRIVER_SOURCES} ${PHIDGETS_DRIVER_HEADERS} )
target_link_libraries( PhidgetsDriver ${PHIDGETS_LIBRARIES} )

list( APPEND HAL_LIBRARIES ${PHIDGETS_LIBRARIES} )
set( HAL_LIBRARIES ${HAL_LIBRARIES} CACHE INTERNAL "" FORCE )

# 2) Tell HAL about this library:
add_to_hal_include_dirs( ${PHIDGETS_INCLUDE_DIR} )
add_to_hal_include_dirs( ${CMAKE_CURRENT_SOURCE_DIR} )
add_to_hal_libraries( PhidgetsDriver )
add_to_hal_link_directories( ${CMAKE_CURRENT_BINARY_DIR} )

# 3) And make sure we are able to install it correctly:
install( TARGETS PhidgetsDriver
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

install( FILES ${PHIDGETS_DRIVER_HEADERS}
         DESTINATION
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/IMU/Drivers/Phidgets )
