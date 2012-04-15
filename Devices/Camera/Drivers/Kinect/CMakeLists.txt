# 1) Build the library:
set( KINECT_DRIVER_SOURCES KinectDriver.cpp )
set( KINECT_DRIVER_HEADERS KinectDriver.h )
add_library( KinectDriver ${KINECT_DRIVER_SOURCES} ${KINECT_DRIVER_HEADERS} )

# 2) Tell RPG about this library:
add_to_rpg_include_dirs( ${OPENNI_INCLUDE_DIRS} )
add_to_rpg_include_dirs( ${CMAKE_CURRENT_SOURCE_DIR} )
add_to_rpg_libraries( KinectDriver )

list( APPEND RPG_LIBRARIES ${OPENNI_LIBRARY} )
set( RPG_LIBRARIES ${RPG_LIBRARIES} CACHE INTERNAL "" FORCE )

add_to_rpg_link_directories( ${CMAKE_CURRENT_BINARY_DIR} )

#MESSAGE( STATUS "here ${RPG_LIBRARIES}" )

# 3) And make sure we are able to install it correctly:
install( TARGETS KinectDriver ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install( FILES ${KINECT_DRIVER_HEADERS}
         DESTINATION
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/Kinect )