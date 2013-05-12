# 1) Build the library:
set( SRCS SimulatedCarDriver.cpp )
set( HDRS SimulatedCarDriver.h ../CarDriverInterface.h ../../../../Utils/PropertyMap.h )
add_library( SimulatedCarDriver ${SRCS} ${HDRS} )

# 2) Tell RPG about this library:
add_to_hal_include_dirs( ${CMAKE_CURRENT_BINARY_DIR} )
add_to_hal_libraries( SimulatedCarDriver )

#list( APPEND RPG_LIBRARIES ${OGRE_LIBRARIES} )
#set( RPG_LIBRARIES ${RPG_LIBRARIES} CACHE INTERNAL "" FORCE )

#MESSAGE( STATUS "here ${RPG_LIBRARIES}" )

# 3) And make sure we are able to install it correctly:
install( TARGETS SimulatedCarDriver ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install( FILES ${HDRS}
         DESTINATION 
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Car/Drivers/SimulatedCarDriver )
