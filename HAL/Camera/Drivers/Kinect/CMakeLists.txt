set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Kinect ${CMAKE_MODULE_PATH} )

find_package( OpenNI QUIET )
if( OPENNI_FOUND )
    add_definitions( -DUSE_KINECT )

    # 1) Build the library:
    set( KINECT_DRIVER_SOURCES KinectDriver.cpp )
    set( KINECT_DRIVER_HEADERS KinectDriver.h )
    
    if(CMAKE_COMPILER_IS_GNUCXX)
        # Ignore warnings generated from third party includes
        set_source_files_properties( KinectDriver.cpp PROPERTIES COMPILE_FLAGS " -Wno-unknown-pragmas -Wno-reorder" )
    endif(CMAKE_COMPILER_IS_GNUCXX)
    
    add_library( KinectDriver ${KINECT_DRIVER_SOURCES} ${KINECT_DRIVER_HEADERS} )
    
    # 2) Tell HAL about this library:
    add_to_hal_include_dirs( ${OPENNI_INCLUDE_DIRS} )
    add_to_hal_libraries( KinectDriver ${OPENNI_LIBRARY})
        
    # 3) And make sure we are able to install it correctly:
    install( TARGETS KinectDriver
      RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
      LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
      ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    )
    
    install( FILES ${KINECT_DRIVER_HEADERS}
             DESTINATION
             ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/Kinect )
endif()
