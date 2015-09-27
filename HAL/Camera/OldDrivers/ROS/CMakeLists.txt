
find_package( roscpp QUIET )
find_package( image_transport QUIET )
find_package( std_msgs QUIET )
if(roscpp_FOUND)

    set( BUILD_ROS_Camera true CACHE BOOL force )

    if( BUILD_ROS_Camera)
        message( STATUS "HAL: building 'ROS' camera driver.")
        add_to_hal_include_dirs(
	    ${roscpp_INCLUDE_DIRS}
	    ${image_transport_INCLUDE_DIRS}
	    ${std_msgs_INCLUDE_DIRS}
        )

        add_to_hal_libraries(
	    ${image_transport_LIBRARIES}
	    ${std_msgs_LIBRARIES}
	    ${roscpp_LIBRARIES}
	    ${PbMsgs_LIBRARIES}
        )

        add_to_hal_sources(
		ROSDriver.cpp
		ROSFactory.cpp
        )

    endif()
endif()
