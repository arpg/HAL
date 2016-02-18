
find_package( roscpp QUIET )
find_package( std_msgs QUIET )
if(roscpp_FOUND)

    set( BUILD_ROS_Imu false CACHE BOOL force )

    if( BUILD_ROS_Imu)
        message( STATUS "HAL: building 'ROS' IMU driver.")
        add_to_hal_include_dirs(
	    ${roscpp_INCLUDE_DIRS}
	    ${std_msgs_INCLUDE_DIRS}
        )

        add_to_hal_libraries(
	    ${std_msgs_LIBRARIES}
	    ${roscpp_LIBRARIES}
	    ${PbMsgs_LIBRARIES}
        )

        add_to_hal_sources(
		ROSImuDriver.cpp
		ROSImuFactory.cpp
        )

    endif()
endif()
