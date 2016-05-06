
    set( BUILD_Gladiator_Imu false CACHE BOOL force )

    if( BUILD_Gladiator_Imu)
        message( STATUS "HAL: building Gladiator IMU driver.")
        add_to_hal_include_dirs(
	.
        )

        add_to_hal_libraries(
	    ${PbMsgs_LIBRARIES}
        )

        add_to_hal_sources(
		commport.cpp
		GladiatorDriver.cpp
		GladiatorFactory.cpp
		gladiator.cpp
		ThreadedCommand.cc
		ThreadedObject.cc
		
        )

    endif()

