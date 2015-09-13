find_package( OccamIndigo QUIET )
option( BUILD_Occam "Build Occam Driver." OFF )

if( OccamIndigo_FOUND )
    if( BUILD_Occam )

        message( STATUS "HAL: building 'Occam' camera driver.")
        add_to_hal_include_dirs(
	    ${OccamIndigo_INCLUDE_DIRS}	
        )
        add_to_hal_libraries(
	    ${OccamIndigo_LIBRARIES}	
        )
        add_to_hal_sources(
		OccamDriver.cpp
		OccamFactory.cpp
        )
    endif()
else()
	set( BUILD_Occam false)
endif()

