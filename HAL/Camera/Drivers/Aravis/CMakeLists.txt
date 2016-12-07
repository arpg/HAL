find_package( Aravis QUIET )
find_package( OpenCV QUIET )
option( BUILD_Aravis "Build Aravis Driver." true)
#message( "HAL: building ${Aravis_LIBRARIES}")
if( Aravis_FOUND )
    if( BUILD_Aravis )

        message( STATUS "HAL: building 'Aravis' camera driver.")
        add_to_hal_include_dirs(
	    ${Aravis_INCLUDE_DIRS}
	    ${OpenCV_INCLUDE_DIRS}
        )
        add_to_hal_libraries(
	    ${Aravis_LIBRARIES}
	    ${OpenCV_LIBRARIES}
        )
        add_to_hal_sources(
		AravisDriver.cpp
		AravisFactory.cpp
        )
    endif()
else()
	set( BUILD_Aravis false)
endif()

