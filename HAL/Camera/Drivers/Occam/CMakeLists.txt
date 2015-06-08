set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )


find_package( OccamIndigo QUIET )


if(OccamIndigo_FOUND)

    set( BUILD_Occam true CACHE BOOL force )

    if( BUILD_Occam)
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
endif()

message("Occam includes: ${OccamIndigo_INCLUDE_DIRS}")
message("Occam libs: ${OccamIndigo_LIBRARIES}")

