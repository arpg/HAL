
SET(OccamIndigo_FOUND true)
SET(OccamIndigo_INCLUDE_DIRS /home/smcguire/research/indigosdk-2.0.13/include)
SET(OccamIndigo_LIBRARIES /home/smcguire/research/indigosdk-2.0.13/bin/libindigo.so)


if(OccamIndigo_FOUND)

    set( BUILD_Occam true CACHE BOOL force )

    if( BUILD_Occam)
        add_to_hal_include_dirs(
	    ${OccamIndigo_INCLUDE_DIRS}	
        )
        add_to_hal_libraries(
	    /home/smcguire/research/indigosdk-2.0.13/bin/libindigo.so
        )
        add_to_hal_sources(
		OccamDriver.cpp
		OccamFactory.cpp
                   )

    endif()
endif()
