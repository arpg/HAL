

set( BUILD_Cleave true CACHE BOOL force )

if( BUILD_Cleave)

    message( STATUS "HAL: building 'Cleave' abstract camera driver.")

add_to_hal_sources(
	CleaveDriver.cpp
	CleaveFactory.cpp
	)	

endif()


