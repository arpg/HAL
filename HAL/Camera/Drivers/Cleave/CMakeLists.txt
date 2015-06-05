

set( BUILD_Cleave true CACHE BOOL force )

if( BUILD_Cleave)

add_to_hal_sources(
	CleaveDriver.cpp
	CleaveFactory.cpp
	)	

endif()


