find_package(USB1 QUIET)

if(USB1_FOUND)
    set( BUILD_AutoExposure true CACHE BOOL force )

    if( BUILD_AutoExposure)

        message( STATUS "HAL: building 'AutoExposure' abstract camera driver.")

	    add_to_hal_sources(
       		 AutoExposureDriver.h AutoExposureDriver.cpp AutoExposureFactory.cpp
    		)
    endif()	
endif()
