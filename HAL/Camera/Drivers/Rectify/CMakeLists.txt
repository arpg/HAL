find_package( Calibu 0.1 QUIET )
if(Calibu_FOUND)
  set( BUILD_Rectify true CACHE BOOL force )

  if( BUILD_Rectify )
      add_to_hal_libraries( ${Calibu_LIBRARIES} )
      add_to_hal_include_dirs( ${Calibu_INCLUDE_DIRS} )
      add_to_hal_sources(
          RectifyDriver.h RectifyDriver.cpp RectifyFactory.cpp
      )
  endif()
endif()
