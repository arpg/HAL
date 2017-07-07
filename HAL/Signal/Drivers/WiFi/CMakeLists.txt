set( BUILD_WiFi true CACHE BOOL force )

if( BUILD_WiFi )
  add_to_hal_sources( WiFiDriver.h WiFiDriver.cpp WiFiFactory.cpp )
endif()
