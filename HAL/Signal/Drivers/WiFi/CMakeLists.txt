set( BUILD_WiFi true CACHE BOOL force )

if( BUILD_WiFi )
  add_to_hal_sources( WiFiDriver.h WiFiDriver.cpp WiFiFactory.cpp MacOSX/macwifi.mm )
  set_property( GLOBAL APPEND PROPERTY P_LIBRARIES "-framework Foundation" )
  set_property( GLOBAL APPEND PROPERTY P_LIBRARIES "-framework CoreWLAN" )
  set_source_files_properties( MacOSX/macwifi.mm PROPERTIES COMPILE_FLAGS "-x objective-c++" )
endif()
