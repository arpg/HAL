list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package( TinyXML2 QUIET )

message( STATUS "TinyXML2_LIBRARIES ${TinyXML2_LIBRARIES}")

if( TinyXML2_FOUND )
  add_to_hal_sources(VelodyneDriver.h VelodyneDriver.cpp VelodyneFactory.cpp)
  add_to_hal_libraries(${TinyXML2_LIBRARIES})
  add_to_hal_include_dirs(${TinyXML2_INCLUDE_DIRS})
endif()

