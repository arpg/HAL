if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty)
  set(PCAN_HAVE_LEXUSISF12 1 CACHE INTERNAL "ThirdPartyConfig.h Definition" FORCE)
  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  add_subdirectory(ThirdParty)
endif()

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/ThirdPartyConfig.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/ThirdPartyConfig.h)
