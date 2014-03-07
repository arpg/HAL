find_package(Git)
if(GIT_FOUND)
  include(ExternalProject)
  ExternalProject_Add(ThirdPartyHAL
    GIT_REPOSITORY rpg@rpg.robotics.gwu.edu:git/ThirdPartyHAL
    CONFIGURE_COMMAND cmake -E copy_directory . ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    BUILD_IN_SOURCE ON
    )
  set_target_properties(ThirdPartyHAL PROPERTIES EXCLUDE_FROM_ALL ON)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty)
  set(PCAN_HAVE_LEXUSISF12 1 CACHE INTERNAL "ThirdPartyConfig.h Definition" FORCE)
  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  add_subdirectory(ThirdParty)
endif()

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/ThirdPartyConfig.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/ThirdPartyConfig.h)
