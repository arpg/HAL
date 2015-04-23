subdirlist(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})

foreach(subdir ${SUBDIRS})
  if(NOT("${subdir}" STREQUAL "CMakeFiles"))
    add_subdirectory(${subdir})
  endif()
endforeach()
