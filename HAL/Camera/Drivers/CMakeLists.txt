subdirlist(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})
list(REVERSE SUBDIRS)

foreach(subdir ${SUBDIRS})
  add_subdirectory(${subdir})
endforeach()

if(HAL_HAVE_THIRDPARTY)
  subdirlist(SUBDIRS "${HAL_THIRDPARTY_DIR}/Camera")
  foreach(subdir ${SUBDIRS})
    add_subdirectory("${HAL_THIRDPARTY_DIR}/Camera/${subdir}" "${CMAKE_CURRENT_BINARY_DIR}/${subdir}")
  endforeach()
endif()
