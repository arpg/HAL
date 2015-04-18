subdirlist(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})

foreach(subdir ${SUBDIRS})
  if(NOT("${subdir}" STREQUAL "CMakeFiles"))
    add_subdirectory(${subdir})
  endif()
endforeach()

if(HAL_HAVE_THIRDPARTY)
  subdirlist(SUBDIRS "${HAL_THIRDPARTY_DIR}/IMU")
  foreach(subdir ${SUBDIRS})
    add_subdirectory("${HAL_THIRDPARTY_DIR}/IMU/${subdir}" "${CMAKE_CURRENT_BINARY_DIR}/${subdir}")
  endforeach()
endif()
