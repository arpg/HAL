macro(subdirlist result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
        set(dirlist ${dirlist} ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()

#subdirlist(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})
<<<<<<< .merge_file_Xp5c82
set(SUBDIRS FileReader OpenNI Rectify Split Debayer DC1394)
=======
set(SUBDIRS FileReader OpenNI Rectify Split Debayer V4L)
>>>>>>> .merge_file_xdOzej

foreach(subdir ${SUBDIRS})
    add_subdirectory(${subdir})
endforeach()
