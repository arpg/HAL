 #######################################################################################
 # get_git_version.cmake - Function for setting up versions based on GIT.
 #
 # get_git_version - Takes a variable name and sets it to the GIT tag version.
 #
 # If GIT package is not found, sets default version to 1.0.0
 #######################################################################################

function(set_git_version var)
  find_package(GIT QUIET)
  if(GIT_FOUND)
    execute_process(COMMAND ${GIT_EXECUTABLE} describe --abbrev=0
                    RESULT_VARIABLE GIT_TAG_RESULT
                    OUTPUT_VARIABLE GIT_TAG_VERSION
                    OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    if(GIT_TAG_RESULT EQUAL 0)
      set(${var} ${GIT_TAG_VERSION} CACHE INTERNAL "Version Number" FORCE)
    else()
      set(${var} "1.0.0" CACHE INTERNAL "Version Number" FORCE)
      message(STATUS "GIT not found! ${var} set to default value: ${${var}}")
    endif()
  else()
    set(${var} "1.0.0" CACHE INTERNAL "Version Number" FORCE)
    message(STATUS "GIT not found! ${var} set to default value: ${${var}}")
  endif()
endfunction()
