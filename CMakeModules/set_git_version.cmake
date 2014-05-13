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
    execute_process(COMMAND ${GIT_EXECUTABLE} describe OUTPUT_VARIABLE GIT_TAG_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)
    set(${var} ${GIT_TAG_VERSION} CACHE INTERNAL "Version Number" FORCE)
  else()
    set(${var} "1.0.0" CACHE INTERNAL "Version Number" FORCE)
  endif()
endfunction()
