 #######################################################################################
 # get_git_version.cmake - Function for setting up versions based on GIT.
 #
 # get_git_version - Takes a variable name and sets it to the GIT tag version.
 #######################################################################################

function(set_git_version var)
  execute_process(COMMAND git describe OUTPUT_VARIABLE GIT_TAG_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(${var} ${GIT_TAG_VERSION} CACHE INTERNAL "Version Number" FORCE)
endfunction()
