include(FindPackageHandleStandardArgs)

find_path(RealSense2_INCLUDE_DIR "librealsense2/rs.h"
  /usr/include
  /usr/local/include
  /opt/local/include
)

find_library(RealSense2_LIBRARIES NAMES "realsense2"
  /usr/lib64
  /usr/lib
  /usr/local/lib
  /opt/local/lib
)

find_package_handle_standard_args("RealSense2" DEFAULT_MSG
  RealSense2_INCLUDE_DIR RealSense2_LIBRARIES)

set(RealSense2_FOUND ${RealSense2_FOUND} CACHE BOOL "RealSense2 was found or not" FORCE)
mark_as_advanced(RealSense2_INCLUDE_DIR RealSense2_LIBRARIES)
