if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    message( STATUS "HAL: building 'V4L' camera driver.")
    add_to_hal_sources(
        V4LDriver.h V4LDriver.cpp V4LFactory.cpp
    )
endif()
