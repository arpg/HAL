if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    add_to_hal_sources(
        V4LDriver.h V4LDriver.cpp V4LFactory.cpp
    )
endif()
