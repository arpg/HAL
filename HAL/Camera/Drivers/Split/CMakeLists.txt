
message( STATUS "HAL: building 'Split' abstract camera driver.")

add_to_hal_sources(
    SplitDriver.h SplitDriver.cpp SplitFactory.cpp
)
