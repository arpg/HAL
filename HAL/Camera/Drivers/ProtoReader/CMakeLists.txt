
message( STATUS "HAL: building 'ProtoReader' camera driver.")

add_to_hal_sources(
    ProtoReaderDriver.h ProtoReaderDriver.cpp ProtoReaderFactory.cpp
)
