
set( BUILD_FileReader true CACHE BOOL force )

if( BUILD_FileReader )

    message( STATUS "HAL: building 'FileReader' camera driver.")
add_to_hal_sources(
    FileReaderDriver.cpp FileReaderFactory.cpp ReadImage.cpp
    FileReaderDriver.h ReadImage.h
)
endif()
