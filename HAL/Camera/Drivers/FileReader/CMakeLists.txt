
set( Build_FileReader true CACHE BOOL force )

if( Build_FileReader )
add_to_hal_sources(
    FileReaderDriver.cpp FileReaderFactory.cpp ReadImage.cpp DirUtils.cpp
    FileReaderDriver.h ReadImage.h DirUtils.h
)
endif()
