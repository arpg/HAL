# build the library
set( BUILD_Csv true CACHE BOOL force )

if( BUILD_Csv )
    add_to_hal_sources( CsvDriver.h CsvDriver.cpp CsvFactory.cpp )
endif()
