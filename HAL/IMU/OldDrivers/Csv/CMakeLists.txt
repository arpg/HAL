# build the library
set( BUILD_Csv_IMU true CACHE BOOL force )

if( BUILD_Csv_IMU )
    add_to_hal_sources( CsvDriver.h CsvDriver.cpp CsvFactory.cpp )
endif()
