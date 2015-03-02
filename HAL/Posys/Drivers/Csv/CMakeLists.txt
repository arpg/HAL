# build the library
set( BUILD_Csv_Posys true CACHE BOOL force )

if( BUILD_Csv_Posys )
    add_to_hal_sources( CsvPosysDriver.h CsvPosysDriver.cpp CsvPosysFactory.cpp )
endif()
