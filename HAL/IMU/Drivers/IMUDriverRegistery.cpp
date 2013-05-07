#include "HAL/IMU/IMUDriverInterface.h"
#include "HAL/IMU/Drivers/IMUDriverRegistery.h"

#include "HAL/IMU/Drivers/DriverList.h"

IMUDriver* CreateIMUDriver( const std::string& sDriverName )
{
//    printf("NOW g_mDriverTable lives at %p\n", &g_mDriverTable );
    std::map<std::string,IMUDriver*(*)()>::iterator it;
    it = g_mIMUDriverTable.find( sDriverName );
    if( it != g_mIMUDriverTable.end() ){
        return (it->second)();
    }
    std::cerr << "ERROR: unknown driver '" << sDriverName << "'\n";
    std::cerr << "INFO: Known driver list:\n";
    for( it = g_mIMUDriverTable.begin(); it != g_mIMUDriverTable.end(); it++ ){
        std::cerr << "\t" << it->first << std::endl;
    }
    return NULL;
}

