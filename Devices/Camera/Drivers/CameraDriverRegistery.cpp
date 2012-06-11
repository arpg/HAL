#include "RPG/Devices/Camera/CameraDriverInterface.h"
#include "RPG/Devices/Camera/Drivers/CameraDriverRegistery.h"

#include "RPG/Devices/Camera/Drivers/DriverList.h"

CameraDriver* CreateDriver( const std::string& sDriverName )
{
//    printf("NOW g_mDriverTable lives at %p\n", &g_mDriverTable );
    std::map<std::string,CameraDriver*(*)()>::iterator it;
    it = g_mCameraDriverTable.find( sDriverName );
    if( it != g_mCameraDriverTable.end() ){
        return (it->second)();
    }
    std::cerr << "ERROR: unknown driver '" << sDriverName << "'\n";
    std::cerr << "INFO: Known driver list:\n";
    for( it = g_mCameraDriverTable.begin(); it != g_mCameraDriverTable.end(); it++ ){
        std::cerr << "\t" << it->first << std::endl;
    }
    return NULL;
}

