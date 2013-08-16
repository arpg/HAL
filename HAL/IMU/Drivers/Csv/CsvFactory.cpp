#include <HAL/Devices/DeviceFactory.h>
#include <HAL/Utils/StringUtils.h>

#include "CsvDriver.h"

namespace hal
{

class CsvFactory : public DeviceFactory<IMUDriverInterface>
{
public:
    CsvFactory(const std::string& name)
        : DeviceFactory<IMUDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<IMUDriverInterface> GetDevice(const Uri& uri)
    {
        const std::string sDataSourceDir = hal::ExpandTildePath(uri.url);
        const std::string sFileAccel = uri.properties.Get( "Accel", sDataSourceDir+"/accel.txt");
        const std::string sFileGyro  = uri.properties.Get( "Gyro", sDataSourceDir+"/gyro.txt");
        const std::string sFileMag   = uri.properties.Get( "Mag", sDataSourceDir+"/mag.txt");
        const std::string sFileTimestamp  = uri.properties.Get( "Timestamp", sDataSourceDir+"/timestamp.txt");
        
        CsvDriver* pDriver = new CsvDriver(sFileAccel, sFileGyro, sFileMag, sFileTimestamp);
        return std::shared_ptr<IMUDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static CsvFactory g_CsvFactory("csv");

}
