#include <HAL/Devices/DeviceFactory.h>
#include "PhidgetsDriver.h"

namespace hal
{

class PhidgetsFactory : public DeviceFactory<IMUDriverInterface>
{
public:
    PhidgetsFactory(const std::string& name)
        : DeviceFactory<IMUDriverInterface>(name)
    {
        Params() = {
             {"imuhz", "250", "IMU capture rate in Hz; min 1, max 250."}
        };
    }

    std::shared_ptr<IMUDriverInterface> GetDevice(const Uri& uri)
    {
        int imu_hz = uri.properties.Get("imuhz", 250);
        PhidgetsDriver* pDriver = new PhidgetsDriver( imu_hz );
        return std::shared_ptr<IMUDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static PhidgetsFactory g_PhidgetsFactory("phidgets");

}
