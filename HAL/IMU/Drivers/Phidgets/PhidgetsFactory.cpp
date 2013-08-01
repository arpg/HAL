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
        };
    }

    std::shared_ptr<IMUDriverInterface> GetDevice(const Uri& /*uri*/)
    {
        PhidgetsDriver* pDriver = new PhidgetsDriver();
        return std::shared_ptr<IMUDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static PhidgetsFactory g_PhidgetsFactory("phidgets");

}
