#include <HAL/Devices/DeviceFactory.h>

#include "NinjaIMUDriver.h"

namespace hal
{

class NinjaIMUFactory : public DeviceFactory<IMUDriverInterface>
{
public:
    NinjaIMUFactory(const std::string& name)
        : DeviceFactory<IMUDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<IMUDriverInterface> GetDevice(const Uri& uri)
    {
        NinjaIMUDriver* pDriver = new NinjaIMUDriver( uri.url );
        return std::shared_ptr<IMUDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static NinjaIMUFactory g_NinjaIMUFactory("ninja");

}
