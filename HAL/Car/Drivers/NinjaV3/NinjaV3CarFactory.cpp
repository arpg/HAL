#include <HAL/Devices/DeviceFactory.h>
#include <HAL/Car/CarDriverInterface.h>

#include "NinjaV3CarDriver.h"

namespace hal
{

class NinjaV3CarFactory : public DeviceFactory<CarDriverInterface>
{
public:
    NinjaV3CarFactory(const std::string& name)
        : DeviceFactory<CarDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<CarDriverInterface> GetDevice(const Uri& uri)
    {
        NinjaV3CarDriver* pDriver = new NinjaV3CarDriver(uri);
        return std::shared_ptr<CarDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static NinjaV3CarFactory g_NinjaV3CarFactory("ninja_v3");

}
