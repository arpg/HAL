#include <HAL/Devices/DeviceFactory.h>

#include "ViconDriver.h"

namespace hal
{

class ViconFactory : public DeviceFactory<PosysDriverInterface>
{
public:
    ViconFactory(const std::string& name)
        : DeviceFactory<PosysDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<PosysDriverInterface> GetDevice(const Uri& /*uri*/)
    {
        ViconDriver* pDriver = new ViconDriver();
        return std::shared_ptr<PosysDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static ViconFactory g_ViconFactory("vicon");

}
