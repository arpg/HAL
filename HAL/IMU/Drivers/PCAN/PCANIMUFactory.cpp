#include <HAL/Devices/DeviceFactory.h>

#include "PCANIMUDriver.h"

namespace hal
{

class PCANIMUFactory : public DeviceFactory<IMUDriverInterface>
{
public:
    PCANIMUFactory(const std::string& name)
        : DeviceFactory<IMUDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<IMUDriverInterface> GetDevice(const Uri& uri)
    {
        PCANIMUDriver* pDriver = new PCANIMUDriver( uri.url );
        return std::shared_ptr<IMUDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static PCANIMUFactory g_PCANIMUFactory("pcan");

}
