#include <HAL/Devices/DeviceFactory.h>

#include "VelodyneDriver.h"

namespace hal
{

class VelodyneFactory : public DeviceFactory<LIDARDriverInterface>
{
public:
    VelodyneFactory(const std::string& name)
        : DeviceFactory<LIDARDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<LIDARDriverInterface> GetDevice(const Uri& /*uri*/)
    {
        VelodyneDriver* pDriver = new VelodyneDriver();
        return std::shared_ptr<LIDARDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static VelodyneFactory g_VelodyneFactory("velodyne");

}
