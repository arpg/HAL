#include <HAL/Devices/DriverFactory.h>

#include "VelodyneDriver.h"

namespace hal
{

class VelodyneFactory : public DriverFactory<LIDARDriverInterface>
{
public:
    VelodyneFactory(const std::string& name)
        : DriverFactory<LIDARDriverInterface>(name)
    {}

    std::shared_ptr<LIDARDriverInterface> CreateDriver(const Uri& /*uri*/)
    {
        VelodyneDriver* pDriver = new VelodyneDriver();
        return std::shared_ptr<LIDARDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static VelodyneFactory g_VelodyneFactory("velodyne");

}
