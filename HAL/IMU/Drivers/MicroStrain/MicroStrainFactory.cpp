#include <HAL/Devices/DeviceFactory.h>

#include "MicroStrainDriver.h"

namespace hal
{

class MicroStrainFactory : public DeviceFactory<IMUDriverInterface>
{
public:
    MicroStrainFactory(const std::string& name)
        : DeviceFactory<IMUDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<IMUDriverInterface> GetDevice(const Uri& /*uri*/)
    {
        MicroStrainDriver* pDriver = new MicroStrainDriver();
        return std::shared_ptr<IMUDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static MicroStrainFactory g_MicroStrainFactory("microstrain");

}
