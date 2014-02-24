#include <HAL/Devices/DeviceFactory.h>

#include "MicroStrainPosysDriver.h"

namespace hal
{

class MicroStrainPosysFactory : public DeviceFactory<PosysDriverInterface>
{
public:
    MicroStrainPosysFactory(const std::string& name)
        : DeviceFactory<PosysDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<PosysDriverInterface> GetDevice(const Uri& /*uri*/)
    {
        MicroStrainPosysDriver* pDriver = new MicroStrainPosysDriver();
        return std::shared_ptr<PosysDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static MicroStrainPosysFactory g_MicroStrainPosysFactory2("microstrain");

}
