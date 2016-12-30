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

    PosysDriverInterface* GetDevice(const Uri& /*uri*/)
    {
        MicroStrainPosysDriver* pDriver = new MicroStrainPosysDriver();
        return static_cast<PosysDriverInterface*>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static MicroStrainPosysFactory g_MicroStrainPosysFactory2("microstrain");

}
