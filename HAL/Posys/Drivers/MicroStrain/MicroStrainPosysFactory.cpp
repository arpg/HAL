#include <HAL/Devices/DriverFactory.h>

#include "MicroStrainPosysDriver.h"

namespace hal
{

class MicroStrainPosysFactory : public DriverFactory<PosysDriverInterface>
{
public:
    MicroStrainPosysFactory(const std::string& name)
        : DriverFactory<PosysDriverInterface>(name){}

    std::shared_ptr<PosysDriverInterface> CreateDriver(const Uri& /*uri*/)
    {
        MicroStrainPosysDriver* pDriver = new MicroStrainPosysDriver();
        return std::shared_ptr<PosysDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static MicroStrainPosysFactory g_MicroStrainPosysFactory2("microstrain");

}
