#include <HAL/Devices/DriverFactory.h>

#include "MicroStrainDriver.h"

namespace hal
{

class MicroStrainFactory : public DriverFactory<IMUDriverInterface>
{
public:
    MicroStrainFactory(const std::string& name)
        : DriverFactory<IMUDriverInterface>(name) {}

    std::shared_ptr<IMUDriverInterface> CreateDriver( const Uri& uri )
    {
        MicroStrainDriver* pDriver = new MicroStrainDriver( uri );
        return std::shared_ptr<IMUDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static MicroStrainFactory g_MicroStrainFactory("microstrain");

}
