#include <HAL/Devices/DriverFactory.h>
#include "PhidgetsDriver.h"

namespace hal
{

class PhidgetsFactory : public DriverFactory<IMUDriverInterface>
{
public:
    PhidgetsFactory(const std::string& name)
        : DriverFactory<IMUDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<IMUDriverInterface> CreateDriver(const Uri& /*uri*/)
    {
        PhidgetsDriver* pDriver = new PhidgetsDriver();
        return std::shared_ptr<IMUDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static PhidgetsFactory g_PhidgetsFactory("phidgets");

}
