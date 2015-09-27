#include <HAL/Devices/DriverFactory.h>
#include "AndroidDriver.h"

namespace hal
{

class AndroidFactory : public DriverFactory<CameraDriverInterface>
{
public:
    AndroidFactory(const std::string& name)
        : DriverFactory<CameraDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<CameraDriverInterface> CreateDriver(const Uri& /*uri*/)
    {
        AndroidDriver* driver = new AndroidDriver();
        return std::shared_ptr<CameraDriverInterface>( driver );
    }
};

// Register this factory by creating static instance of factory
static AndroidFactory g_AndroidFactory("android");

}
