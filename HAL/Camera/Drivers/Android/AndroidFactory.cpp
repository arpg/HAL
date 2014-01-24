#include <HAL/Devices/DeviceFactory.h>
#include "AndroidDriver.h"

namespace hal
{

class AndroidFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    AndroidFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& /*uri*/)
    {
        AndroidDriver* driver = new AndroidDriver();
        return std::shared_ptr<CameraDriverInterface>( driver );
    }
};

// Register this factory by creating static instance of factory
static AndroidFactory g_AndroidFactory("android");

}
