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
            {"id","0","Camera id."},
            {"name", "AndroidCam", "Camera name"}
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        unsigned int nCamId     = uri.properties.Get<unsigned int>("id", 0);
        std::string sName       = uri.properties.Get<std::string>("name", "AndroidCam");
        AndroidDriver* driver = new AndroidDriver();
        return std::shared_ptr<CameraDriverInterface>( driver );
    }
};

// Register this factory by creating static instance of factory
static AndroidFactory g_AndroidFactory("android");

}
