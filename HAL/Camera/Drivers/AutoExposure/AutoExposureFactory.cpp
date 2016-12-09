#include <HAL/Devices/DeviceFactory.h>
#include "AutoExposureDriver.h"

#include <unistd.h>

namespace hal
{

class AutoExposureFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    AutoExposureFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
            {"target", "100", "The target meant brightness of the image."},
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        // Create input camera
        std::shared_ptr<CameraDriverInterface> InCam =
                DeviceRegistry<hal::CameraDriverInterface>::Instance().Create(uri.url);

        int nTarget  = uri.properties.Get("target", 100);

        AutoExposureDriver* driver = new AutoExposureDriver(nTarget,InCam);
        return std::shared_ptr<CameraDriverInterface>( driver );
    }
};

// Register this factory by creating static instance of factory
static AutoExposureFactory g_SplitFactory("autoexposure");


}
