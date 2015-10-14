#include <HAL/Devices/DriverFactory.h>
#include "AutoExposureDriver.h"

#include <unistd.h>

namespace hal
{

class AutoExposureFactory : public DriverFactory<CameraDriverInterface>
{
public:
    AutoExposureFactory(const std::string& name)
        : DriverFactory<CameraDriverInterface>(name)
    {
        Params() = {
            {"target", "100", "The target meant brightness of the image."},
        };
    }

    std::shared_ptr<CameraDriverInterface> CreateDriver(const Uri& uri)
    {
        // Create input camera
        std::shared_ptr<CameraDriverInterface> InCam =
                DeviceDriverRegistry<hal::CameraDriverInterface>::Instance().Create(uri.url);

        int nTarget  = uri.properties.Get("target", 100);

        AutoExposureDriver* driver = new AutoExposureDriver(nTarget,InCam);
        return std::shared_ptr<CameraDriverInterface>( driver );
    }
};

// Register this factory by creating static instance of factory
static AutoExposureFactory g_SplitFactory("autoexposure");


}
