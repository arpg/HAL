#include <HAL/Devices/DeviceFactory.h>
#include "WebcamDriver.h"

namespace hal
{

class WebcamFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    WebcamFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        unsigned int nCamId     = uri.properties.Get<unsigned int>("id", 0);
        bool bGrey              = uri.properties.Get<bool>("grey", false);

        WebcamDriver* pDriver = new WebcamDriver( nCamId, bGrey );
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static WebcamFactory g_WebcamFactory("webcam");

}
