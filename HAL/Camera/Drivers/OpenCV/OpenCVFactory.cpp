#include <HAL/Devices/DeviceFactory.h>
#include "OpenCVDriver.h"

namespace hal
{

class OpenCVFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    OpenCVFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        unsigned int nCamId     = uri.properties.Get<unsigned int>("id", 0);
        bool bGrey              = uri.properties.Get<bool>("grey", false);

        OpenCVDriver* pDriver = new OpenCVDriver( nCamId, bGrey );
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static OpenCVFactory g_OpenCVFactory("opencv");

}
