#include <HAL/Devices/DeviceFactory.h>
#include "V4LDriver.h"

namespace hal
{

class V4LFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    V4LFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
            {"id","0","Camera id."},
            {"name", "V4LCam", "Camera name."},
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        const std::string devname = uri.url;
        unsigned int nCamId     = uri.properties.Get<unsigned int>("id", 0);
        std::string sName       = uri.properties.Get<std::string>("name", "V4LCam");
        V4LDriver* pDriver = new V4LDriver(devname, IO_METHOD_MMAP);
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static V4LFactory g_V4LFactory("v4l");
}
