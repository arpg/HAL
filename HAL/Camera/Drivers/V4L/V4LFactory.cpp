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
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        const std::string devname = uri.url;
        V4LDriver* pDriver = new V4LDriver(devname, IO_METHOD_MMAP);
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static V4LFactory g_V4LFactory("v4l");
}
