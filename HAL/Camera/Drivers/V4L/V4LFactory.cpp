#include <HAL/Devices/DriverFactory.h>
#include "V4LDriver.h"

namespace hal
{

class V4LFactory : public DriverFactory<CameraDriverInterface>
{
public:
    V4LFactory(const std::string& name)
        : DriverFactory<CameraDriverInterface>(name){}

    std::shared_ptr<CameraDriverInterface> CreateDriver(const Uri& uri)
    {
        V4LDriver* pDriver = new V4LDriver( uri );
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static V4LFactory g_V4LFactory("v4l");
}
