#include <HAL/Devices/DriverFactory.h>
#include "OpenNIDriver.h"

namespace hal
{

class OpenNIFactory : public DriverFactory<CameraDriverInterface>
{
public:
    OpenNIFactory(const std::string& name)
        : DriverFactory<CameraDriverInterface>(name){}

    std::shared_ptr<CameraDriverInterface> CreateDriver(const Uri& uri)
    {
        OpenNIDriver* pDriver = new OpenNIDriver( uri );
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static OpenNIFactory g_OpenNIFactory("openni");

}
