#include <HAL/Devices/DriverFactory.h>
#include "DebayerDriver.h"

#include <string>

namespace hal
{

class DebayerFactory : public DriverFactory<CameraDriverInterface>
{
public:
    DebayerFactory(const std::string& name)
        : DriverFactory<CameraDriverInterface>(name){}

    std::shared_ptr<CameraDriverInterface> CreateDriver(const Uri& uri)
    {
        const Uri input_uri = Uri(uri.url);
        std::shared_ptr<CameraDriverInterface> input =
                DeviceRegistry<hal::CameraDriverInterface>::Instance().Create(input_uri);
        DebayerDriver* pDriver = new DebayerDriver( input, uri );
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static DebayerFactory g_DebayerFactory("debayer");

}
