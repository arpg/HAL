#include <HAL/Devices/DriverFactory.h>
#include "DeinterlaceDriver.h"

#include <string>

namespace hal
{

class DeinterlaceFactory : public DriverFactory<CameraDriverInterface>
{
public:
    DeinterlaceFactory(const std::string& name)
        : DriverFactory<CameraDriverInterface>(name) {}

    std::shared_ptr<CameraDriverInterface> CreateDriver(const Uri& uri)
    {
        const Uri input_uri = Uri(uri.url);
        std::shared_ptr<CameraDriverInterface> input_cam =
                DeviceDriverRegistry<hal::CameraDriverInterface>::Instance().Create(input_uri);
        DeinterlaceDriver* pDriver = new DeinterlaceDriver( input_cam, uri );
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static DeinterlaceFactory g_DeinterlaceFactory("deinterlace");

}
