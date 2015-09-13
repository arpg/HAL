#include <HAL/Devices/DeviceFactory.h>
#include "DeinterlaceDriver.h"

#include <string>

namespace hal
{

class DeinterlaceFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    DeinterlaceFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        const Uri input_uri = Uri(uri.url);

        // Create input camera
        std::shared_ptr<CameraDriverInterface> Input =
                DeviceRegistry<hal::CameraDriverInterface>::Instance().Create(input_uri);

        DeinterlaceDriver* pDriver = new DeinterlaceDriver( Input );
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static DeinterlaceFactory g_DeinterlaceFactory("deinterlace");

}
