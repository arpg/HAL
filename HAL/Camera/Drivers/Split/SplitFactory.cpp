#include <HAL/Devices/DriverFactory.h>
#include "SplitDriver.h"

#include <unistd.h>

namespace hal
{

class SplitFactory : public DriverFactory<CameraDriverInterface>
{
public:
    SplitFactory(const std::string& name)
        : DriverFactory<CameraDriverInterface>(name) {}

    std::shared_ptr<CameraDriverInterface> CreateDriver(const Uri& uri)
    {
        // Standard boiler plate... should probably be consolidated elsewhere
        hal::Uri subUri(uri.url);
        subUri.SetProperties( uri.properties );
        std::shared_ptr<CameraDriverInterface> input =
                DeviceRegistry<hal::CameraDriverInterface>::Instance().Create(subUri);
        SplitDriver* pDriver = new SplitDriver( input, uri );
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static SplitFactory g_SplitFactory("split");

}
