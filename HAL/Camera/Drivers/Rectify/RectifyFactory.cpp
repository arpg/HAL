#include <HAL/Devices/DriverFactory.h>
#include "RectifyDriver.h"

#include <calibu/cam/camera_xml.h>

namespace hal
{

class RectifyFactory : public DriverFactory<CameraDriverInterface>
{
public:
    RectifyFactory(const std::string& name)
        : DriverFactory<CameraDriverInterface>(name) {}
 
    std::shared_ptr<CameraDriverInterface> CreateDriver(const Uri& uri)
    {
        // Standard boiler plate... should probably be consolidated elsewhere
        const Uri input_uri = Uri(uri.url);
        std::shared_ptr<CameraDriverInterface> input =
                DeviceRegistry<hal::CameraDriverInterface>::Instance().Create(input_uri);
        RectifyDriver* pDriver = new RectifyDriver( input, uri );
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static RectifyFactory g_RectifyFactory("rectify");

}
