#include <HAL/Devices/DeviceFactory.h>
#include "RectifyDriver.h"

namespace hal
{

class RectifyFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    RectifyFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
        };
    }
    
    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        // Create input camera
        std::shared_ptr<CameraDriverInterface> input =
                DeviceRegistry<hal::CameraDriverInterface>::I().Create(uri.url);
        
        
        RectifyDriver* rectify = new RectifyDriver(input);
        return std::shared_ptr<CameraDriverInterface>( rectify );
    }
};

// Register this factory by creating static instance of factory
static RectifyFactory g_RectifyFactory("rectify");

}
