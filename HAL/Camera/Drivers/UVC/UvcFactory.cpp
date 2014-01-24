#include <HAL/Devices/DeviceFactory.h>
#include "UvcDriver.h"

namespace hal
{

class UvcFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    UvcFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
        };
    }
        
    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& /*uri*/)
    {
        UvcDriver* Uvc = new UvcDriver();
        return std::shared_ptr<CameraDriverInterface>( Uvc );
    }
};

// Register this factory by creating static instance of factory
static UvcFactory g_UvcFactory("uvc");

}
