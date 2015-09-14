#include <HAL/Devices/DriverFactory.h>
#include "UvcDriver.h"

namespace hal
{

class UvcFactory : public DriverFactory<CameraDriverInterface>
{
public:
    UvcFactory(const std::string& name)
        : DriverFactory<CameraDriverInterface>(name)
    {
        Params() = {
        };
    }
        
    std::shared_ptr<CameraDriverInterface> CreateDriver(const Uri& /*uri*/)
    {
        UvcDriver* Uvc = new UvcDriver();
        return std::shared_ptr<CameraDriverInterface>( Uvc );
    }
};

// Register this factory by creating static instance of factory
static UvcFactory g_UvcFactory("uvc");

}
