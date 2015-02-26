#include <HAL/Devices/DeviceFactory.h>
#include "RealSense.h"

namespace hal
{

class RealSenseFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    RealSenseFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
        };
    }
        
    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& /*uri*/)
    {
        RealSenseDriver* rs = new RealSenseDriver();
        return std::shared_ptr<CameraDriverInterface>( rs );
    }
};

// Register this factory by creating static instance of factory
static RealSenseFactory g_RealSenseFactory("realsense");

}
