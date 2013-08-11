#include <HAL/Devices/DeviceFactory.h>

#include "EpiphanDriver.h"

namespace hal
{

class EpiphanFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    EpiphanFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        EpiphanDriver* pDriver = new EpiphanDriver();
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static EpiphanFactory g_EpiphanFactory("epiphan");

}
