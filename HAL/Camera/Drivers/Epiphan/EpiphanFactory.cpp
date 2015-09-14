#include <HAL/Devices/DriverFactory.h>

#include "EpiphanDriver.h"

namespace hal
{

class EpiphanFactory : public DriverFactory<CameraDriverInterface>
{
public:
    EpiphanFactory(const std::string& name)
        : DriverFactory<CameraDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<CameraDriverInterface> CreateDriver( const Uri& uri)
    {
        EpiphanDriver* pDriver = new EpiphanDriver();
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static EpiphanFactory g_EpiphanFactory("epiphan");

}
