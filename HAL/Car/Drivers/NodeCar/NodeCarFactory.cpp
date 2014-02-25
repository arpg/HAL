#include <HAL/Devices/DeviceFactory.h>
#include <HAL/Car/CarDriverInterface.h>

#include "NodeCarDriver.h"

namespace hal
{

class NodeCarFactory : public DeviceFactory<CarDriverInterface>
{
public:
    NodeCarFactory(const std::string& name)
        : DeviceFactory<CarDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<CarDriverInterface> GetDevice(const Uri& uri)
    {
        NodeCarDriver* pDriver = new NodeCarDriver(uri);
        return std::shared_ptr<CarDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static NodeCarFactory g_NodeCarFactory("NodeCar");

}
