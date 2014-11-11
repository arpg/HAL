#include <HAL/Devices/DeviceFactory.h>
#include "NodeSimCamDriver.h"

namespace hal
{

class NodeSimCamFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    NodeSimCamFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
            {"id", "", "Device ID (Serve the purpose of UUID)."},
            {"sim", "", "name of the simulator, which is also node name."},
            {"name", "", "name of the camera. Id will be used if not set"},
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        NodeSimCamDriver* pDriver = new NodeSimCamDriver(uri);
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static NodeSimCamFactory g_NodeCamFactory("nodesim");

}
