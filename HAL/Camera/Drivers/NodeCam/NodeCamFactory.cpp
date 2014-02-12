#include <HAL/Devices/DeviceFactory.h>
#include "NodeCamDriver.h"

namespace hal
{

class NodeCamFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    NodeCamFactory(const std::string& name)
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
        NodeCamDriver* pDriver = new NodeCamDriver(uri);
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static NodeCamFactory g_NodeCamFactory("NodeCam");

}
