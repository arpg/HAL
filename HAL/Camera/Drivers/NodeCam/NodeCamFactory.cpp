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
            {"device", "", "DeviceName."},
            {"host", "", "name of host that NodeCam connect."},
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        std::string  Device = uri.properties.Get<std::string>("device","NodeCam");
        std::string  Host = uri.properties.Get<std::string>("host","StateKeeper");

        NodeCamDriver* pDriver = new NodeCamDriver(Device, Host);
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static NodeCamFactory g_NodeCamFactory("NodeCam");

}
