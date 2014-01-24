#include <HAL/Devices/DeviceFactory.h>
#include "Node2CamDriver.h"

namespace hal
{

class Node2CamFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    Node2CamFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
            {"id","0","Camera id."},
            {"name", "Node2Cam", "Camera name."},
            {"device", "", "DeviceName."},
            {"host", "", "name of host that Node2Cam connect."},

        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        unsigned int nCamId     = uri.properties.Get<unsigned int>("id", 0);
        std::string sName       = uri.properties.Get<std::string>("name", "Node2Cam");
        std::string  Device = uri.properties.Get<std::string>("device","NodeCam");
        std::string  Host = uri.properties.Get<std::string>("host","StateKeeper");

        Node2CamDriver* pDriver = new Node2CamDriver(Device, Host);
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static Node2CamFactory g_Node2CamFactory("node2cam");

}
