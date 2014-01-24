#include <HAL/Devices/DeviceFactory.h>

// hack to enable sleep_for (GCC < 4.8)
#ifndef _GLIBCXX_USE_NANOSLEEP
#define _GLIBCXX_USE_NANOSLEEP
#endif  // _GLIBCXX_USE_NANOSLEEP

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
            {"id","0","Camera id."},
            {"name", "NodeCam", "Camera name."},

        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        unsigned int nCamId     = uri.properties.Get<unsigned int>("id", 0);
        std::string sName       = uri.properties.Get<std::string>("name", "NodeCam");
        NodeCamDriver* pDriver = new NodeCamDriver( uri.url );
        return std::shared_ptr<CameraDriverInterface>( pDriver );        
    }
};

// Register this factory by creating static instance of factory
static NodeCamFactory g_NodeCamFactory("node");

}
