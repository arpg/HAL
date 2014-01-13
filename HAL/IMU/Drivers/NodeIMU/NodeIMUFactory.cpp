#include <HAL/Devices/DeviceFactory.h>

// hack to enable sleep_for (GCC < 4.8)
#ifndef _GLIBCXX_USE_NANOSLEEP
#define _GLIBCXX_USE_NANOSLEEP
#endif  // _GLIBCXX_USE_NANOSLEEP

#include "NodeIMUDriver.h"

namespace hal
{

class NodeIMUFactory : public DeviceFactory<IMUDriverInterface>
{
public:
    NodeIMUFactory(const std::string& name)
        : DeviceFactory<IMUDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<IMUDriverInterface> GetDevice(const Uri& uri)
    {
        NodeIMUDriver* pDriver = new NodeIMUDriver( uri.url );
        return std::shared_ptr<IMUDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static NodeIMUFactory g_NodeIMUFactory("node");

}
