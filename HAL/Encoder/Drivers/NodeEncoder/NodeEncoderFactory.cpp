#include <HAL/Devices/DeviceFactory.h>

// hack to enable sleep_for (GCC < 4.8)
#define _GLIBCXX_USE_NANOSLEEP

#include "NodeEncoderDriver.h"

namespace hal
{

class NodeEncoderFactory : public DeviceFactory<EncoderDriverInterface>
{
public:
    NodeEncoderFactory(const std::string& name)
        : DeviceFactory<EncoderDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<EncoderDriverInterface> GetDevice(const Uri& uri)
    {
        NodeEncoderDriver* pDriver = new NodeEncoderDriver( uri.url );
        return std::shared_ptr<EncoderDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static NodeEncoderFactory g_NodeEncoderFactory("node");

}
