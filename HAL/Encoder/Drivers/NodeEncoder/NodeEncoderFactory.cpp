#include <HAL/Devices/DeviceFactory.h>

// hack to enable sleep_for (GCC < 4.8)
#ifndef _GLIBCXX_USE_NANOSLEEP
#define _GLIBCXX_USE_NANOSLEEP
#endif  // _GLIBCXX_USE_NANOSLEEP

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
        std::string local_node = uri.properties.Get<std::string>(
              "name", "nodeencoder");

        // parse url: remote/topic
        std::string::size_type p = uri.url.find('/');
        if(p != std::string::npos && p > 0 && p < uri.url.length() - 1)
        {
          std::string remote_node = uri.url.substr(0, p);
          std::string topic = uri.url.substr(p + 1);
          return std::shared_ptr<EncoderDriverInterface>
              (new NodeEncoderDriver(local_node, remote_node, topic));
        }
        else
          throw DeviceException("NodeEncoderDriver: ill-formed URI");
    }
};

// Register this factory by creating static instance of factory
static NodeEncoderFactory g_NodeEncoderFactory("node");

}
