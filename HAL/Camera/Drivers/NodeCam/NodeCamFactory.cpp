#include <HAL/Devices/DriverFactory.h>
#include <HAL/Devices/DeviceException.h>
#include "NodeCamDriver.h"

namespace hal
{

class NodeCamFactory : public DriverFactory<CameraDriverInterface>
{
public:
  NodeCamFactory(const std::string& name)
      : DriverFactory<CameraDriverInterface>(name)
  {
    Params() = {};
  }

  std::shared_ptr<CameraDriverInterface> CreateDriver(const Uri& uri)
  {
    std::string local_node = uri.properties.Get<std::string>("name", "nodecam");
    double timeout = uri.properties.Get<double>("timeout", 3.);

    // parse url: remote/topic
    std::string::size_type p = uri.url.find('/');
    if(p != std::string::npos && p > 0 && p < uri.url.length() - 1)
    {
      std::string remote_node = uri.url.substr(0, p);
      std::string topic = uri.url.substr(p + 1);
      return std::shared_ptr<CameraDriverInterface>
          (new NodeCamDriver(local_node, remote_node, topic, timeout));
    }
    else
      throw DeviceException("NodeCamDriver: ill-formed URI");
  }
};

// Register this factory by creating static instance of factory
static NodeCamFactory g_NodeCamFactory("node");

}
