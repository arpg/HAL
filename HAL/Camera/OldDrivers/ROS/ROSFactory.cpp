#include <HAL/Devices/DriverFactory.h>
#include "ROSDriver.h"

namespace hal
{

class ROSFactory : public DriverFactory<CameraDriverInterface>
{
public:
    ROSFactory(const std::string& name)
      : DriverFactory<CameraDriverInterface>(name) {}
 
    std::shared_ptr<CameraDriverInterface> CreateDriver(const Uri& uri)
    {
      return std::shared_ptr<CameraDriverInterface>( new ROSDriver(uri) );
    }
};

// Register this factory by creating static instance of factory
static ROSFactory g_ROSFactory("ros");

}
