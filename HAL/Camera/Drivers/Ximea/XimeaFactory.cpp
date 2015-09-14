#include <HAL/Devices/DriverFactory.h>

#include "XimeaDriver.h"


namespace hal
{

class XimeaFactory : public DriverFactory<CameraDriverInterface>
{
public:
  XimeaFactory(const std::string& name)
    : DriverFactory<CameraDriverInterface>(name) {}

  std::shared_ptr<CameraDriverInterface> CreateDriver(const Uri& uri)
  {
    XimeaDriver* pDriver = new XimeaDriver( uri );
    return std::shared_ptr<CameraDriverInterface>(pDriver);
  }
};

// Register this factory by creating static instance of factory.
static XimeaFactory g_XimeaFactory("ximea");

}
