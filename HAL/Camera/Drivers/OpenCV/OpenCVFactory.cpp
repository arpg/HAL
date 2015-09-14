#include <HAL/Devices/DriverFactory.h>
#include "OpenCVDriver.h"

namespace hal {

class OpenCVFactory : public DriverFactory<CameraDriverInterface> {
 public:
  OpenCVFactory(const std::string& name)
      : DriverFactory<CameraDriverInterface>(name) {}

  std::shared_ptr<CameraDriverInterface> CreateDriver(const Uri& uri) 
  {
    return std::shared_ptr<CameraDriverInterface>( new OpenCVDriver(uri));
  }
};

// Register this factory by creating static instance of factory
static OpenCVFactory g_OpenCVFactory("opencv");
}  // namespace hal
