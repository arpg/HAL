#include <HAL/Devices/DriverFactory.h>
#include "./JoinDriver.h"

namespace hal {

class JoinFactory : public DriverFactory<IMUDriverInterface> {
 public:
  JoinFactory(const std::string& name)
      : DriverFactory<IMUDriverInterface>(name) {}

  std::shared_ptr<IMUDriverInterface> CreateDriver(const Uri& uri) 
  {
    return std::shared_ptr<IMUDriverInterface>(new JoinDriver(
        DeviceRegistry<hal::IMUDriverInterface>::Instance().Create(uri.url)));
  }
};

// Register this factory by creating static instance of factory
static JoinFactory g_JoinFactory("join");

}  // end namespace hal
