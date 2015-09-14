#include <HAL/Devices/DriverFactory.h>
#include "./AndroidIMUDriver.h"

namespace hal {
class AndroidIMUFactory : public DriverFactory<IMUDriverInterface> {
 public:
  AndroidIMUFactory(const std::string& name)
      : DriverFactory<IMUDriverInterface>(name) {
    Params() = {};
  }

  std::shared_ptr<IMUDriverInterface> CreateDriver(const Uri& /*uri*/) {
    return std::shared_ptr<IMUDriverInterface>(new AndroidIMUDriver());
  }
};

// Register this factory by creating static instance of factory
static AndroidIMUFactory g_AndroidIMUFactory("android");
}  // end namespace hal
