#include <HAL/Devices/DeviceFactory.h>
#include "./AndroidIMUDriver.h"

namespace hal {
class AndroidIMUFactory : public DeviceFactory<IMUDriverInterface> {
 public:
  AndroidIMUFactory(const std::string& name)
      : DeviceFactory<IMUDriverInterface>(name) {
    Params() = {};
  }

  std::shared_ptr<IMUDriverInterface> GetDevice(const Uri& /*uri*/) {
    return std::shared_ptr<IMUDriverInterface>(new AndroidIMUDriver());
  }
};

// Register this factory by creating static instance of factory
static AndroidIMUFactory g_AndroidIMUFactory("android");
}  // end namespace hal
