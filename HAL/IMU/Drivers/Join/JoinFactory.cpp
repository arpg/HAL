#include <HAL/Devices/DeviceFactory.h>
#include "./JoinDriver.h"

namespace hal {

class JoinFactory : public DeviceFactory<IMUDriverInterface> {
 public:
  JoinFactory(const std::string& name)
      : DeviceFactory<IMUDriverInterface>(name) {
    Params() = {};
  }

  std::shared_ptr<IMUDriverInterface> GetDevice(const Uri& uri) {
    return std::shared_ptr<IMUDriverInterface>(new JoinDriver(
        DeviceRegistry<hal::IMUDriverInterface>::Instance().Create(uri.url)));
  }
};

// Register this factory by creating static instance of factory
static JoinFactory g_JoinFactory("join");

}  // end namespace hal
