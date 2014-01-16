#include <HAL/Devices/DeviceFactory.h>
#include "KitKatDriver.h"

namespace hal {

class KitKatFactory : public DeviceFactory<CameraDriverInterface> {
 public:
  KitKatFactory(const std::string& name)
      : DeviceFactory<CameraDriverInterface>(name) {
    Params() = {};
  }

  std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& /*uri*/) {
    return std::shared_ptr<CameraDriverInterface>(new KitKatDriver);
  }
};

// Register this factory by creating static instance of factory
static KitKatFactory g_KitKatFactory("kitkat");

}
