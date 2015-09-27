#include <HAL/Devices/DriverFactory.h>
#include "KitKatDriver.h"

namespace hal {

class KitKatFactory : public DriverFactory<CameraDriverInterface> {
 public:
  KitKatFactory(const std::string& name)
      : DriverFactory<CameraDriverInterface>(name) {
    Params() = {};
  }

  std::shared_ptr<CameraDriverInterface> CreateDriver( const Uri& /*uri*/) {
    return std::shared_ptr<CameraDriverInterface>(new KitKatDriver);
  }
};

// Register this factory by creating static instance of factory
static KitKatFactory g_KitKatFactory("kitkat");

}
