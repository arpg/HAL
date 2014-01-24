#include <HAL/Devices/DeviceFactory.h>
#include "OpenCVDriver.h"

namespace hal {

class OpenCVFactory : public DeviceFactory<CameraDriverInterface> {
 public:
  OpenCVFactory(const std::string& name)
      : DeviceFactory<CameraDriverInterface>(name) {
    Params() = {};
  }

  std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri) {
    bool bGrey            = uri.properties.Get<bool>("grey", false);
    std::string sName     = uri.properties.Get<std::string>("name", "OpenCVCam");

    unsigned int nCamId = uri.properties.Get<unsigned int>("id", 0);
    bool bGrey = uri.properties.Get<bool>("grey", false);
    return std::shared_ptr<CameraDriverInterface>(
        new OpenCVDriver(nCamId, bGrey));
  }
};

// Register this factory by creating static instance of factory
static OpenCVFactory g_OpenCVFactory("opencv");
}  // namespace hal
