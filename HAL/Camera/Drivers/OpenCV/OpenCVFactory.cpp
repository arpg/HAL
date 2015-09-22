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

    unsigned int nCamId = uri.properties.Get<unsigned int>("id", 0);

    const std::string path = ExpandTildePath(uri.url);
    if(path.empty()){
		std::string sName     = uri.properties.Get<std::string>("name", "OpenCVCam");
		return std::shared_ptr<CameraDriverInterface>(
				new OpenCVDriver(nCamId, bGrey));
	}else{
		std::string sName     = uri.properties.Get<std::string>("name", "OpenCVVideoFile");
		return std::shared_ptr<CameraDriverInterface>(
						new OpenCVDriver(path, bGrey));
	}
  }
};

// Register this factory by creating static instance of factory
static OpenCVFactory g_OpenCVFactory("opencv");
}  // namespace hal
