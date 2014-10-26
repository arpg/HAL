#include <HAL/Devices/DeviceFactory.h>

#include "XimeaDriver.h"


namespace hal
{

class XimeaFactory : public DeviceFactory<CameraDriverInterface>
{
public:
  XimeaFactory(const std::string& name)
    : DeviceFactory<CameraDriverInterface>(name)
  {
    Params() = {
    {"idN", "0", "Camera serial number."},
    {"fps", "DEFAULT", "Capture framerate: 30, 60, 120"},
    {"mode", "MONO8", "Video mode: MONO8, MONO16"},
    {"size", "640x480", "Capture resolution."},
    {"roi", "0+0+640x480", "ROI resolution."},
  };
  }

  std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
  {
    float fps               = uri.properties.Get<float>("fps", 0);
    std::string mode        = uri.properties.Get<std::string>("mode", "MONO8");
    ImageDim dims           = uri.properties.Get<ImageDim>("size", ImageDim(640,480));
    ImageRoi ROI            = uri.properties.Get<ImageRoi>("roi", ImageRoi(0,0,0,0));

    std::vector<unsigned int> vector_ids;

    while(true) {
      std::stringstream ss;
      ss << "id" << vector_ids.size();
      const std::string key = ss.str();

      if(!uri.properties.Contains(key)) {
        break;
      }

      vector_ids.push_back(uri.properties.Get<unsigned int>(key, 0));
    }

    if(ROI.w == 0 && ROI.h == 0) {
      ROI.w = dims.x;
      ROI.h = dims.y;
    }

    XI_IMG_FORMAT xi_mode;
    if (mode == "MONO16") {
      xi_mode = XI_MONO16;
    } else {
      xi_mode = XI_MONO8;
    }

    XimeaDriver* pDriver = new XimeaDriver(vector_ids, fps, xi_mode, ROI);

    return std::shared_ptr<CameraDriverInterface>(pDriver);
  }
};

// Register this factory by creating static instance of factory.
static XimeaFactory g_XimeaFactory("ximea");

}
