#include <HAL/Devices/DeviceFactory.h>
#include "RealSense2Driver.h"

namespace hal
{

class RealSense2Factory : public DeviceFactory<CameraDriverInterface>
{
  public:

    RealSense2Factory(const std::string& name) :
      DeviceFactory<CameraDriverInterface>(name)
    {
      Params() =
      {
        {"idN", "0", "Camera serial number, where N is zero-base index"},
        {"size", "640x480", "Capture resolution"},
        {"fps", "30", "Capture framerate"},
        {"rgb", "true", "Capture RGB image"},
        {"depth", "true", "Capture Depth image"},
        {"ir0", "false", "Capture first IR image"},
        {"ir1", "false", "Capture second IR image"},
        {"emitter", "0.462", "Laser emitter strength, 0 to disable"},
        {"exposure", "0", "RGB exposure value, 0 for auto-exposure"},
        {"gain", "64", "RGB camera gain"},
      };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
      ImageDim dims      = uri.properties.Get("size", ImageDim(640, 480));
      bool capture_color = uri.properties.Get("rgb", true);
      bool capture_depth = uri.properties.Get("depth", true);
      bool capture_ir0   = uri.properties.Get("ir0", false);
      bool capture_ir1   = uri.properties.Get("ir1", false);
      double exposure    = uri.properties.Get("exposure", 0.0);
      double gain        = uri.properties.Get("gain", 64);
      double emitter     = uri.properties.Get("emitter", 0.462);
      int frame_rate     = uri.properties.Get("fps", 30);

      std::vector<std::string> ids;

      while (true)
      {
        std::stringstream stream;
        stream << "id" << ids.size();
        const std::string key = stream.str();
        if (!uri.properties.Contains(key)) break;
        ids.push_back(uri.properties.Get<std::string>(key, ""));
      }

      std::shared_ptr<RealSense2Driver> driver =
          std::make_shared<RealSense2Driver>(dims.x, dims.y, frame_rate,
          capture_color, capture_depth, capture_ir0, capture_ir1, ids);

      if (capture_color)
      {
        const int channel = capture_ir0 + capture_ir1;
        driver->SetExposure(exposure, channel);
        driver->SetGain(gain, channel);
      }

      for (size_t i = 0; i < driver->NumDevices(); ++i)
      {
        driver->SetEmitter(i, emitter);
      }

      return driver;
    }
};

static RealSense2Factory g_RealSenseFactory("realsense2");

} // namespace hal