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
        {"size", "640x480", "Capture resolution"},
        {"fps", "30", "Capture framerate"},
        {"rgb", "true", "Capture RGB image"},
        {"depth", "true", "Capture Depth image"},
        {"ir0", "false", "Capture first IR image"},
        {"ir1", "false", "Capture second IR image"},
        {"emitter", "true", "Enable IR emitter"},
        {"exposure", "0", "Exposure value, 0 for auto-exposure"},
        {"gain", "0", "Camera gain"},
      };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
      ImageDim dims      = uri.properties.Get("size", ImageDim(640, 480));
      bool capture_color = uri.properties.Get("rgb", true);
      bool capture_depth = uri.properties.Get("depth", true);
      bool capture_ir0   = uri.properties.Get("ir0", false);
      bool capture_ir1   = uri.properties.Get("ir1", false);
      bool emit_ir       = uri.properties.Get("emitter", true);
      double exposure    = uri.properties.Get("exposure", 0.0);
      double gain        = uri.properties.Get("gain", 0.0);
      int fps            = uri.properties.Get("fps", 30);

      return std::make_shared<RealSense2Driver>(dims.x, dims.y, fps,
          capture_color, capture_depth, capture_ir0, capture_ir1, emit_ir,
          exposure, gain);
    }
};

static RealSense2Factory g_RealSenseFactory("realsense2");

} // namespace hal