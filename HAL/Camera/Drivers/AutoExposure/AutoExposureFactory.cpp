#include <HAL/Devices/DeviceFactory.h>
#include "AutoExposureDriver.h"

namespace hal
{

class AutoExposureFactory : public DeviceFactory<CameraDriverInterface>
{
  public:

    AutoExposureFactory(const std::string& name) :
      DeviceFactory<CameraDriverInterface>(name)
    {
      Params() =
      {
        {"p", "-1", "Proportional gain (-1 driver default)"},
        {"i", "-1", "Integral gain (-1 driver default)"},
        {"d", "-1", "Derivative gain (-1 driver default)"},
        {"target", "127", "Target mean image intensity"},
        {"roi", "0+0+0x0", "ROI for computing mean intensity"},
        {"limit", "1.0", "Exposure limit (proportional to max value)"},
        {"gain", "0.0", "Constant camera gain (proportional to max value)"},
        {"sync", "true", "Apply exposure to all camera channels"},
        {"channel", "0", "Camera channel for computing exposure"},
        {"color", "-1", "Color channel for computing exposure (-1 all colors)"}
      };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri &uri)
    {
      const double p = uri.properties.Get<double>("p", -1);
      const double i = uri.properties.Get<double>("i", -1);
      const double d = uri.properties.Get<double>("d", -1);
      const double target = uri.properties.Get<double>("target", 127);
      const ImageRoi roi = uri.properties.Get<ImageRoi>("roi", ImageRoi());
      const double limit = uri.properties.Get<double>("limit", 1.0);
      const double gain = uri.properties.Get<double>("gain", 0.0);
      const bool sync = uri.properties.Get<bool>("sync", true);
      const int channel = uri.properties.Get<int>("channel", 0);
      const int color = uri.properties.Get<int>("color", -1);
      std::shared_ptr<AutoExposureInterface> input = GetInput(uri.url);

      return std::make_shared<AutoExposureDriver>(input, p, i, d, target, roi,
          limit, gain, sync, channel, color);
    }

  protected:

    std::shared_ptr<AutoExposureInterface> GetInput(const std::string& url)
    {
      std::shared_ptr<AutoExposureInterface> input;
      std::shared_ptr<CameraDriverInterface> raw_input;
      raw_input = DeviceRegistry<CameraDriverInterface>::Instance().Create(url);
      input = std::dynamic_pointer_cast<AutoExposureInterface>(raw_input);
      if (!input) throw hal::DeviceException("auto-exposure not supported ");
      return input;
    }
};

static AutoExposureFactory g_AutoExposureFactory("autoexp");

} // namespace hal