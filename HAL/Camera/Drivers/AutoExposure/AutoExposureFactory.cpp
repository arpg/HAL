#include <HAL/Devices/DeviceFactory.h>
#include <HAL/Camera/Drivers/AutoExposure/AutoExposureDriver.h>
#include <unistd.h>

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
        {"target", "100", "The target mean brightness of the image."},
      };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
      std::shared_ptr<CameraDriverInterface> input;
      auto instance = DeviceRegistry<hal::CameraDriverInterface>::Instance();
      input = instance.Create(uri.url);

      const int desiredLuminance = uri.properties.Get("target", 100);

      AutoExposureDriver* driver = new AutoExposureDriver(input);
      driver->SetDesiredLuminance(desiredLuminance);
      return std::shared_ptr<CameraDriverInterface>(driver);
    }
};

// Register this factory by creating static instance of factory
static AutoExposureFactory g_AutoExposureFactory1("autoexposure");
static AutoExposureFactory g_AutoExposureFactory2("autoexp");

} // namespace hal