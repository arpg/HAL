#include <HAL/Devices/DeviceFactory.h>
#include "ConvertDriver.h"

#include <string>

namespace hal
{

class ConvertFactory : public DeviceFactory<CameraDriverInterface>
{
public:
  ConvertFactory(const std::string& name)
    : DeviceFactory<CameraDriverInterface>(name)
  {
    Params() = {
      {"fmt", "MONO8", "Video format: MONO8, RGB8"}
  };
  }

  std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
  {
    std::string sFormat = uri.properties.Get<std::string>("fmt", "MONO8");

    const Uri input_uri = Uri(uri.url);

    // Create input camera
    std::shared_ptr<CameraDriverInterface> Input =
        DeviceRegistry<hal::CameraDriverInterface>::I().Create(input_uri);

    ConvertDriver* pDriver = new ConvertDriver( Input, sFormat );
    return std::shared_ptr<CameraDriverInterface>( pDriver );
  }
};

// Register this factory by creating static instance of factory
static ConvertFactory g_ConvertFactory("convert");

}
