#include <HAL/Devices/DriverFactory.h>
#include "ConvertDriver.h"

#include <cstdlib>
#include <string>

namespace hal
{

class ConvertFactory : public DriverFactory<CameraDriverInterface>
{
public:
  ConvertFactory(const std::string& name)
    : DriverFactory<CameraDriverInterface>(name) {}

  std::shared_ptr<CameraDriverInterface> CreateDriver( const Uri& uri )
  {
    // Standard boiler plate... should probably be consolidated elsewhere
    const Uri input_uri = Uri(uri.url);
    std::shared_ptr<CameraDriverInterface> input =
        DeviceDriverRegistry<hal::CameraDriverInterface>::Instance().Create(input_uri);
    ConvertDriver* pDriver = new ConvertDriver( input, uri );
    return std::shared_ptr<CameraDriverInterface>( pDriver );
  }
};

// Register this factory by creating static instance of factory
static ConvertFactory g_ConvertFactory("convert");

}
