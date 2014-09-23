#include <HAL/Devices/DeviceFactory.h>
#include "ConvertDriver.h"

#include <cstdlib>
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
      {"fmt", "MONO8", "Output video format: MONO8, RGB8, BGR8"},
      {"range", "1", "Range of values of 16 and 32 bit images: ir (1023), "
                     "depth (4500) or numerical value"}
  };
  }

  std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
  {
    std::string sFormat = uri.properties.Get<std::string>("fmt", "MONO8");
    std::string sRange = uri.properties.Get<std::string>("range", "1");
    double dRange;

    if(sRange == "ir")
      dRange = 1023; // OpenNi uses the 10 l.s.bits only (range [0, 1023])
    else if(sRange == "ir2")
      dRange = 65535; // max infrared value in libfreenect2
    else if(sRange == "depth")
      dRange = 4500; // max range (mm) of asus xtion pro live
    else {
      dRange = strtod(sRange.c_str(), nullptr);
      if(dRange == 0.) dRange = 1.;
    }

    const Uri input_uri = Uri(uri.url);

    // Create input camera
    std::shared_ptr<CameraDriverInterface> Input =
        DeviceRegistry<hal::CameraDriverInterface>::I().Create(input_uri);

    ConvertDriver* pDriver = new ConvertDriver( Input, sFormat, dRange );
    return std::shared_ptr<CameraDriverInterface>( pDriver );
  }
};

// Register this factory by creating static instance of factory
static ConvertFactory g_ConvertFactory("convert");

}
