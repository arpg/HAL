#include <HAL/Devices/DeviceFactory.h>
#include <calibu/Calibu.h>
#include "AlignDriver.h"

#include <cstdlib>
#include <string>

namespace hal
{

class AlignFactory : public DeviceFactory<CameraDriverInterface>
{
public:
  AlignFactory(const std::string& name)
    : DeviceFactory<CameraDriverInterface>(name)
  {
    Params() = {
      {"cam", "cameras.xml", "Calibration filename"},
      {"idx", "0", "Channel index of the reference image"}
    };
  }

  std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
  {
    std::string sCam = uri.properties.Get<std::string>("cam", "cameras.xml");
    int nIdx = uri.properties.Get<int>("idx", 0);
    const Uri input_uri = Uri(uri.url);

    // Read calibration file
    std::shared_ptr<calibu::Rig<double>> rig = calibu::ReadXmlRig(sCam);
    if (rig->cameras_.empty())
      throw DeviceException("No cameras loaded from the calibration file");

    // Create input cameras
    std::shared_ptr<CameraDriverInterface> Input =
        DeviceRegistry<hal::CameraDriverInterface>::I().Create(input_uri);

    AlignDriver* pDriver = new AlignDriver( rig, nIdx, Input );
    return std::shared_ptr<CameraDriverInterface>( pDriver );
  }
};

// Register this factory by creating static instance of factory
static AlignFactory g_AlignFactory("align");

}
