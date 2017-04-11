#include <HAL/Devices/DeviceFactory.h>

#include "NinjaV3CarDriver.h"

namespace hal
{

class NinjaV3CarFactory : public DeviceFactory<CarDriverInterface>
{
public:
    NinjaV3CarFactory(const std::string& name)
        : DeviceFactory<CarDriverInterface>(name)
    {
        Params() = {
        {"baud", "115200", "Comport baudrate"},
        {"dev", "/dev/ttyUSB0", "Comport device to open"}
        };
    }

    std::shared_ptr<CarDriverInterface> GetDevice(const Uri& uri)
    {
      int baud = uri.properties.Get<int>("baud", 115200);
      std::string dev = uri.properties.Get<std::string>("dev","/dev/ttyUSB0");
      NinjaV3CarDriver* pDriver = new NinjaV3CarDriver(dev,baud);
      return std::shared_ptr<CarDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static NinjaV3CarFactory g_NinjaV3CarFactory("ninja_v3");

}
