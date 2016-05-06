#include <HAL/Devices/DeviceFactory.h>
#include <HAL/Utils/StringUtils.h>
#include "GladiatorDriver.h"

namespace hal
{

class GladiatorFactory : public DeviceFactory<IMUDriverInterface>
{
public:
    GladiatorFactory(const std::string& name)
        : DeviceFactory<IMUDriverInterface>(name)
    {
        Params() = {
	  {"port","/dev/ttyUSB0","RS485-capable serial port"},
        };
    }

    std::shared_ptr<IMUDriverInterface> GetDevice(const Uri& uri)
    {
      std::string port = uri.properties.Get<std::string>("port", "/dev/ttyUSB0");
      GladiatorDriver* pDriver = new GladiatorDriver(port.c_str());
      return std::shared_ptr<IMUDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static GladiatorFactory g_GladiatorFactory("gladiator");

}
