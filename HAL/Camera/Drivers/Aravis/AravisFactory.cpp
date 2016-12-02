#include <HAL/Devices/DeviceFactory.h>
#include "AravisDriver.h"



namespace hal
{

class AravisFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    AravisFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
      Params() = {
	{"serial", "0", "Serial number of the camera to open (default: first discovered)"},
      };
    };
    
        
    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {

      std::string serial = uri.properties.Get<std::string>("serial", "");
            
      AravisDriver* rs = new AravisDriver(serial);
      return std::shared_ptr<CameraDriverInterface>( rs );
    }
};

// Register this factory by creating static instance of factory
static AravisFactory g_AravisFactory("aravis");

}
