#include <HAL/Devices/DeviceFactory.h>
#include "OccamDriver.h"



namespace hal
{

class OccamFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    OccamFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
      Params() = {
	{"serial", "0", "Serial number of the camera to open (default: first discovered)"},
      };
    };
    
        
    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {

      std::string serial = uri.properties.Get<std::string>("serial", "");
            
      OccamDriver* rs = new OccamDriver(serial);
      return std::shared_ptr<CameraDriverInterface>( rs );
    }
};

// Register this factory by creating static instance of factory
static OccamFactory g_OccamFactory("occam");

}
