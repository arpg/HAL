#include <HAL/Devices/DeviceFactory.h>
#include "RealSense.h"

namespace hal
{

class RealSenseFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    RealSenseFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
      Params() = {
	{"ir","0","Select ir or depth image for the second channel, 0=depth"},
	{"sync","0","Attempt to use the SCR value from UVC Video spec to sync depth/rgb streams, 0=unsynced images"}
      };
    };
    
        
    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
      bool useIR = false;
      bool useSync = false;
      std::string sUseIR = uri.properties.Get<std::string>("ir", "0");
      std::string sSync = uri.properties.Get<std::string>("sync", "0");
      if (sUseIR == "1")
	useIR = true;

      if (sSync == "1")
	useSync = true;
      
      RealSenseDriver* rs = new RealSenseDriver(useIR, useSync);
      return std::shared_ptr<CameraDriverInterface>( rs );
    }
};

// Register this factory by creating static instance of factory
static RealSenseFactory g_RealSenseFactory("realsense");

}
