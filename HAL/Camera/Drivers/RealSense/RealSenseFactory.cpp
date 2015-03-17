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
	{"ir","0","Select ir or depth image for the second channel, 0=depth"}
      };
    };
    
        
    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
      bool useIR = false;
      std::string sUseIR = uri.properties.Get<std::string>("ir", "0");
      if (sUseIR == "1")
	useIR = true;
	  
        RealSenseDriver* rs = new RealSenseDriver(useIR);
        return std::shared_ptr<CameraDriverInterface>( rs );
    }
};

// Register this factory by creating static instance of factory
static RealSenseFactory g_RealSenseFactory("realsense");

}
