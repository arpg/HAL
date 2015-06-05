#include <HAL/Devices/DeviceFactory.h>
#include "CleaveDriver.h"



namespace hal
{

class CleaveFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    CleaveFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
      Params() = {
	{"max", "0", "Maximum channels to pass through"},
      };
    };
    
        
    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
      hal::Uri subUri(uri.url);
      int maxChannel = uri.properties.Get<int>("max", 0);
      // Create input camera
      std::shared_ptr<CameraDriverInterface> InCam =
	DeviceRegistry<hal::CameraDriverInterface>::I().Create(subUri);      
      CleaveDriver* rs = new CleaveDriver(InCam, maxChannel);
      return std::shared_ptr<CameraDriverInterface>( rs );
    }
};

// Register this factory by creating static instance of factory
static CleaveFactory g_CleaveFactory("cleave");

}
