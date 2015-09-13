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
	{"max", "0", "Maximum channel number to pass through"},
	{"min", "0", "Minimum channel number to pass through"},
      };
    };
    
        
    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
      hal::Uri subUri(uri.url);
      int maxChannel = uri.properties.Get<int>("max", 0);
      int minChannel = uri.properties.Get<int>("min", 0);

      if (maxChannel < minChannel)
	{
	  printf("Cleave: Max channel number [%u] is not >= min channel [%u]\n", maxChannel, minChannel);
	  return NULL;
	}
      // Create input camera
      std::shared_ptr<CameraDriverInterface> InCam =
	DeviceRegistry<hal::CameraDriverInterface>::Instance().Create(subUri);      
      CleaveDriver* rs = new CleaveDriver(InCam, maxChannel, minChannel);
      return std::shared_ptr<CameraDriverInterface>( rs );
    }
};

// Register this factory by creating static instance of factory
static CleaveFactory g_CleaveFactory("cleave");

}
