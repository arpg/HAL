#include <HAL/Devices/DriverFactory.h>
#include "CleaveDriver.h"



namespace hal
{

class CleaveFactory : public DriverFactory<CameraDriverInterface>
{
public:
    CleaveFactory(const std::string& name)
        : DriverFactory<CameraDriverInterface>(name)
    {};
 
    std::shared_ptr<CameraDriverInterface> CreateDriver( const Uri& uri)
    {
      hal::Uri subUri(uri.url);
      // Create input camera
      std::shared_ptr<CameraDriverInterface> input =
        DeviceDriverRegistry<hal::CameraDriverInterface>::Instance().Create(subUri);      
      CleaveDriver* rs = new CleaveDriver(input,uri);
      return std::shared_ptr<CameraDriverInterface>( rs );
    }
};

// Register this factory by creating static instance of factory
static CleaveFactory g_CleaveFactory("cleave");

}
