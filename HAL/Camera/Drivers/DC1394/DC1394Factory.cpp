#include <HAL/Devices/DriverFactory.h>
#include "DC1394Driver.h"


namespace hal
{

class DC1394Factory : public DriverFactory<CameraDriverInterface>
{
public:
    DC1394Factory(const std::string& name)
        : DriverFactory<CameraDriverInterface>(name) {}

    std::shared_ptr<CameraDriverInterface> CreateDriver(const Uri& uri)
    {
      DC1394Driver* pDriver = new DC1394Driver( uri ); 
      return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static DC1394Factory g_DC1394Factory("dc1394");

}
