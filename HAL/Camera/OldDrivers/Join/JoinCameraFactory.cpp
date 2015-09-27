#include <HAL/Devices/DriverFactory.h>
#include "JoinCameraDriver.h"

namespace hal
{

  class JoinCameraFactory : public DriverFactory<CameraDriverInterface>
  {
    public:
      JoinCameraFactory(const std::string& name)
        : DriverFactory<CameraDriverInterface>(name) {}

      std::shared_ptr<CameraDriverInterface> CreateDriver(const Uri& uri)
      {
        JoinCameraDriver* pDriver = new JoinCameraDriver( uri );
        return std::shared_ptr<CameraDriverInterface>( pDriver );
      }
  };

  // Register this factory by creating static instance of factory
  static JoinCameraFactory g_JoinCameraFactory("join");

}
