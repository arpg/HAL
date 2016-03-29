#include <HAL/Devices/DeviceFactory.h>
#include <HAL/Utils/StringUtils.h>

#include "ROSImuDriver.h"

#define DEFAULT_SIZE "640x480"

namespace hal
{

class ROSImuFactory : public DeviceFactory<IMUDriverInterface>
{
public:
    ROSImuFactory(const std::string& name)
        : DeviceFactory<IMUDriverInterface>(name)
    {
      Params() = {
	{"topic","/imu","Topic to subscribe to"},
      };
    };
    
        
    std::shared_ptr<IMUDriverInterface> GetDevice(const Uri& uri)
    {

      std::string topic = uri.properties.Get<std::string>("topic", "/imu");
      
      ROSImuDriver* rs = new ROSImuDriver(topic);
      return std::shared_ptr<IMUDriverInterface>( rs );
    }
};

// Register this factory by creating static instance of factory
static ROSImuFactory g_ROSImuFactory("ros_imu");

}
