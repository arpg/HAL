#include <HAL/Devices/DeviceFactory.h>
#include "ROSDriver.h"

#define DEFAULT_WIDTH 640
#define DEFAULT_HEIGHT 480
namespace hal
{

class ROSFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    ROSFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
      Params() = {
	{"topics","/image_left","Topics to subscribe to, separated by +"},
	{"width", "DEFAULT_WIDTH","Width of each image"},
	{"height","DEFAULT_HEIGHT","Height of each image"},
      };
    };
    
        
    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {

      std::string topics = uri.properties.Get<std::string>("topics", "/image_left");
      int width = DEFAULT_WIDTH; //uri.properties.Get<std::int>("width", "DEFAULT_WIDTH");
      int height = DEFAULT_HEIGHT; //uri.properties.Get<std::int>("height", "DEFAULT_HEIGHT");

      
      ROSDriver* rs = new ROSDriver(topics, width, height);
      return std::shared_ptr<CameraDriverInterface>( rs );
    }
};

// Register this factory by creating static instance of factory
static ROSFactory g_ROSFactory("ros");

}
