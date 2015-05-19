#include <HAL/Devices/DeviceFactory.h>
#include "ROSDriver.h"

#define DEFAULT_SIZE "640x480"

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
	{"sizes", "DEFAULT_SIZE","Size of each image (wxh), separated by +"},
	{"gray_scale","0","Should the gray images be scaled dynamically to the output range?"},
      };
    };
    
        
    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {

      std::string topics = uri.properties.Get<std::string>("topics", "/image_left");
      std::string sizes = uri.properties.Get<std::string>("sizes", "DEFAULT_SIZE");
      int grayScale = uri.properties.Get<int>("gray_scale", 0);

      
      ROSDriver* rs = new ROSDriver(topics, sizes, grayScale);
      return std::shared_ptr<CameraDriverInterface>( rs );
    }
};

// Register this factory by creating static instance of factory
static ROSFactory g_ROSFactory("ros");

}
