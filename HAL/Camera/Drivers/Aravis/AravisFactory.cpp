#include <HAL/Devices/DeviceFactory.h>
#include "AravisDriver.h"



namespace hal
{

class AravisFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    AravisFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
      Params() = {
	{"dev", "", "Device name of the camera to open (default: first discovered)"},
	{"width","", "ROI width, default: full"},
	{"height","", "ROI height, default: full"},
	{"x","", "ROI left offset, default: 0"},
	{"y","", "ROI top offset, default: 0"},
	{"exposure","", "Exposure time (uS), default: 15000"},
	{"gain","", "Gain (dB), floating point, default: 0"},
	{"bandwidth","", "Bandwidth limit to set in Mbps, default: -1 (no limit)"},
      };
    };
    
        
    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {

      std::string dev = uri.properties.Get<std::string>("dev", "");
      int width = uri.properties.Get<int>("width",0);
      int height = uri.properties.Get<int>("height", 0);
      int x = uri.properties.Get<int>("x",0);
      int y = uri.properties.Get<int>("y",0);
      int exposure = uri.properties.Get<int>("exposure", 15000);
      float gain = uri.properties.Get<float>("gain", 0.0);
      int bandwidth = uri.properties.Get<int>("bandwidth", -1);
      
      AravisDriver* rs = new AravisDriver(dev, width, height, x, y, exposure, gain, bandwidth);
      return std::shared_ptr<CameraDriverInterface>( rs );
    }
};

// Register this factory by creating static instance of factory
static AravisFactory g_AravisFactory("aravis");

}
