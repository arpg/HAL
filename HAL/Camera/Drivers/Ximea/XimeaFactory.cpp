#include <HAL/Devices/DeviceFactory.h>

#include "XimeaDriver.h"


namespace hal
{

class XimeaFactory : public DeviceFactory<CameraDriverInterface>
{
public:
  XimeaFactory(const std::string& name)
    : DeviceFactory<CameraDriverInterface>(name)
  {
    Params() = {
    {"idN", "0", "Camera serial number. Increment N from 0 to (number of serial numbers -1)"},
    {"fps", "DEFAULT", "Capture framerate: [1, 150]"},
    {"exp", "0", "Exposure time (microseconds): [1,1000000] (0: AUTO)"},
    {"gain", "0.5", "Gain (dB): [-2.4, 12.0]"},
    {"mode", "MONO8", "Video mode: RAW8, RAW16, MONO8, MONO16, RGB24, RGB32"},
    {"size", "640x480", "Capture resolution."},
    {"roi", "0+0+640x480", "ROI resolution."},
    {"sync", "0", "Sync type. [0: none, 1: software, 2: hardware]"},
    {"binning", "0", "Binning: Divide frame by this integer in hardware"},
    {"bus_cams", "0", "Divide the total bus bandwidth by this number per camera"}
  };
  }

  std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
  {
    float fps               = uri.properties.Get<float>("fps", 0);
    int exp                 = uri.properties.Get<int>("exp", 0);
    float gain              = uri.properties.Get<float>("gain", 0.5);
    std::string mode        = uri.properties.Get<std::string>("mode", "MONO8");
    ImageDim dims           = uri.properties.Get<ImageDim>("size", ImageDim(640,480));
    ImageRoi ROI            = uri.properties.Get<ImageRoi>("roi", ImageRoi(0,0,0,0));
    int sync                = uri.properties.Get<int>("sync", 0);
    int binning         = uri.properties.Get<int>("binning", 0);
    int bus_cams        = uri.properties.Get<int>("bus_cams", 0);

    std::vector<std::string> vector_ids;
    
    printf("bus_cams: 0x%x\n", bus_cams);
    
    while(true) {
      std::stringstream ss;
      ss << "id" << vector_ids.size();
      const std::string key = ss.str();

      if(!uri.properties.Contains(key)) {
        break;
      }

      vector_ids.push_back(uri.properties.Get<std::string>(key, ""));
    }

    if(ROI.w == 0 && ROI.h == 0) {
      ROI.w = dims.x;
      ROI.h = dims.y;
    }

    XI_IMG_FORMAT xi_mode;
    if (mode == "RAW8") {
      xi_mode = XI_RAW8;
    } else if (mode == "RAW16") {
      xi_mode = XI_RAW16;
    } else if (mode == "MONO16") {
      xi_mode = XI_MONO16;
    } else if (mode == "RGB24") {
      xi_mode = XI_RGB24;
    } else if (mode == "RGB32") {
      xi_mode = XI_RGB32;
    }else {
      xi_mode = XI_MONO8;
    }

    XimeaDriver* pDriver = new XimeaDriver(vector_ids, fps, exp, gain, xi_mode,
                                           ROI, sync, binning, bus_cams);

    return std::shared_ptr<CameraDriverInterface>(pDriver);
  }
};

// Register this factory by creating static instance of factory.
static XimeaFactory g_XimeaFactory("ximea");

}
