#include <HAL/Devices/DeviceFactory.h>
#include "OpenNI2Driver.h"

namespace hal
{

class OpenNI2Factory : public DeviceFactory<CameraDriverInterface>
{
public:
    OpenNI2Factory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
            {"size", "640x480", "Capture resolution."},
            {"fps", "30", "Capture framerate."},
            {"rgb", "true", "Capture RGB image."},
            {"depth", "true", "Capture depth image."},
            {"ir", "false", "Capture infrared image."},
            {"hw_align", "false", "Align depth with RGB in hardware"},
            {"exposure", "0", "Exposure value, 0 for auto-exposure"},
            {"gain", "0", "Camera gain"},
            {"cmod", "false", "Camera model for software aligning depth with RGB."},
	    {"sn", "", "Open a particular S/N (first available otherwise)"},
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        ImageDim Dims          = uri.properties.Get("size", ImageDim(640,480));
        unsigned int nFPS      = uri.properties.Get("fps", 30);
        bool bRGB              = uri.properties.Get("rgb", true);
        bool bDepth            = uri.properties.Get("depth", true);
        bool bIR               = uri.properties.Get("ir", false);
        bool bAlign            = uri.properties.Get("hw_align", false);
        unsigned int nExposure = uri.properties.Get("exposure", 0);
        unsigned int nGain     = uri.properties.Get("gain", 0);
        std::string scmod   = uri.properties.Get<std::string>("cmod", "");
        std::string dev_sn = uri.properties.Get<std::string>("sn","");

        OpenNI2Driver* pDriver = new OpenNI2Driver(
                    Dims.x, Dims.y, nFPS, bRGB, bDepth, bIR, bAlign,
                    nExposure, nGain, dev_sn, scmod);

        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static OpenNI2Factory g_OpenNI2Factory("openni2");

}
