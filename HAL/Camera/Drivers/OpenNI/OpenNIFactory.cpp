#include <HAL/Devices/DeviceFactory.h>
#include "OpenNIDriver.h"

namespace hal
{

class OpenNIFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    OpenNIFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
            {"id","0","Camera id."},
            {"name", "OpenNICam", "Camera name."},
            {"size", "640x480", "Capture resolution."},
            {"fps", "30", "Capture framerate."},
            {"rgb", "true", "Capture RGB image."},
            {"depth", "true", "Capture depth image."},
            {"ir", "false", "Capture infrared image."},
            {"align", "false", "Align depth with RGB."},
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        unsigned int nCamId     = uri.properties.Get<unsigned int>("id", 0);
        std::string sName       = uri.properties.Get<std::string>("name", "OpenNICam");
        ImageDim Dims       = uri.properties.Get("size", ImageDim(640,480));
        unsigned int nFPS   = uri.properties.Get("fps", 30);
        bool bRGB           = uri.properties.Get("rgb", true);
        bool bDepth         = uri.properties.Get("depth", true);
        bool bIR            = uri.properties.Get("ir", false);
        bool bAlign         = uri.properties.Get("align", false);

        OpenNIDriver* pDriver = new OpenNIDriver(
                    Dims.x, Dims.y, nFPS, bRGB, bDepth, bIR, bAlign
                    );
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static OpenNIFactory g_OpenNIFactory("openni");

}
