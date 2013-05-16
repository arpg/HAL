#include <HAL/Devices/DeviceFactory.h>
#include "DC1394Driver.h"


namespace hal
{

class DC1394Factory : public DeviceFactory<CameraDriverInterface>
{
public:
    DC1394Factory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
            {"id","0","Camera id."},
            {"mode","MONO8","Video mode: RGB8, MONO8, MONO16, FORMAT7_X"},
            {"size", "640x480", "Capture resolution."},
            {"fps", "30", "Capture framerate. For Format7 this is packet size."}
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        unsigned int nCamId     =   uri.properties.Get("id", 0);
        std::string sMode       =   uri.properties.Get<std::string>("mode", "MONO8");
        ImageDim Dims           = uri.properties.Get("size", ImageDim(640,480));
        unsigned int nFPS       = uri.properties.Get("fps", 30);

        DC1394Driver* pDriver = new DC1394Driver(
                    nCamId, sMode, Dims, nFPS
                    );
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static DC1394Factory g_DC1394Factory("dc1394");

}
