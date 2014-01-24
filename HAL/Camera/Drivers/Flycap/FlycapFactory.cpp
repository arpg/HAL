#include <HAL/Devices/DeviceFactory.h>

#include "FlycapDriver.h"


namespace hal
{

class FlycapFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    FlycapFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
            {"id","0","Camera id."},
            {"mode","MONO8","Video mode: RGB8, MONO8, MONO16, FORMAT7_X"},
            {"size", "640x480", "Capture resolution."},
            {"roi", "0+0+640x480", "ROI resolution for Format7."},
            {"fps", "30.0", "Capture framerate."},
            {"iso", "400", "ISO speed."},
            {"dma", "4", "Number of DMA channels."}
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        float fFPS              = uri.properties.Get<float>("fps", 30);

        FlycapDriver* pDriver = new FlycapDriver();
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static FlycapFactory g_FlycapFactory("flycap");

}
