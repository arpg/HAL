#include <HAL/Devices/DriverFactory.h>
#include "DC1394Driver.h"


namespace hal
{

class DC1394Factory : public DriverFactory<CameraDriverInterface>
{
public:
    DC1394Factory(const std::string& driver_name, const std::vector<param_t>& default_params )
        : DriverFactory<CameraDriverInterface>(driver_name, default_params) {}

    std::shared_ptr<CameraDriverInterface> CreateDriver(const Uri& uri)
    {
      DC1394Driver* pDriver = new DC1394Driver(); 
      pDriver->Init( default_params_, uri );
      return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static DC1394Factory g_DC1394Factory("dc1394",
    {
    {"idN","0","Camera id number."},
    {"mode","MONO8","Video mode: RGB8, MONO8, MONO16, FORMAT7_X"},
    {"size", "640x480", "Capture resolution."},
    {"roi", "0+0+640x480", "ROI resolution for Format7."},
    {"fps", "30.0", "Capture framerate."},
    {"iso", "800", "ISO speed."},
    {"dma", "4", "Number of DMA channels."},
    {"exp", "AUTO", "Sets exposure to absolute value."},
    {"ptgrey_timestamp", "false", "use point grey device timestamp."}
    }
    );

}
