#include <HAL/Devices/DeviceFactory.h>
#include "UvcDriver.h"

namespace hal
{

class UvcFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    UvcFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
            {"id","0","Camera id."},
            {"name", "ProtoCam", "Camera name."},
        };
    }
        
    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        unsigned int nCamId     = uri.properties.Get<unsigned int>("id", 0);
        std::string sName       = uri.properties.Get<std::string>("name", "UVCCam");

        UvcDriver* Uvc = new UvcDriver();
        return std::shared_ptr<CameraDriverInterface>( Uvc );
    }
};

// Register this factory by creating static instance of factory
static UvcFactory g_UvcFactory("uvc");

}
