#include <HAL/Devices/DeviceFactory.h>

#include "EpiphanDriver.h"

namespace hal
{

class EpiphanFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    EpiphanFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
            {"id","0","Camera id."},
            {"name", "EpiphanCam", "Camera name"}
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        unsigned int nCamId     = uri.properties.Get<unsigned int>("id", 0);
        std::string sName       = uri.properties.Get<std::string>("name", "EpiphanCam");
        EpiphanDriver* pDriver = new EpiphanDriver();
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static EpiphanFactory g_EpiphanFactory("epiphan");

}
