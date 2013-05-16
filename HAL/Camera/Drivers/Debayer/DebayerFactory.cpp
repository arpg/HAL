#include <HAL/Devices/DeviceFactory.h>
#include "DebayerDriver.h"


namespace hal
{

class DebayerFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    DebayerFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
            {"method","downsample","Debayer method: nearest, simple, bilinear, hqlinear, downsample"},
            {"filter","rggb","Debayer filter: rggb, gbrg, grbg, bggr"},
            {"depth","8","Pixel depth: 8 or 16."}
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        const Uri input_uri = Uri(uri.url);

        // Create input camera
        std::shared_ptr<CameraDriverInterface> Input =
                DeviceRegistry<hal::CameraDriverInterface>::I().Create(input_uri);


        std::string sMethod =   uri.properties.Get<std::string>("method", "downsample");
        std::string sFilter =   uri.properties.Get<std::string>("filter", "rggb");
        unsigned int nDepth =   uri.properties.Get("depth", 8);

        DebayerDriver* pDriver = new DebayerDriver( Input, sMethod, sFilter, nDepth );
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static DebayerFactory g_DebayerFactory("debayer");

}
