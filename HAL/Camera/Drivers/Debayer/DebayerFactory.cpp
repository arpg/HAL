#include <HAL/Devices/DeviceFactory.h>
#include "DebayerDriver.h"

#include <string>

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
                DeviceRegistry<hal::CameraDriverInterface>::Instance().Create(input_uri);


        std::string sMethod =   uri.properties.Get<std::string>("method", "downsample");
        std::string sFilter =   uri.properties.Get<std::string>("filter", "rggb");
        unsigned int nDepth =   uri.properties.Get("depth", 8);
        
        dc1394bayer_method_t Method;
        if( sMethod == "nearest" ) {
            Method = DC1394_BAYER_METHOD_NEAREST;
        } else if( sMethod == "simple" ) {
            Method = DC1394_BAYER_METHOD_SIMPLE;
        } else if( sMethod == "bilinear" ) {
            Method = DC1394_BAYER_METHOD_BILINEAR;
        } else if( sMethod == "hqlinear" ) {
            Method = DC1394_BAYER_METHOD_HQLINEAR;
        } else {
            Method = DC1394_BAYER_METHOD_DOWNSAMPLE;
        }
    
        dc1394color_filter_t Filter;
        if( sFilter == "rggb" ) {
            Filter = DC1394_COLOR_FILTER_RGGB;
        } else if( sFilter == "gbrg" ) {
            Filter = DC1394_COLOR_FILTER_GBRG;
        } else if( sFilter == "grbg" ) {
            Filter = DC1394_COLOR_FILTER_GRBG;
        } else {
            Filter = DC1394_COLOR_FILTER_BGGR;
        }

        DebayerDriver* pDriver = new DebayerDriver( Input, Method, Filter, nDepth );
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static DebayerFactory g_DebayerFactory("debayer");

}
