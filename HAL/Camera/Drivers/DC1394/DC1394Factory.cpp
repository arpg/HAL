#include <HAL/Devices/DriverFactory.h>
#include "DC1394Driver.h"


namespace hal
{

  class DC1394Factory : public DriverFactory<CameraDriverInterface>
  {
    public:

      // Called by static global instance below to register this device driver
      DC1394Factory(
          const std::string& driver_name, const std::vector<param_t>& default_params )
        : DriverFactory<CameraDriverInterface>(driver_name, default_params) {}


      // Called by CameraDevice::Init function to allocate and initialize a
      // Driver of this type. 
      std::shared_ptr<CameraDriverInterface> CreateDriver( 
          PropertyMap& props, // Output param
          const Uri& uri
          )
      {
        if( !InitDevicePropertyMap( props, uri ) ){
          return nullptr; 
        }
        return std::shared_ptr<CameraDriverInterface>( new DC1394Driver(props) );
      }
  };

  // Register this factory by creating static instance of factory
  static DC1394Factory g_DC1394Factory("dc1394",
      {
      {"idN","0","Camera id number."},
      {"mode","MONO8","Video mode: RGB8, MONO8, MONO16, FORMAT7_0, FORMAT7_1, FORMAT7_2, etc."},
      {"size", "640x480", "Capture resolution."},
      {"roi", "0+0+0+0", "ROI resolution for Format7."},
      {"fps", "30.0", "Capture framerate."},
      {"iso", "800", "ISO speed."},
      {"dma", "8", "Number of DMA channels."},
      {"exp", "AUTO", "Sets exposure to absolute value."},
      {"ptgrey_timestamp", "false", "use point grey device timestamp."},
      {"image_format", "0x1909", "image format, coincides with OpenGL image format enum."},
      {"image_type", "0x1401", "image piexl type, coincides with OpenGL image type enum."},
      {"depth", "8", "bits per pixel."},
      {"debayer_method", "bilinear", "debayer method: none, nearest, simple"
			                                 ", downsample, bilinear,hqlinear, vng, ahd"},
      {"debayer_filter", "gbrg", "debayer filter: rggb, gbrg, grbg"}
      }
      );
}
