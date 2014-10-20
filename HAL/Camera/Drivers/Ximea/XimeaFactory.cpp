#include <HAL/Devices/DeviceFactory.h>

#include "XimeaDriver.h"


namespace hal
{

class XimeaFactory : public DeviceFactory<CameraDriverInterface>
{
public:
  XimeaFactory(const std::string& name)
    : DeviceFactory<CameraDriverInterface>(name)
  {
    Params() = {
    {"idN","0","Camera serial number."},
    {"mode","FORMAT7_0","Video mode: FORMAT7_X"},
    {"size", "640x480", "Capture resolution."},
    {"roi", "0+0+640x480", "ROI resolution for Format7."},
  };
  }

  std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
  {
    std::string sMode       = uri.properties.Get<std::string>("mode", "MONO8");
    ImageDim Dims           = uri.properties.Get<ImageDim>("size", ImageDim(640,480));
    ImageRoi ROI            = uri.properties.Get<ImageRoi>("roi", ImageRoi(0,0,0,0));

    std::vector<unsigned int> vID;

    while(true)
    {
      std::stringstream ss;
      ss << "id" << vID.size();
      const std::string key = ss.str();

      if(!uri.properties.Contains(key)) {
        break;
      }

      vID.push_back( uri.properties.Get<unsigned int>( key, 0 ) );
    }

    if( ROI.w == 0 && ROI.h == 0 ) {
      ROI.w = Dims.x;
      ROI.h = Dims.y;
    }

    /*Ximea::Mode Mode;
    if( sMode == "FORMAT7_1" ) {
      Mode = Ximea::MODE_1;
    } else if( sMode == "FORMAT7_2" ) {
      Mode = Ximea::MODE_2;
    } else if( sMode == "FORMAT7_3" ) {
      Mode = Ximea::MODE_3;
    } else if( sMode == "FORMAT7_4" ) {
      Mode = Ximea::MODE_4;
    } else if( sMode == "FORMAT7_5" ) {
      Mode = Ximea::MODE_5;
    } else if( sMode == "FORMAT7_6" ) {
      Mode = Ximea::MODE_6;
    } else if( sMode == "FORMAT7_7" ) {
      Mode = Ximea::MODE_7;
    } else if( sMode == "FORMAT7_8" ) {
      Mode = Ximea::MODE_8;
    } else if( sMode == "FORMAT7_9" ) {
      Mode = Ximea::MODE_9;
    } else if( sMode == "FORMAT7_10" ) {
      Mode = Ximea::MODE_10;
    } else if( sMode == "FORMAT7_11" ) {
      Mode = Ximea::MODE_11;
    } else if( sMode == "FORMAT7_12" ) {
      Mode = Ximea::MODE_12;
    } else if( sMode == "FORMAT7_13" ) {
      Mode = Ximea::MODE_13;
    } else if( sMode == "FORMAT7_14" ) {
      Mode = Ximea::MODE_14;
    } else if( sMode == "FORMAT7_15" ) {
      Mode = Ximea::MODE_15;
    } else {
      Mode = Ximea::MODE_0;
    }
*/

    XimeaDriver* pDriver = new XimeaDriver(vID, ROI);

    return std::shared_ptr<CameraDriverInterface>( pDriver );
  }
};

// Register this factory by creating static instance of factory
static XimeaFactory g_XimeaFactory("ximea");

}
