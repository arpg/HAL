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

    FlyCapture2::Mode Mode;
    if( sMode == "FORMAT7_1" ) {
      Mode = FlyCapture2::MODE_1;
    } else if( sMode == "FORMAT7_2" ) {
      Mode = FlyCapture2::MODE_2;
    } else if( sMode == "FORMAT7_3" ) {
      Mode = FlyCapture2::MODE_3;
    } else if( sMode == "FORMAT7_4" ) {
      Mode = FlyCapture2::MODE_4;
    } else if( sMode == "FORMAT7_5" ) {
      Mode = FlyCapture2::MODE_5;
    } else if( sMode == "FORMAT7_6" ) {
      Mode = FlyCapture2::MODE_6;
    } else if( sMode == "FORMAT7_7" ) {
      Mode = FlyCapture2::MODE_7;
    } else if( sMode == "FORMAT7_8" ) {
      Mode = FlyCapture2::MODE_8;
    } else if( sMode == "FORMAT7_9" ) {
      Mode = FlyCapture2::MODE_9;
    } else if( sMode == "FORMAT7_10" ) {
      Mode = FlyCapture2::MODE_10;
    } else if( sMode == "FORMAT7_11" ) {
      Mode = FlyCapture2::MODE_11;
    } else if( sMode == "FORMAT7_12" ) {
      Mode = FlyCapture2::MODE_12;
    } else if( sMode == "FORMAT7_13" ) {
      Mode = FlyCapture2::MODE_13;
    } else if( sMode == "FORMAT7_14" ) {
      Mode = FlyCapture2::MODE_14;
    } else if( sMode == "FORMAT7_15" ) {
      Mode = FlyCapture2::MODE_15;
    } else {
      Mode = FlyCapture2::MODE_0;
    }


    FlycapDriver* pDriver = new FlycapDriver(vID, Mode, ROI);

    return std::shared_ptr<CameraDriverInterface>( pDriver );
  }
};

// Register this factory by creating static instance of factory
static FlycapFactory g_FlycapFactory("flycap");

}
