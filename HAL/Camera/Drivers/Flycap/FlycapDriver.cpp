#include <iostream>

#include "FlycapDriver.h"

using namespace hal;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void FlycapDriver::_CheckError( FlyCapture2::Error err )
{
  if( err != FlyCapture2::PGRERROR_OK ) {
    err.PrintErrorTrace();
    throw DeviceException("Flycapture SDK exception!");
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
FlycapDriver::FlycapDriver(
    std::vector<unsigned int>& vID,
    FlyCapture2::Mode Mode,
    ImageRoi ROI
    )
{
  // TODO(jmf)
  // * Support other formats aside from format_7??
  // * Have adjustable framerates?
  // * Config some GPIO pins.. at least triggerIN and out?
  // * Configure some image properties? gain, etc.
  // * This only work for blackflies... too much hardcoded.

  // initialize
  const int max_width = 1600;
  const int max_height = 1200;

  m_nImgWidth  = ROI.w;
  m_nImgHeight = ROI.h;

  FlyCapture2::Error error;

  FlyCapture2::BusManager BusMgr;
  unsigned int            nTotalCams;

  error = BusMgr.GetNumOfCameras(&nTotalCams);
  _CheckError(error);

  if (nTotalCams == 0) {
    throw DeviceException("No cameras found!");
  }

  if (nTotalCams < vID.size()) {
    throw DeviceException("Less cameras detected than those requested!");
  }

  unsigned int nNumCams;
  // If no ids are ovided, all cameras will be opened.
  if (vID.empty()) {
    nNumCams = nTotalCams;
  } else {
    nNumCams = vID.size();
  }

  // prepare Format 7 config
  FlyCapture2::Format7ImageSettings F7Config;
  F7Config.mode = Mode;
  F7Config.height = m_nImgHeight;
  F7Config.width = m_nImgWidth;
  if (ROI.x == 0 && ROI.y == 0) {
    F7Config.offsetX = max_width / 2 - m_nImgWidth / 2;
    F7Config.offsetY = max_height / 2 - m_nImgHeight / 2;
  } else {
    F7Config.offsetX = ROI.x;
    F7Config.offsetY = ROI.y;
  }
  F7Config.pixelFormat = FlyCapture2::PIXEL_FORMAT_MONO8;
  const unsigned int PacketSize = 1400;

  /*
    // image properties
    Property P_Shutter;
    P_Shutter.type = SHUTTER;
    P_Shutter.onOff = true;
    P_Shutter.autoManualMode = false;
    P_Shutter.absControl = true;
    P_Shutter.absValue = 8;

    Property P_Exposure;
    P_Exposure.type = AUTO_EXPOSURE;
    P_Exposure.onOff = false;
    P_Exposure.autoManualMode = false;
    P_Exposure.absControl = true;
    P_Exposure.absValue = -3.5;

    Property P_Gain;
    P_Gain.type = GAIN;
    P_Gain.onOff = true;
    P_Gain.autoManualMode = false;
    P_Gain.absControl = true;
    P_Gain.absValue = 3;

    Property P_Brightness;
    P_Brightness.type = BRIGHTNESS;
    P_Brightness.onOff = true;
    P_Brightness.autoManualMode = false;
    P_Brightness.absControl = false;
    P_Brightness.absValue = 50;

    Property P_Sharpness;
    P_Sharpness.type = SHARPNESS;
    P_Sharpness.onOff = true;
    P_Sharpness.autoManualMode = false;
    P_Sharpness.absControl = false;
    P_Sharpness.absValue = 0;
    */

  for(unsigned int ii = 0; ii < nNumCams; ++ii) {
    FlyCapture2::PGRGuid         GUID;

    // look for camera
    if(vID.empty()) {
      error = BusMgr.GetCameraFromIndex(ii, &GUID);
      _CheckError(error);
    } else {
      error = BusMgr.GetCameraFromSerialNumber(vID[ii], &GUID);
      _CheckError(error);
    }
    std::cout << "Setting up camera " << ii << std::endl;

    // connect to camera
    FlyCapture2::Camera* pCam = new FlyCapture2::Camera;
    error = pCam->Connect(&GUID);
    _CheckError(error);

    // set video mode and framerate
    error = pCam->SetFormat7Configuration(&F7Config, PacketSize);
    _CheckError(error);

    // set jumbo packet config




    /*
        error = m_Cam1.SetProperty( &P_Shutter );
        CheckError(error);
        error = m_Cam1.SetProperty( &P_Exposure );
        CheckError(error);
        error = m_Cam1.SetProperty( &P_Gain );
        CheckError(error);
        error = m_Cam1.SetProperty( &P_Brightness );
        CheckError(error);
        error = m_Cam1.SetProperty( &P_Sharpness );
        CheckError(error);
        */

    /* */
    // prepare trigger. first camera always strobes, others get trigger.
    if (ii == 0 && false) {
      FlyCapture2::TriggerMode Trigger;

      // external trigger is disabled
      Trigger.onOff = false;
      Trigger.mode = 0;
      Trigger.parameter = 0;
      Trigger.source = 0;

      // set trigger
      error = pCam->SetTriggerMode( &Trigger );
      _CheckError(error);

      // prepare strobe
      FlyCapture2::StrobeControl Strobe;

      // set GPIO pin direction to out
      const unsigned int StrobeOut = 1;
      pCam->SetGPIOPinDirection( StrobeOut, 1 );

      // set GPIO as strobe
      Strobe.onOff = true;
      Strobe.source = StrobeOut;
      Strobe.delay = 0;
      Strobe.duration = 50;
      Strobe.polarity = 0;

      error = pCam->SetStrobe( &Strobe );
      _CheckError(error);
    } else {
      FlyCapture2::TriggerMode Trigger;

      // set GPIO pin direction to in
      const unsigned int TriggerIn = 0;
      pCam->SetGPIOPinDirection(TriggerIn, 0);

      // external trigger is enabled
      Trigger.onOff = true;
      Trigger.mode = 0;
      Trigger.parameter = 0;
      Trigger.source = TriggerIn;

      // set trigger
      error = pCam->SetTriggerMode( &Trigger );
      _CheckError(error);
    }
    /* */
    m_vCams.push_back(pCam);
  }

  //    const FlyCapture2::Camera** tmp = m_vCams.const_pointer;
  //    FlyCapture2::Camera::StartSyncCapture(m_vCams.size(), tmp);

  // initiate transmission on all cameras
  for(unsigned int ii = 0; ii < m_vCams.size(); ++ii) {
    error = m_vCams[ii]->StartCapture();
    //        _CheckError(error);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
FlycapDriver::~FlycapDriver()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool FlycapDriver::Capture(hal::CameraMsg& vImages)
{
  FlyCapture2::Error error;

  for(unsigned int ii = 0; ii < m_vCams.size(); ++ii) {
    FlyCapture2::Image Image;

    error = m_vCams[ii]->RetrieveBuffer( &Image );

    if( error != FlyCapture2::PGRERROR_OK ) {
      std::cerr << "HAL: Error grabbing image from camera." << std::endl;
      std::cerr << "Error was: " << error.GetDescription() << std::endl;
      return false;
    } else {

      // set timestamp only from first camera
      if( ii == 0 ) {
        vImages.set_device_time( Image.GetMetadata().embeddedTimeStamp );
      }

      hal::ImageMsg* pbImg = vImages.add_image();
      pbImg->set_width( Image.GetCols() );
      pbImg->set_height( Image.GetRows() );
      pbImg->set_data( Image.GetData(), Image.GetDataSize() );
      pbImg->set_type( hal::PB_UNSIGNED_BYTE );
      pbImg->set_format( hal::PB_LUMINANCE );
    }
  }
  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string FlycapDriver::GetDeviceProperty(const std::string& /*sProperty*/)
{
  return std::string();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
size_t FlycapDriver::NumChannels() const
{
  return m_vCams.size();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
size_t FlycapDriver::Width( size_t /*idx*/ ) const
{
  return m_nImgWidth;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
size_t FlycapDriver::Height( size_t /*idx*/ ) const
{
  return m_nImgHeight;
}
