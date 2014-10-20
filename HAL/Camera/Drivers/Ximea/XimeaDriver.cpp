#include <iostream>
#include "XimeaDriver.h"

using namespace hal;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void XimeaDriver::_CheckError( XI_RETURN err, std::string place)
{
  if( err != XI_OK ) {
    std::cout << "Error after" << place << " " << err << std::endl;
    throw DeviceException("Ximea SDK exception!");
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
XimeaDriver::XimeaDriver(std::vector<unsigned int>& vID,
    ImageRoi ROI
    )
{

  // initialize
  //const int max_width = 1600;
  //const int max_height = 1200;

  m_nImgWidth  = ROI.w;
  m_nImgHeight = ROI.h;

  XI_RETURN error = XI_OK;

  // Get number of camera devices
  error = xiGetNumberDevices(&dwNumberOfDevices);
  _CheckError(error,"xiGetNumberDevices (no camera found)");

  if (dwNumberOfDevices == 0) {
    throw DeviceException("No cameras found!");
  }

  if (dwNumberOfDevices < vID.size()) {
    throw DeviceException("Less cameras detected than those requested!");
  }

  unsigned int nNumCams;
  // If no ids are ovided, all cameras will be opened.
  if (vID.empty()) {
    nNumCams = dwNumberOfDevices;
  } else {
    nNumCams = vID.size();
  }

  // Retrieving a handle to the camera device
  error = xiOpenDevice(0, &xiH);
  _CheckError(error,"xiOpenDevice");

  // Setting "exposure" parameter (10ms=10000us)
  error = xiSetParamInt(xiH, XI_PRM_EXPOSURE, 10000);
  _CheckError(error,"xiSetParam (exposure set)");

  error = xiStartAcquisition(xiH);
  _CheckError(error,"xiStartAcquisition");

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
XimeaDriver::~XimeaDriver()
{
    // Close device
    if (xiH)
        xiCloseDevice(xiH);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool XimeaDriver::Capture(pb::CameraMsg& vImages)
{
  XI_RETURN error = XI_OK;

  // image buffer
  XI_IMG image;
  memset(&image,0,sizeof(image));
  image.size = sizeof(XI_IMG);

  error = xiGetImage(xiH, 5000, &image);
  _CheckError(error,"xiGetImage");

  // set timestamp only from first camera
  vImages.set_device_time( image.tsSec );

  pb::ImageMsg* pbImg = vImages.add_image();
  pbImg->set_width( image.width );
  pbImg->set_height( image.height );
  pbImg->set_data( image.bp, image.bp_size );
  pbImg->set_type( pb::PB_UNSIGNED_BYTE );
  pbImg->set_format( pb::PB_LUMINANCE );

  if(error == XI_OK)
    return true;
  else
    return false;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string XimeaDriver::GetDeviceProperty(const std::string& /*sProperty*/)
{
  return std::string();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
size_t XimeaDriver::NumChannels() const
{
  return dwNumberOfDevices;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
size_t XimeaDriver::Width( size_t /*idx*/ ) const
{
  return m_nImgWidth;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
size_t XimeaDriver::Height( size_t /*idx*/ ) const
{
  return m_nImgHeight;
}
