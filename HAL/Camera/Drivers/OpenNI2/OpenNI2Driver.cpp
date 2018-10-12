#include "OpenNI2Driver.h"

#include <glog/logging.h>
#include <HAL/Devices/DeviceException.h>
#include <HAL/Utils/TicToc.h>
#include <HAL/Utils/StringUtils.h>

#include "OniEnums.h"

#include <iostream>

using namespace hal;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
OpenNI2Driver::OpenNI2Driver(
        unsigned int            nWidth,
        unsigned int            nHeight,
        unsigned int            nFPS,
        bool                    bCaptureRGB,
        bool                    bCaptureDepth,
        bool                    bCaptureIR,
        bool                    bAlignDepth,
        unsigned int            nExposure,
        unsigned int            nGain,
        const std::string&      dev_sn,
        const std::string&      scmod
    )

{
  m_dev_sn = dev_sn;
  m_bHardwareAlign = bAlignDepth;
  m_exposure = nExposure;
  m_gain = nGain;
  m_sCameraModel = scmod;
  m_exposureUpdated = true;
  m_frame = 0;

  if (m_bHardwareAlign && (m_sCameraModel.size() > 0))
    {
      LOG(FATAL) << "OpenNI2Driver: Pick either hardware alignment (hw_align=1) or supply a camera model via cmod=\"filename.xml\"";
    }

  openni::Status rc = openni::STATUS_OK;
  rc = openni::OpenNI::initialize();
  if (rc != openni::STATUS_OK) {
    LOG(FATAL) << "OpenNI2Driver: After initialization:\n"
      << openni::OpenNI::getExtendedError();
  }


  int chosenDevice = -1;
  if (dev_sn.size() > 0)
    {
      printf("OpenNI2Driver: Looking for user-supplied device S/N: %s\n", dev_sn.c_str() );
    }

  openni::Array<openni::DeviceInfo> deviceList;
  openni::OpenNI::enumerateDevices(&deviceList);
  printf("OpenNI2Driver: Found %d devices:\n", deviceList.getSize() );
  for( int ii = 0; ii < deviceList.getSize(); ii++ ){
    std::string oneUri = deviceList[ii].getUri();
    std::string serNum = getSerial(oneUri);
    printf("  Device[%d] URI: %s Serial: %s\n", ii, oneUri.c_str(), serNum.c_str());
    if (serNum == dev_sn)
      {
  chosenDevice = ii;
      }
  }

  //If a user s/n was provided and found, open it. Otherwise, open the first device found
  if ((dev_sn.size() > 0) && (chosenDevice >= 0))
    {
      rc = m_device.open(deviceList[chosenDevice].getUri() );
    }
  else
    {
      rc = m_device.open(openni::ANY_DEVICE); //pick anything we can
    }


  printf("OpenNI2: RGB? %d, Depth? %d, IR? %d\n", bCaptureRGB, bCaptureDepth, bCaptureIR);

  if (rc != openni::STATUS_OK) {
    printf("OpenNI2Driver: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
    openni::OpenNI::shutdown();
  }

  //printf("OpenNI2Driver: Opened device [%s]\n",  m_device.getUri());

  //Turn off mirroring so that the imagery is consistent with other HAL drivers



  // setup depth channel
  if( bCaptureDepth ){
    rc = m_depthStream.create(m_device, openni::SENSOR_DEPTH);
    if (rc != openni::STATUS_OK)	{
      printf("OpenNI2Driver: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
    }

    m_depthVideoMode = m_depthStream.getVideoMode();
    m_depthVideoMode.setResolution( nWidth, nHeight );
    m_depthVideoMode.setFps(nFPS);
    m_depthVideoMode.setPixelFormat( openni::PIXEL_FORMAT_DEPTH_100_UM );
    m_depthStream.setVideoMode( m_depthVideoMode );

    rc = m_depthStream.start();
    if (rc != openni::STATUS_OK) {
      printf("OpenNI2Driver: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
      m_depthStream.destroy();
    }
    m_depthStream.setMirroringEnabled(false);
    m_streams.push_back( &m_depthStream );
  }

  if( bCaptureRGB ){
    rc = m_colorStream.create(m_device, openni::SENSOR_COLOR);
    if (rc != openni::STATUS_OK) {
      printf("OpenNI2Driver: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
    }

    m_colorVideoMode = m_colorStream.getVideoMode();
    m_colorVideoMode.setResolution( nWidth, nHeight );
    m_colorVideoMode.setFps( nFPS );
    m_colorVideoMode.setPixelFormat( openni::PIXEL_FORMAT_RGB888 );
    m_colorStream.setVideoMode( m_colorVideoMode );

    rc = m_colorStream.start();

    if (rc != openni::STATUS_OK) {
      printf("OpenNI2Driver: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
      m_colorStream.destroy();
    }

    m_colorStream.setMirroringEnabled(false);
    m_streams.push_back( &m_colorStream );
  }

  if (bCaptureRGB && bCaptureIR)
    {
       printf("OpenNI2Driver: Can't capture from both RGB and IR at the same time\n");
    }

  if( bCaptureIR && (!bCaptureRGB)){
    rc = m_irStream.create(m_device, openni::SENSOR_IR);
    if (rc != openni::STATUS_OK) {
      printf("OpenNI2Driver: Couldn't find ir stream:\n%s\n", openni::OpenNI::getExtendedError());
    }

    m_irVideoMode = m_irStream.getVideoMode();
    m_irVideoMode.setResolution( nWidth, nHeight );
    m_irVideoMode.setFps( nFPS );
    m_irVideoMode.setPixelFormat( openni::PIXEL_FORMAT_GRAY16);
    m_irStream.setVideoMode( m_irVideoMode );

    rc = m_irStream.start();
    if (rc != openni::STATUS_OK) {
      printf("OpenNI2Driver: Couldn't start ir stream:\n%s\n", openni::OpenNI::getExtendedError());
      m_irStream.destroy();
    }
    m_irStream.setMirroringEnabled(false);
    m_streams.push_back( &m_irStream );
  }

  //Set hardware registration appropriately
  setHardwareRegistrationMode(m_bHardwareAlign);


  //Setup software alignment and rectification if needed
  if(m_sCameraModel.size() > 0)
    {
      std::string modelFileName = hal::ExpandTildePath(m_sCameraModel);
      if (!hal::FileExists(modelFileName))
  {
    LOG(FATAL) << "OpenNI2Driver: Could not find camera model file: " << modelFileName;
  }

      m_pRig = calibu::ReadXmlRig( modelFileName);
      Eigen::VectorXd RGBParams = m_pRig->cameras_[0]->GetParams();
      m_rRGBImgIn.init(RGBParams(0), RGBParams(1), RGBParams(2), RGBParams(3));

      Eigen::VectorXd DepthParams = m_pRig->cameras_[1]->GetParams();
      m_rDepthImgIn.init(DepthParams(0), DepthParams(1), DepthParams(2), DepthParams(3));
      printf("OpenNI2Driver: Software alignment enabled using file: %s\n", m_sCameraModel.c_str() );

      // Set up rectification
      /*
       // Convert rig to vision frame.
      std::shared_ptr<calibu::Rig<double>> new_rig =
  calibu::ToCoordinateConvention(m_pRig, calibu::RdfVision);

      // Generate lookup tables for stereo rectify.
      m_vLuts.resize(new_rig->NumCams());
      for(size_t i=0; i< new_rig->NumCams(); ++i) {
  m_vLuts[i] = calibu::LookupTable(new_rig->cameras_[i]->Width(), new_rig->cameras_[i]->Height());
      }


      if(new_rig->NumCams() == 2) {
  m_pRig = calibu::CreateScanlineRectifiedLookupAndCameras(
                new_rig->cameras_[1]->Pose().inverse()*new_rig->cameras_[0]->Pose(),
                new_rig->cameras_[0], new_rig->cameras_[1],
                m_T_nr_nl,
                m_vLuts[0], m_vLuts[1]
                );


      }
   */
    }


  m_width = nWidth;
  m_height = nHeight;

  LOG(INFO) << "OpenNI2Driver: opened device at "
    << m_width << "x" << m_height << " resolution\n";
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
OpenNI2Driver::~OpenNI2Driver()
{
  if( m_depthStream.isValid() ){
    m_depthStream.stop();
    m_depthStream.destroy();
  }
  if( m_colorStream.isValid() ){
    m_colorStream.stop();
    m_colorStream.destroy();
  }
  if( m_irStream.isValid() ){
    m_irStream.stop();
    m_irStream.destroy();
  }
  m_device.close();

  openni::OpenNI::shutdown();
}

//From the ROS openni2_camera package, src/openni2_device_manager.cpp

std::string OpenNI2Driver::getSerial(const std::string& Uri) const
{
  //OpenNI2 doesn't report serial numbers until the device is actually opened
  {
  openni::Device openni_device;
  std::string ret;

  // we need to open the device to query the serial number
  if (Uri.length() > 0 && openni_device.open(Uri.c_str()) == openni::STATUS_OK)
  {
    int serial_len = 100;
    char serial[serial_len];

    openni::Status rc = openni_device.getProperty(openni::DEVICE_PROPERTY_SERIAL_NUMBER, serial, &serial_len);
    if (rc == openni::STATUS_OK)
      ret = serial;
    else
    {
      printf("OpenNI2: Serial number query failed: %s", openni::OpenNI::getExtendedError());
    }
    // close the device again
    openni_device.close();
  }
  else
  {
    printf("OpenNI2: Device open failed: %s", openni::OpenNI::getExtendedError());
  }
  return ret;
  }
}

//From the ROS openni2_camera node, src/openni2_device.cpp
void OpenNI2Driver::setHardwareRegistrationMode(bool enable)
{
  if (m_device.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
  {
    if (enable == true)
    {
      openni::Status rc = m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
      if (rc != openni::STATUS_OK)
  {
    printf("OpenNI2Driver: Enabling image registration mode failed: \n%s\n", openni::OpenNI::getExtendedError());

  }
    }
    else
    {
      openni::Status rc = m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
      if (rc != openni::STATUS_OK)
  printf("OpenNI2Driver: Disabling image registration mode failed: \n%s\n", openni::OpenNI::getExtendedError());
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool OpenNI2Driver::Capture( hal::CameraMsg& vImages )
{
  int changedIndex;
  openni::Status rc;
  static double d0;

  // check if exposure update needed
  if (m_colorStream.isValid()) //  && m_exposureUpdated)
  {
    // check if using manual exposure
    if ( m_exposure > 0) {

      int gain = m_gain;
      int exposure = m_exposure;

      if (m_frame < 3)
      {
        gain += (gain > 0.5 * MaxGain(0)) ? -1 : 0;
        exposure += (exposure > 0.5 * MaxExposure(0)) ? -1 : 0;

        gain += (m_frame % 2);
        exposure += (m_frame % 2);

        ++m_frame;
      }

      m_colorStream.getCameraSettings()->setGain( gain );
      m_colorStream.getCameraSettings()->setExposure( exposure );
      m_colorStream.getCameraSettings()->setAutoExposureEnabled( false );
      m_colorStream.getCameraSettings()->setAutoWhiteBalanceEnabled( false );
    }
    // otherwise use auto-exposure
    else
    {
      m_colorStream.getCameraSettings()->setAutoExposureEnabled( true );
      m_colorStream.getCameraSettings()->setAutoWhiteBalanceEnabled( true );
    }

    m_exposureUpdated = false;
  }

  rc = openni::OpenNI::waitForAnyStream( &m_streams[0], m_streams.size(), &changedIndex );
  if (rc != openni::STATUS_OK)	{
    printf("Wait failed\n");
    return false;
  }

  if( m_streams[changedIndex] == &m_depthStream ){
    m_depthStream.readFrame(&m_depthFrame);
    if(  m_colorStream.isValid() ){
      m_colorStream.readFrame(&m_colorFrame);
    }
    if( m_irStream.isValid() ){
      m_irStream.readFrame(&m_irFrame);
    }
  }

  if( m_streams[changedIndex] == &m_colorStream ){
    m_colorStream.readFrame(&m_colorFrame);
    if( m_depthStream.isValid() ){
      m_depthStream.readFrame(&m_depthFrame);
    }
    if( m_irStream.isValid() ){
      m_irStream.readFrame(&m_irFrame);
    }
  }

  if( m_streams[changedIndex] == &m_irStream ){
    if( m_irStream.isValid() ){
      m_irStream.readFrame(&m_irFrame);
    }
  }

  // if the channel is on, make sure we get the image
  if( m_colorStream.isValid() ){
    if( !m_colorFrame.isValid() ){
      return false;
    }
  }
  if( m_depthStream.isValid() ){
    if( !m_depthFrame.isValid() ){
      return false;
    }
  }
  if( m_irStream.isValid() ){
    if( !m_irFrame.isValid() ){
      return false;
    }
  }

  //LOG(INFO) << m_dev_sn << " capturing at\t" <<  1000.0/hal::TocMS(d0) << "hz\n";

  if( m_colorStream.isValid() ){
    hal::ImageMsg* im = vImages.add_image();
    im->set_timestamp( m_colorFrame.getTimestamp() );
    im->set_width( m_width );
    im->set_height( m_height );
    im->set_type( hal::PB_UNSIGNED_BYTE );
    im->set_format( hal::PB_RGB );
    im->set_data( m_colorFrame.getData(), m_colorFrame.getDataSize() );
  }

  if( m_depthStream.isValid() ){
    hal::ImageMsg* im = vImages.add_image();
    im->set_timestamp( m_depthFrame.getTimestamp() );
    im->set_width( m_width );
    im->set_height( m_height );
    im->set_type( hal::PB_UNSIGNED_SHORT );
    im->set_format( hal::PB_LUMINANCE );
    im->set_data( m_depthFrame.getData(), m_depthFrame.getDataSize() );
  }

  if( m_irStream.isValid() ){
    hal::ImageMsg* im = vImages.add_image();
    im->set_timestamp( m_irFrame.getTimestamp() );
    im->set_width( m_width );
    im->set_height( m_height );
    im->set_type( hal::PB_UNSIGNED_SHORT);
    im->set_format( hal::PB_LUMINANCE );
    uint16_t *scaled = AutoScale(m_irFrame.getData(), m_irFrame.getHeight()*m_irFrame.getWidth());
    im->set_data( scaled, 2*m_irFrame.getHeight()*m_irFrame.getWidth() );
    delete[] scaled;
  }

  if (m_sCameraModel.size() > 0)
    SoftwareAlign(vImages);

  return true;
}

uint16_t* OpenNI2Driver::AutoScale(const void* src, uint32_t pixelCount)
{
  //Given a gray source image with a specified bpp, make a new image that has the dynamic range expanded to gray16
  uint16_t* src_data = (uint16_t*) src;

  //Make a gray16 dest image
  uint16_t *dest = new uint16_t[pixelCount];

  //For this frame, find the max value:
  uint16_t maxVal = 0;
  for (unsigned int ii = 0; ii < pixelCount; ii++)
    {
      if (src_data[ii] > maxVal)
  maxVal = src_data[ii];
    }

  //Scale by the max value
  float scaleFactor = (1 << 16) / maxVal;
  for (unsigned int ii = 0; ii < pixelCount; ii++)
    {
      dest[ii] = src_data[ii]*scaleFactor;
    }

  //Caller releases storage
  return dest;
}

std::string OpenNI2Driver::GetDeviceProperty(const std::string& sProperty)
{
  if(sProperty == hal::DeviceDepthBaseline) {
    return std::to_string( m_DepthBaseline );
  }
  if(sProperty == hal::DeviceDepthFocalLength) {
    return std::to_string( m_DepthFocalLength );
  }
  return std::string();
}

size_t OpenNI2Driver::NumChannels() const
{
  return m_streams.size();
}

// all channels must have same resolution
size_t OpenNI2Driver::Width( size_t /*idx*/ ) const
{
  return m_width;
}

// all channels must have same resolution
size_t OpenNI2Driver::Height( size_t /*idx*/ ) const
{
  return m_height;
}

bool OpenNI2Driver::AutoExposure() const
{
  return m_exposure == 0;
}

unsigned int OpenNI2Driver::Exposure() const
{
  return m_exposure;
}

void OpenNI2Driver::SetExposure(unsigned int exposure)
{
  if (m_exposure != exposure)
  {
    m_exposure = exposure;
    m_exposureUpdated = true;
  }
}

unsigned int OpenNI2Driver::Gain() const
{
  return m_gain;
}

void OpenNI2Driver::SetGain(unsigned int gain)
{
  if (m_gain != gain)
  {
    m_gain = gain;
    m_exposureUpdated = true;
  }
}

///////////////////////////////////////////////////////////////////////////
double OpenNI2Driver::MaxExposure(int) const
{
  return 1000;
}

///////////////////////////////////////////////////////////////////////////
double OpenNI2Driver::MinExposure(int) const
{
  return 1;
}

///////////////////////////////////////////////////////////////////////////
double OpenNI2Driver::MaxGain(int) const
{
  return 1000;
}

///////////////////////////////////////////////////////////////////////////
double OpenNI2Driver::MinGain(int) const
{
  return 100;
}

///////////////////////////////////////////////////////////////////////////
double OpenNI2Driver::Exposure(int)
{
  return m_colorStream.getCameraSettings()->getExposure();
}

///////////////////////////////////////////////////////////////////////////
void OpenNI2Driver::SetExposure(double exposure, int)
{
  m_colorStream.getCameraSettings()->setExposure(exposure);
  m_colorStream.getCameraSettings()->setAutoExposureEnabled(false);
  m_colorStream.getCameraSettings()->setAutoWhiteBalanceEnabled(false);
}

///////////////////////////////////////////////////////////////////////////
double OpenNI2Driver::Gain(int)
{
  return m_colorStream.getCameraSettings()->getGain();
}

///////////////////////////////////////////////////////////////////////////
void OpenNI2Driver::SetGain(double gain, int)
{
  m_colorStream.getCameraSettings()->setGain(gain);
  m_colorStream.getCameraSettings()->setAutoExposureEnabled(false);
  m_colorStream.getCameraSettings()->setAutoWhiteBalanceEnabled(false);
}

///////////////////////////////////////////////////////////////////////////
double OpenNI2Driver::ProportionalGain(int) const
{
  return 0.075;
}

///////////////////////////////////////////////////////////////////////////
double OpenNI2Driver::IntegralGain(int) const
{
  return 0.001;
}

///////////////////////////////////////////////////////////////////////////
double OpenNI2Driver::DerivativeGain(int) const
{
  return 0.05;
}

///////////////////////////////////////////////////////////////////////////
static cv::Vec3b getColorSubpix(const cv::Mat &img, cv::Point2f pt)
{
  cv::Mat patch;
  cv::getRectSubPix(img, cv::Size(1, 1), pt, patch);
  return patch.at<cv::Vec3b>(0, 0);
}

// color the depth map
void OpenNI2Driver::SoftwareAlign(hal::CameraMsg& vImages)
{

  /*
  //First, rectify the image using Calibu
  hal::Image raw_img[2] = { hal::Image(vImages.image(0)),
          hal::Image(vImages.image(1)) };

  for (unsigned int k=0; k<2; k++)
    {
      calibu::Rectify(m_vLuts[k], raw_img[k].data(),
          reinterpret_cast<unsigned char*>(&pimg->mutable_data()->front()),
          img.Width(), img.Height(), num_channels);
    }
  */

    hal::Image rawRGBImg = hal::Image(vImages.image(0));
    cv::Mat &rRGB8UC3 = rawRGBImg.Mat();

    hal::Image rawDepthImg = hal::Image(vImages.image(1));
    cv::Mat &rDepth16U = rawDepthImg.Mat();

    // get the camera intrinsics
    Sophus::SE3d T_RGB_Depth = m_pRig->cameras_[1]->Pose();

    std::cout<<"T_RGB_Depth:"<<_T2Cart(T_RGB_Depth.matrix()).transpose()<<std::endl;

    // -------------
    cv::Mat OutDepth16U =
      cv::Mat::zeros(rDepth16U.rows, rDepth16U.cols, CV_16U);
    cv::Mat OutRGB8UC3 =
      cv::Mat::zeros(rDepth16U.rows, rDepth16U.cols, CV_8UC3);

    // register depth image to rgb image
    for (float i = 0; i != rDepth16U.rows; i++) {
      for (float j = 0; j != rDepth16U.cols; j++) {
        // for a depth val
        float fDepth = static_cast<float>(rDepth16U.at<ushort>(i, j)) / 1000.f;

        if (fDepth > 0  ) {
          // get x, y, z of the voxel as P_w
          Eigen::Vector3d P_w_Depth = m_rDepthImgIn.Unproject(i, j, fDepth);
          Sophus::SE3d sP_w_Depth(_Cart2T(P_w_Depth(0), P_w_Depth(1), P_w_Depth(2),0,0,0));

          // get the new pose of p_w in 3D w.r.t the rgb camera.
          Sophus::SE3d sP_w_RGB = T_RGB_Depth * sP_w_Depth;

          const Eigen::Vector3d P_w_RGB = sP_w_RGB.translation();

          // project the 3D point it back to the rgb camera frame
          Eigen::Vector2d p_i = m_rRGBImgIn.Project(P_w_RGB);

          // see if the pixel is in range of the rgb camera
          if (cvRound(p_i(0)) >= 0 && cvRound(p_i(0)) < rRGB8UC3.rows && // 480
              cvRound(p_i(1)) >= 0 && cvRound(p_i(1)) < rRGB8UC3.cols)   // 640
          {
            // do interpations here to color the depth image..
            OutRGB8UC3.at<cv::Vec3b>(i, j) =
              getColorSubpix(rRGB8UC3, cv::Point2f(p_i(1), p_i(0)));
            OutDepth16U.at<ushort>(i, j) = rDepth16U.at<ushort>(i, j);
          }
        }
      }

      //cv::imshow("rgb", OutRGB8UC3);
    //cv::waitKey(1);


    // now, change the data in the Pb messages.
    hal::ImageMsg* pbImgRGB  = vImages.mutable_image(0);
    pbImgRGB->set_data(OutRGB8UC3.data, sizeof(OutRGB8UC3.data));

    hal::ImageMsg* pbImgDepth  = vImages.mutable_image(1);
    pbImgDepth->set_data(OutDepth16U.data, sizeof(OutDepth16U.data));
  }
}


