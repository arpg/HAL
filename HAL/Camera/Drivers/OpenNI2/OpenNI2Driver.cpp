#include "OpenNI2Driver.h"

#include <glog/logging.h>
#include <HAL/Devices/DeviceException.h>
#include <HAL/Utils/TicToc.h>

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
        const std::string&      dev_sn,
        const std::string&      scmod
    ) 
{
  m_dev_sn = dev_sn;
  m_bSoftwareAlign = bAlignDepth;
  m_sCameraModel = scmod;

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


  if (rc != openni::STATUS_OK) {
    printf("OpenNI2Driver: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
    openni::OpenNI::shutdown();
  }
  
  //printf("OpenNI2Driver: Opened device [%s]\n",  m_device.getUri());
  
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
    m_streams.push_back( &m_colorStream );
  }
  if( bCaptureIR ){
    m_streams.push_back( &m_irStream );
  } 

  m_width = m_depthVideoMode.getResolutionX();
  m_height = m_depthVideoMode.getResolutionY();

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


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool OpenNI2Driver::Capture( hal::CameraMsg& vImages )
{
  int changedIndex;
  openni::Status rc;
  static double d0;

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
      m_irStream.readFrame(&m_colorFrame);
    }
  }

  if( m_streams[changedIndex] == &m_colorStream ){
    m_colorStream.readFrame(&m_colorFrame);
    if( m_depthStream.isValid() ){
      m_depthStream.readFrame(&m_depthFrame);
    } 
    if( m_irStream.isValid() ){ 
      m_irStream.readFrame(&m_colorFrame);
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
  d0 = hal::Tic();

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
    im->set_type( hal::PB_UNSIGNED_BYTE );
    im->set_format( hal::PB_LUMINANCE );
    im->set_data( m_irFrame.getData(), m_irFrame.getDataSize() );
  }

  SoftwareAlign(vImages);
  return true;
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


static cv::Vec3b getColorSubpix(const cv::Mat &img, cv::Point2f pt) 
{
  cv::Mat patch;
  cv::getRectSubPix(img, cv::Size(1, 1), pt, patch);
  return patch.at<cv::Vec3b>(0, 0);
}

// color the depth map
void OpenNI2Driver::SoftwareAlign(hal::CameraMsg& vImages) 
{
  if(m_bSoftwareAlign)
  {
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
    }

    cv::imshow("rgb", OutRGB8UC3);
    cv::waitKey(1);


    // now, change the data in the Pb messages.
    hal::ImageMsg* pbImgRGB  = vImages.mutable_image(0);
    pbImgRGB->set_data(OutRGB8UC3.data, sizeof(OutRGB8UC3.data));

    hal::ImageMsg* pbImgDepth  = vImages.mutable_image(1);
    pbImgDepth->set_data(OutDepth16U.data, sizeof(OutDepth16U.data));
  }
}


