/*
 * Format7 code extracted from Pangolin library.
 *
 */


#include <bitset>
#include <iostream>

#include <dc1394/conversions.h>

#include <HAL/Utils/TicToc.h>

#include "DC1394Driver.h"


using namespace hal;


/////////////////////////////////////////////////////////////////////////////
void DC1394Driver::_SetImageMetaDataFromCamera( hal::ImageMsg* img, dc1394camera_t* pCam )
{
  hal::ImageInfoMsg* info = img->mutable_info();

  // obtain meta data from image
  dc1394error_t e;
  float feature;
  e = dc1394_feature_get_absolute_value( pCam, DC1394_FEATURE_SHUTTER, &feature );
  if( e == DC1394_SUCCESS ) {
    info->set_shutter( feature );
  }

  e = dc1394_feature_get_absolute_value( pCam, DC1394_FEATURE_GAIN, &feature );
  if( e == DC1394_SUCCESS ) {
    info->set_gain( feature );
  }

  //    e = dc1394_feature_get_absolute_value( pCam, DC1394_FEATURE_GAMMA, &feature );
  if( e == DC1394_SUCCESS ) {
    info->set_gamma( feature );
  }
}



/////////////////////////////////////////////////////////////////////////////
inline int DC1394Driver::_NearestValue(int value, int step, int min, int max)
{
  int low, high;

  low=value-(value%step);
  high=value-(value%step)+step;
  if (low<min)
    low=min;
  if (high>max)
    high=max;

  if (abs(low-value)<abs(high-value))
    return low;
  else
    return high;
}


/////////////////////////////////////////////////////////////////////////////
DC1394Driver::DC1394Driver( const Uri& uri )
{
  CameraDriverInterface::SetDefaultProperties({
      {"idN","0","Camera id number."},
      {"mode","MONO8","Video mode: RGB8, MONO8, MONO16, FORMAT7_X"},
      {"size", "640x480", "Capture resolution."},
      {"roi", "0+0+640x480", "ROI resolution for Format7."},
      {"fps", "30.0", "Capture framerate."},
      {"iso", "800", "ISO speed."},
      {"dma", "4", "Number of DMA channels."},
      {"exp", "AUTO", "Sets exposure to absolute value."},
      {"ptgrey_timestamp", "false", "use point grey device timestamp."}
      }
  );
  if( !CameraDriverInterface::ParseUriProperties( uri.properties ) ){
    std::cerr << "DC1394Driver knows about the following properties:\n";
    CameraDriverInterface::PrintPropertyMap();
    return;
  }

  ImageDim Dims         = GetProperty<ImageDim>("size", ImageDim(640,480));
  ImageRoi ROI          = GetProperty<ImageRoi>("roi", ImageRoi(0,0,0,0));
  float fFPS            = GetProperty<float>("fps", 30);
  float fEXP            = GetProperty<float>("exp", FLT_MAX);
  unsigned int nDMA     = GetProperty<unsigned int>("dma", 4);
  bool ptgrey_timestamp = GetProperty<bool>("ptgrey_timestamp", false);

  // Get ROI.
  if( ROI.w == 0 && ROI.h == 0 ) {
    ROI.w = Dims.x;
    ROI.h = Dims.y;
  }
  unsigned int nTop = ROI.x;
  unsigned int nLeft = ROI.y;
  unsigned int nWidth = ROI.w;
  unsigned int nHeight = ROI.h;

  // Get IDs.
  std::vector<unsigned int> vCamId;
  while(true) {
    std::stringstream ss;
    ss << "id" << vCamId.size();
    const std::string key = ss.str();
    if(!uri.properties.Contains(key)) {
      break;
    }
    vCamId.push_back( GetProperty<unsigned int>( key, 0 ) );
  }

  // Get Mode.
  std::string sMode = GetProperty<std::string>("mode", "MONO8");
  dc1394video_mode_t Mode;
  if( sMode.find( "FORMAT7" ) != std::string::npos ) {
    if( sMode == "FORMAT7_0" ) {
      Mode = DC1394_VIDEO_MODE_FORMAT7_0;
    } else if( sMode == "FORMAT7_1" ) {
      Mode = DC1394_VIDEO_MODE_FORMAT7_1;
    } else if( sMode == "FORMAT7_2" ) {
      Mode = DC1394_VIDEO_MODE_FORMAT7_2;
    } else if( sMode == "FORMAT7_3" ) {
      Mode = DC1394_VIDEO_MODE_FORMAT7_3;
    } else if( sMode == "FORMAT7_4" ) {
      Mode = DC1394_VIDEO_MODE_FORMAT7_4;
    } else if( sMode == "FORMAT7_5" ) {
      Mode = DC1394_VIDEO_MODE_FORMAT7_5;
    } else if( sMode == "FORMAT7_6" ) {
      Mode = DC1394_VIDEO_MODE_FORMAT7_6;
    } else {
      Mode = DC1394_VIDEO_MODE_FORMAT7_7;
    }
  } else {
    if( sMode == "MONO16" ) {
      if( ROI.w == 1024 ) {
        Mode = DC1394_VIDEO_MODE_1024x768_MONO16;
      } else if( ROI.w == 1280 ) {
        Mode = DC1394_VIDEO_MODE_1280x960_MONO16;
      } else if( ROI.w == 1600 ) {
        Mode = DC1394_VIDEO_MODE_1600x1200_MONO16;
      } else {
        Mode = DC1394_VIDEO_MODE_640x480_MONO16;
      }
    } else if( sMode == "RGB8" ) {
      if( ROI.w == 1024 ) {
        Mode = DC1394_VIDEO_MODE_1024x768_RGB8;
      } else if( ROI.w == 1280 ) {
        Mode = DC1394_VIDEO_MODE_1280x960_RGB8;
      } else if( ROI.w == 1600 ) {
        Mode = DC1394_VIDEO_MODE_1600x1200_RGB8;
      } else {
        Mode = DC1394_VIDEO_MODE_640x480_RGB8;
      }
    } else {
      // MONO8
      if( ROI.w == 1024 ) {
        Mode = DC1394_VIDEO_MODE_1024x768_MONO8;
      } else if( ROI.w == 1280 ) {
        Mode = DC1394_VIDEO_MODE_1280x960_MONO8;
      } else if( ROI.w == 1600 ) {
        Mode = DC1394_VIDEO_MODE_1600x1200_MONO8;
      } else {
        Mode = DC1394_VIDEO_MODE_640x480_MONO8;
      }
    }
  }

  // Get speed.
  unsigned int nISO = GetProperty<unsigned int>("iso", 400);
  dc1394speed_t Speed;
  if( nISO == 100 ) {
    Speed = DC1394_ISO_SPEED_100;
  } else if( nISO == 200 ) {
    Speed = DC1394_ISO_SPEED_200;
  } else if( nISO == 800 ) {
    Speed = DC1394_ISO_SPEED_800;
  } else {
    Speed = DC1394_ISO_SPEED_400;
  }

  m_PtGreyTimestamp = ptgrey_timestamp;
  m_DeviceTimestampOffset = 0;
  m_PreviousTimestamp = 0;
  dc1394error_t e;

  m_pBus = dc1394_new();

  dc1394camera_list_t*  pCameraList = NULL;
  e = dc1394_camera_enumerate( m_pBus, &pCameraList );

  if( pCameraList->num == 0 ) {
    std::cerr << "No DC1394 cameras found.\n";
    return;
  }

  // If no ids are provided, all cameras will be opened.
  if( vCamId.empty() ) {
    m_nNumChannels = pCameraList->num;
    for(size_t ii = 0; ii < m_nNumChannels; ++ii) {
      vCamId.push_back(ii);
    }
  } else {
    m_nNumChannels = vCamId.size();
  }

  // Resize camera vector to accomodate requested cameras.
  m_vCam.resize(m_nNumChannels);

  for(size_t ii = 0; ii < m_nNumChannels; ++ii) {
    m_vCam[ii] = dc1394_camera_new( m_pBus, pCameraList->ids[vCamId[ii]].guid );
  }

  // Free the camera list.
  dc1394_camera_free_list(pCameraList);


  for(size_t ii = 0; ii < m_nNumChannels; ++ii) {

    dc1394camera_t* pCam = m_vCam[ii];

    printf("Configuring camera with GUID %lu ...\n", (long unsigned int)pCam->guid);
    fflush(stdout);

    dc1394_camera_reset(pCam);

    //----- Set ISO speed.
    if( Speed >= DC1394_ISO_SPEED_800) {
      e = dc1394_video_set_operation_mode( pCam, DC1394_OPERATION_MODE_1394B );
      if( e != DC1394_SUCCESS ){
        std::cerr << "Could not set DC1394_OPERATION_MODE_1394B\n";
        return;
      }
    }

    e = dc1394_video_set_iso_speed(pCam,Speed);
    if( e != DC1394_SUCCESS ){
      std::cerr << "Could not set iso speed\n";
      return;
    }

    //----- set framerate and mode
    if( Mode < DC1394_VIDEO_MODE_FORMAT7_MIN ) {
      // "regular" mode

      e = dc1394_video_set_mode( pCam, Mode);
      std::cout << "Setting video mode to " << Mode << std::endl;
      if( e != DC1394_SUCCESS ){
        std::cerr << "Could not set video mode\n";
        return;
      }

      // get framerate
      dc1394framerate_t FPS;
      if( fFPS == 1.875 ) {
        FPS = DC1394_FRAMERATE_1_875;
      } else if( fFPS == 3.75 ) {
        FPS = DC1394_FRAMERATE_3_75;
      } else if( fFPS == 7.5 ) {
        FPS = DC1394_FRAMERATE_7_5;
      } else if( fFPS == 15 ) {
        FPS = DC1394_FRAMERATE_15;
      } else if( fFPS == 30 ) {
        FPS = DC1394_FRAMERATE_30;
      } else if( fFPS == 60 ) {
        FPS = DC1394_FRAMERATE_60;
      } else if( fFPS == 120 ) {
        FPS = DC1394_FRAMERATE_120;
      } else {
        FPS = DC1394_FRAMERATE_240;
      }

      e = dc1394_video_set_framerate(pCam, FPS);
      if( e != DC1394_SUCCESS ){
        std::cerr << "Could not set frame rate.\n";
        return;
      }

    } else {
      // format 7 mode

      dc1394format7mode_t Info;
      Info.present = DC1394_TRUE;

      // check that the required mode is actually supported
      e = dc1394_format7_get_mode_info(pCam, Mode, &Info);
      if( e != DC1394_SUCCESS ){
        std::cerr << "Could not get format7 mode info.\n";
        return;
      }

      // safely set the video mode
      e = dc1394_video_set_mode( pCam, Mode );
      if( e != DC1394_SUCCESS ){
        std::cerr << "Could not set format7 video mode.\n";
        return;
      }

      // set position to 0,0 so that setting any size within min and max is a valid command
      e = dc1394_format7_set_image_position(pCam, Mode, 0, 0);
      if( e != DC1394_SUCCESS ){
        std::cerr << "Could not set format7 image position.\n";
        return;
      }

      // work out the desired image size
      nWidth = _NearestValue(nWidth, Info.unit_pos_x, 0, Info.max_size_x - nLeft);
      nHeight = _NearestValue(nHeight, Info.unit_pos_y, 0, Info.max_size_y - nTop);

      // set size
      e = dc1394_format7_set_image_size(pCam, Mode, nWidth, nHeight);
      if( e != DC1394_SUCCESS ){
        std::cerr << "Could not set format7 size.\n";
        return;
      }

      // get the info again since many parameters depend on image size
      e = dc1394_format7_get_mode_info(pCam, Mode, &Info);
      if( e != DC1394_SUCCESS ){
        std::cerr << "Could not get format7 mode info.\n";
        return;
      }

      // work out position of roi
      nLeft = _NearestValue(nLeft, Info.unit_size_x, Info.unit_size_x, Info.max_size_x - nWidth);
      nTop = _NearestValue(nTop, Info.unit_size_y, Info.unit_size_y, Info.max_size_y - nHeight);

      // set roi position
      e = dc1394_format7_set_image_position(pCam, Mode, nLeft, nTop);
      if( e != DC1394_SUCCESS ){
        std::cerr << "Could not set format7 size.\n";
        return;
      }

      std::cout<<"   ROI: "<<nLeft<<" "<<nTop<<" "<<nWidth<<" "<<nHeight<<" ";



      e = dc1394_format7_set_packet_size(pCam, Mode, Info.max_packet_size);
      if( e != DC1394_SUCCESS ){
        std::cerr << "Could not set format7 packet size.\n";
        return;
      }

      if((fFPS != MAX_FR) && (fFPS != EXT_TRIG)){
        //set the framerate by using the absolute feature as suggested by the
        //folks at PointGrey
        e = dc1394_feature_set_absolute_control(pCam,DC1394_FEATURE_FRAME_RATE,DC1394_ON);
        if( e != DC1394_SUCCESS ){
          std::cerr << "Could not turn on absolute frame rate control.\n";
          return;
        }

        e = dc1394_feature_set_mode(pCam,DC1394_FEATURE_FRAME_RATE,DC1394_FEATURE_MODE_MANUAL);
        if( e != DC1394_SUCCESS ){
          std::cerr << "Could not make frame rate manual.\n";
          return;
        }

        e = dc1394_feature_set_absolute_value(pCam,DC1394_FEATURE_FRAME_RATE,fFPS);
        if( e != DC1394_SUCCESS ){
          std::cerr << "Could not set format7 framerate.\n";
          return;
        }
      }

      // ask the camera what is the resulting framerate (this assume that such a rate is actually
      // allowed by the shutter time)
      float value;
      e = dc1394_feature_get_absolute_value(pCam,DC1394_FEATURE_FRAME_RATE,&value);
      if( e != DC1394_SUCCESS ){
        std::cerr << "Could not get framerate.\n";
        return;
      } 

      std::cout << " - Framerate(shutter permitting): " << value << std::endl;
    }


    //----- set DMA channels    //----- set DMA channels

    e = dc1394_capture_setup(pCam, nDMA, DC1394_CAPTURE_FLAGS_DEFAULT);
    if( e != DC1394_SUCCESS ){
      std::cerr << "Could not setup camera - check settings.\n";
      return;
    }

    //----- set exposure
    if (fEXP != FLT_MAX) {
      dc1394bool_t ret_val1, ret_val2;
      e = dc1394_feature_is_present(pCam, DC1394_FEATURE_EXPOSURE, &ret_val1);
      e = dc1394_feature_has_absolute_control(pCam, DC1394_FEATURE_EXPOSURE, &ret_val2);

      if (ret_val1 == DC1394_TRUE && ret_val2 == DC1394_TRUE) {

        float vmin,vmax;
        e = dc1394_feature_get_absolute_boundaries(pCam, DC1394_FEATURE_EXPOSURE, &vmin, &vmax);

        if (fEXP < vmin || fEXP > vmax) {
          std::cout << " - Exposure value out of bounds [" << vmin << "," << vmax << "]." << std::endl;
        } else {
          dc1394switch_t power;
          e = dc1394_feature_get_power(pCam, DC1394_FEATURE_EXPOSURE, &power);
          if( e != DC1394_SUCCESS ){
            std::cerr << "Could not set exposure feature.\n";
            return;
          }

          dc1394feature_mode_t feature_mode = DC1394_FEATURE_MODE_MANUAL;
          e = dc1394_feature_set_mode(pCam, DC1394_FEATURE_EXPOSURE, feature_mode);
          if( e != DC1394_SUCCESS ){
            std::cerr << "Could not set exposure feature.\n";
            return;
          }

          e = dc1394_feature_set_absolute_control(pCam, DC1394_FEATURE_EXPOSURE, power);
          if( e != DC1394_SUCCESS ){
            std::cerr << "Could not set exposure feature.\n";
            return;
          }

          e = dc1394_feature_set_absolute_value(pCam, DC1394_FEATURE_EXPOSURE, fEXP);
          if( e != DC1394_SUCCESS ){
            std::cerr << "Could not set exposure value.\n";
            return;
          }
        }
      } else {
        std::cout << " - Absolute exposure not supported.\n";
      }
    }

    if (ptgrey_timestamp) {
      uint32_t frame_info_val = 0;
      e = dc1394_get_control_register(pCam, 0x12F8, &frame_info_val);
      if( e != DC1394_SUCCESS ){
        std::cerr << "Could not get FRAME_INFO value.\n";
        return;
      }


      frame_info_val |= (1 << 31) | 1;
      e = dc1394_set_control_register(pCam, 0x12F8, frame_info_val);
      if( e != DC1394_SUCCESS ){
        std::cerr << "Could not set FRAME_INFO value.\n";
        return;
      }

      frame_info_val = 0;
      e = dc1394_get_control_register(pCam, 0x12F8, &frame_info_val);
      if( e != DC1394_SUCCESS ){
        std::cerr << "Could not get FRAME_INFO value.\n";
        return;
      }

      std::bitset<32> frame_info_bits(frame_info_val);
      std::cerr << "FRAME_INFO: " << frame_info_bits << std::endl;
    }

    printf("OK.\n");
  }


  // Initiate transmission on all cameras.
  for(size_t ii = 0; ii < m_nNumChannels; ++ii) {
    e = dc1394_video_set_transmission( m_vCam[ii], DC1394_ON );
    if( e != DC1394_SUCCESS ){
      std::cerr << "Could not start camera iso transmission.\n";
      return;
    }
  }

  // Capture one frame to get image dimensions.
  // Note: If you are getting no captures, check that the ISO speed is OK!
  dc1394video_frame_t* pFrame;
  e = dc1394_capture_dequeue( m_vCam[0], DC1394_CAPTURE_POLICY_WAIT, &pFrame );
  if( e != DC1394_SUCCESS ){
    std::cerr << "Could not capture a frame.\n";
    return;
  }


  // Image information. This is RAW.
  m_nImageWidth = pFrame->size[0];
  m_nImageHeight = pFrame->size[1];

  if( pFrame->color_coding == DC1394_COLOR_CODING_MONO8  ) {
    m_VideoFormat = hal::PB_LUMINANCE;
  } else if( pFrame->color_coding == DC1394_COLOR_CODING_MONO16  ) {
    m_VideoFormat = hal::PB_LUMINANCE;
  } else if( pFrame->color_coding == DC1394_COLOR_CODING_RGB8  ) {
    m_VideoFormat = hal::PB_RGB;
  } else {
    m_VideoFormat = hal::PB_RAW;
  }

  if( pFrame->data_depth == 16 ) {
    m_VideoType = hal::PB_UNSIGNED_SHORT;
  } else {
    m_VideoType = hal::PB_UNSIGNED_BYTE;
  }

  // Release the frame.
  e = dc1394_capture_enqueue( m_vCam[0], pFrame );
}



/////////////////////////////////////////////////////////////////////////////
bool DC1394Driver::Capture( hal::CameraMsg& vImages )
{
  dc1394video_frame_t* pFrame;
  dc1394error_t e;

  for(size_t ii = 0; ii < m_nNumChannels; ++ii) {
    hal::ImageMsg* pbImg = vImages.add_image();

    e = dc1394_capture_dequeue( m_vCam[ii], DC1394_CAPTURE_POLICY_WAIT, &pFrame );
    if( e != DC1394_SUCCESS )
      return false;

    pbImg->set_width( m_nImageWidth );
    pbImg->set_height( m_nImageHeight );
    pbImg->set_data( pFrame->image, pFrame->image_bytes );
    pbImg->set_type( m_VideoType );
    pbImg->set_format( m_VideoFormat );

    if (m_PtGreyTimestamp) {
      double seconds = (pFrame->image[0] >> 1) & 0x7F;
      double cycles = ((pFrame->image[0] & 1) << 12) | (pFrame->image[1] << 4) |
          ((pFrame->image[2] & 0xF0) >> 4);
      double offset = ((pFrame->image[2] & 0x0F) << 8) | pFrame->image[3];
      double device_timestamp = seconds + cycles * 125e-6 +
          offset * (1.0 / 24.576e6);

      if (m_PreviousTimestamp == -1) {
        m_DeviceTimestampOffset = 0;
        m_PreviousTimestamp = device_timestamp;
      } else if (device_timestamp <= m_PreviousTimestamp) {
        m_DeviceTimestampOffset += 128.0;
      }

      m_PreviousTimestamp = device_timestamp;

      std::cout << "Captured with second:" << seconds << " cycles: " <<
                   cycles << " offset: " << offset << " and t: " <<
                   m_DeviceTimestampOffset + device_timestamp << std::endl;
      vImages.set_device_time(
            (double)(m_DeviceTimestampOffset + device_timestamp));
    } else {
      vImages.set_device_time( (double)pFrame->timestamp * 1E-6 );
    }

    vImages.set_system_time(hal::Tic());

    // Add image properties: gain, gamma, etc
    _SetImageMetaDataFromCamera( pbImg, m_vCam[ii] );

    e = dc1394_capture_enqueue( m_vCam[ii], pFrame );
  }
  return true;
}


/////////////////////////////////////////////////////////////////////////////
size_t DC1394Driver::NumChannels() const
{
  return m_nNumChannels;
}

/////////////////////////////////////////////////////////////////////////////
size_t DC1394Driver::Width( size_t /*idx*/ ) const
{
  return m_nImageWidth;
}

/////////////////////////////////////////////////////////////////////////////
size_t DC1394Driver::Height( size_t /*idx*/ ) const
{
  return m_nImageHeight;
}
