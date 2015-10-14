#include <bitset>
#include <iostream>

#include <dc1394/conversions.h>
#include <dc1394/types.h>

#include <HAL/Utils/TicToc.h>

#include <dc1394/conversions.h>

#include "DC1394Driver.h"


using namespace hal;

// Format7 code from Pangolin library.
// some code from http://roboticssrv.wtb.tue.nl/svn/ros/old/dev/Rene/bumblebeeXB3 which is BSD licensed
std::string str_operation_mode[DC1394_OPERATION_MODE_NUM] = {
  "DC1394_OPERATION_MODE_LEGACY",
  "DC1394_OPERATION_MODE_1394B"
}; // video.h

std::string str_iso_speed[DC1394_ISO_SPEED_NUM] = {
  "DC1394_ISO_SPEED_100",
  "DC1394_ISO_SPEED_200",
  "DC1394_ISO_SPEED_400",
  "DC1394_ISO_SPEED_800",
  "DC1394_ISO_SPEED_1600",
  "DC1394_ISO_SPEED_3200"
}; // video.h


std::string str_frame_rate[DC1394_FRAMERATE_NUM] = { 
  "DC1394_FRAMERATE_1_875",
  "DC1394_FRAMERATE_3_75",
  "DC1394_FRAMERATE_7_5",
  "DC1394_FRAMERATE_15",
  "DC1394_FRAMERATE_30",
  "DC1394_FRAMERATE_60",
  "DC1394_FRAMERATE_120",
  "DC1394_FRAMERATE_240"
}; // video.h

std::string str_video_mode[DC1394_VIDEO_MODE_NUM] = {
  "DC1394_VIDEO_MODE_160x120_YUV444",
  "DC1394_MODE_320x240_YUV422",
  "DC1394_MODE_640x480_YUV411",
  "DC1394_MODE_640x480_YUV422",
  "DC1394_MODE_640x480_RGB8",
  "DC1394_MODE_640x480_MONO8",
  "DC1394_MODE_640x480_MONO16",
  "DC1394_MODE_800x600_YUV422",
  "DC1394_MODE_800x600_RGB8",
  "DC1394_MODE_800x600_MONO8",
  "DC1394_MODE_1024x768_YUV422",
  "DC1394_MODE_1024x768_RGB8",
  "DC1394_MODE_1024x768_MONO8",
  "DC1394_MODE_800x600_MONO16",
  "DC1394_MODE_1024x768_MONO16",
  "DC1394_MODE_1280x960_YUV422",
  "DC1394_MODE_1280x960_RGB8",
  "DC1394_MODE_1280x960_MONO8",
  "DC1394_MODE_1600x1200_YUV422",
  "DC1394_MODE_1600x1200_RGB8",
  "DC1394_MODE_1600x1200_MONO8",
  "DC1394_MODE_1280x960_MONO16",
  "DC1394_MODE_1600x1200_MONO16",
  "DC1394_MODE_EXIF",
  "DC1394_MODE_FORMAT7_0",
  "DC1394_MODE_FORMAT7_1",
  "DC1394_MODE_FORMAT7_2",
  "DC1394_MODE_FORMAT7_3",
  "DC1394_MODE_FORMAT7_4",
  "DC1394_MODE_FORMAT7_5",
  "DC1394_MODE_FORMAT7_6",
  "DC1394_MODE_FORMAT7_7"
}; // types.h

std::string str_color_coding[DC1394_COLOR_CODING_NUM] = {
  "DC1394_COLOR_CODING_MONO8",
  "DC1394_COLOR_CODING_YUV411",
  "DC1394_COLOR_CODING_YUV422",
  "DC1394_COLOR_CODING_YUV444",
  "DC1394_COLOR_CODING_RGB8",
  "DC1394_COLOR_CODING_MONO16",
  "DC1394_COLOR_CODING_RGB16",
  "DC1394_COLOR_CODING_MONO16S",
  "DC1394_COLOR_CODING_RGB16S",
  "DC1394_COLOR_CODING_RAW8",
  "DC1394_COLOR_CODING_RAW16"
}; // types.h

std::string str_color_filter[DC1394_COLOR_FILTER_NUM] = {
  "DC1394_COLOR_FILTER_RGGB",
  "DC1394_COLOR_FILTER_GBRG",
  "DC1394_COLOR_FILTER_GRBG",
  "DC1394_COLOR_FILTER_BGGR"
}; // types.h

std::string str_yuv_byte_order[DC1394_BYTE_ORDER_NUM] = {
  "DC1394_BYTE_ORDER_UYVY",
  "DC1394_BYTE_ORDER_YUYV"
}; // types.h


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
  DC1394Driver::DC1394Driver( PropertyMap& device_properties )
: device_properties_(device_properties)
{
  ImageDim Dims  = device_properties_.GetProperty<ImageDim>("size", ImageDim(640,480));
  ImageRoi ROI   = device_properties_.GetProperty<ImageRoi>("roi", ImageRoi(0,0,0,0));
  float fFPS     = device_properties_.GetProperty<float>("fps", 30);
  float fEXP     = device_properties_.GetProperty<float>("exp", FLT_MAX);
  unsigned int nDMA     = device_properties_.GetProperty<unsigned int>("dma", 4);
  bool ptgrey_timestamp = device_properties_.GetProperty<bool>("ptgrey_timestamp", false);
  std::string sMode     = device_properties_.GetProperty<std::string>("mode", "MONO8");
	std::string sMethod   = device_properties_.GetProperty<std::string>("debayer_method", "bilinear");
	std::string sFilter   = device_properties_.GetProperty<std::string>("debayer_filter", "");

  depth_ = device_properties_.GetProperty("depth", 8);

	// dc1394bayer_method_t Method;
	if( sMethod == "nearest" ) {
		debayer_method_ = DC1394_BAYER_METHOD_NEAREST;
	} else if( sMethod == "simple" ) {
		debayer_method_ = DC1394_BAYER_METHOD_SIMPLE;
	} else if( sMethod == "bilinear" ) {
		debayer_method_ = DC1394_BAYER_METHOD_BILINEAR;
	} else if( sMethod == "hqlinear" ) {
		debayer_method_ = DC1394_BAYER_METHOD_HQLINEAR;
	} else if( sMethod == "downsample" ) {
		debayer_method_ = DC1394_BAYER_METHOD_DOWNSAMPLE;
	} else if( sMethod == "none" ) {
		debayer_method_ = (dc1394bayer_method_t)0;
	}

	// dc1394color_filter_t Filter;
	if( sFilter == "rggb" ) {
		debayer_filter_ = DC1394_COLOR_FILTER_RGGB;
	} else if( sFilter == "gbrg" ) {
		debayer_filter_ = DC1394_COLOR_FILTER_GBRG;
	} else if( sFilter == "grbg" ) {
		debayer_filter_ = DC1394_COLOR_FILTER_GRBG;
	} else {
		debayer_filter_ = DC1394_COLOR_FILTER_BGGR;
	}


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
    if(!device_properties_.Contains(key)) {
      break;
    }
    vCamId.push_back( device_properties_.GetProperty<unsigned int>( key, 0 ) );
  }

  // Get Mode.
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
				depth_ = 16;
      } else if( ROI.w == 1280 ) {
        Mode = DC1394_VIDEO_MODE_1280x960_MONO16;
				depth_ = 16;
      } else if( ROI.w == 1600 ) {
        Mode = DC1394_VIDEO_MODE_1600x1200_MONO16;
				depth_ = 16;
      } else {
        Mode = DC1394_VIDEO_MODE_640x480_MONO16;
				depth_ = 16;
      }
    } else if( sMode == "RGB8" ) {
      if( ROI.w == 1024 ) {
        Mode = DC1394_VIDEO_MODE_1024x768_RGB8;
				depth_ = 8;
      } else if( ROI.w == 1280 ) {
        Mode = DC1394_VIDEO_MODE_1280x960_RGB8;
				depth_ = 8;
      } else if( ROI.w == 1600 ) {
        Mode = DC1394_VIDEO_MODE_1600x1200_RGB8;
				depth_ = 8;
      } else {
        Mode = DC1394_VIDEO_MODE_640x480_RGB8;
				depth_ = 8;
      }
    } else {
      // MONO8
      if( ROI.w == 1024 ) {
        Mode = DC1394_VIDEO_MODE_1024x768_MONO8;
				depth_ = 8;
      } else if( ROI.w == 1280 ) {
        Mode = DC1394_VIDEO_MODE_1280x960_MONO8;
				depth_ = 8;
      } else if( ROI.w == 1600 ) {
        Mode = DC1394_VIDEO_MODE_1600x1200_MONO8;
				depth_ = 8;
      } else {
        Mode = DC1394_VIDEO_MODE_640x480_MONO8;
				depth_ = 8;
      }
    }
  }

  // Get speed.
  unsigned int nISO = device_properties_.GetProperty<unsigned int>("iso", 400);
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
    video_format_ = hal::PB_LUMINANCE;
  } else if( pFrame->color_coding == DC1394_COLOR_CODING_MONO16  ) {
    video_format_ = hal::PB_LUMINANCE;
  } else if( pFrame->color_coding == DC1394_COLOR_CODING_RGB8  ) {
    video_format_ = hal::PB_RGB;
  } else {
    video_format_ = hal::PB_RAW;
  }

  if( pFrame->data_depth == 16 ) {
    video_type_ = hal::PB_UNSIGNED_SHORT;
  } else {
    video_type_ = hal::PB_UNSIGNED_BYTE;
  }

  // pass properties back using driver property map
	if( debayer_method_ != (dc1394bayer_method_t)0 ){
		video_format_ = hal::PB_RGB;
		video_type_   = hal::PB_UNSIGNED_BYTE;
	}

  // Release the frame.
  e = dc1394_capture_enqueue( m_vCam[0], pFrame );
}


/////////////////////////////////////////////////////////////////////////////
void DC1394Driver::PrintInfo()
{
  dc1394error_t e;
  dc1394camera_list_t*  pCameraList = NULL;
  e = dc1394_camera_enumerate( m_pBus, &pCameraList );

  // already set by constructor
  for(size_t ii = 0; ii < m_nNumChannels; ++ii) {
    dc1394camera_t* pCam = m_vCam[ii];

    dc1394_camera_reset(pCam);

    // both
    //    dc1394operation_mode_t  operation_mode;
    //    dc1394speed_t     iso_speed;
    //    uint32_t    iso_channel;
    uint32_t    data_depth;
    //    uint32_t    bandwidth;
    dc1394color_coding_t  color_coding;
    dc1394color_codings_t   color_codings;
    dc1394color_filter_t  color_filter;

    // mode7
    //dc1394format7modeset_t modeset7;
    dc1394format7mode_t   mode7;
    uint32_t    width;
    uint32_t    height;
    uint32_t    left;
    uint32_t    top;
    uint32_t    unit_bytes;
    uint32_t    max_bytes;
    uint32_t    packet_size;
    uint32_t    packet_size_per_frame;
    float       interval;
    uint32_t    pixnum;
    uint64_t    total_bytes;

    // non-scalable
    dc1394framerates_t  framerates;
    dc1394framerate_t   framerate;
    dc1394bool_t    is_color;
    uint32_t    color_coding_data_depth;
    uint32_t    color_coding_bit_size;
    dc1394video_modes_t video_modes;
    e = dc1394_video_get_supported_modes( pCam, &video_modes);
    DC1394_ERR(e, "Could not get supported modes");
    printf("> available video modes: %d\n",video_modes.num);
    for(unsigned int i=0; i<video_modes.num; i++) {
      // video mode
      dc1394video_mode_t video_mode = video_modes.modes[i];
      printf("\t- %s (%d)\n",str_video_mode[video_mode-DC1394_VIDEO_MODE_MIN].c_str(),video_mode);
      // is scalable?
      bool video_mode_is_scalable = dc1394_is_video_mode_scalable(video_mode);
      if (video_mode_is_scalable){
        // mode7 info
        e=dc1394_format7_get_mode_info(pCam, video_modes.modes[i], &mode7);
        DC1394_ERR(e,"Can't get mode 7 info");
        // color codings
        e = dc1394_format7_get_color_codings(pCam, video_mode, &color_codings);
        DC1394_ERR(e,"Can't get mode 7 color codings");
        printf("\t\t> color codings: %d\n",color_codings.num);
        for (unsigned int j=0;j<color_codings.num;j++){
          color_coding = color_codings.codings[j];
          printf("\t\t\t- %s (%d)\n",str_color_coding[color_coding-DC1394_COLOR_CODING_MIN].c_str(),color_coding);
        }
        // color filter
        e = dc1394_format7_get_color_filter(pCam, video_mode, &color_filter);
        DC1394_ERR(e,"Can't get mode 7 color filter");
        printf("\t\t> color filter: %s (%d)\n",str_color_filter[color_filter-DC1394_COLOR_FILTER_MIN].c_str(),color_filter);  
        // other
        printf("\t\t> other:\n");
        e = dc1394_format7_get_max_image_size(pCam, video_mode, &width, &height);
        DC1394_ERR(e,"Can't get max image size");
        printf("\t\t\t- max image size [width, height]:\t\t[%d, %d]\n",width,height); 
        e = dc1394_format7_get_image_size(pCam, video_mode, &width, &height);
        DC1394_ERR(e,"Can't get image size");
        printf("\t\t\t- image size [width, height]:\t\t\t[%d, %d]\n",width,height); 
        e = dc1394_format7_get_image_position(pCam, video_mode, &left, &top);
        DC1394_ERR(e,"Can't get image position");
        printf("\t\t\t- image position [left, top]:\t\t\t[%d, %d]\n",left,top);
        e = dc1394_format7_get_unit_size(pCam, video_mode, &left, &top);
        DC1394_ERR(e,"Can't get unit size");
        printf("\t\t\t- unit size [left, top]:\t\t\t[%d, %d]\n",left,top);
        e = dc1394_format7_get_unit_position(pCam, video_mode, &left, &top);
        DC1394_ERR(e,"Can't get unit position");
        printf("\t\t\t- image position [left, top]:\t\t\t[%d, %d]\n",left,top);
        e = dc1394_format7_get_packet_parameters(pCam, video_mode, &unit_bytes, &max_bytes);
        DC1394_ERR(e,"Can't get packet parameters");
        printf("\t\t\t- packet parameters [unit_bytes, max_bytes]:\t[%d, %d]\n", unit_bytes, max_bytes);
        e = dc1394_format7_get_packet_size(pCam, video_mode, &packet_size);
        DC1394_ERR(e,"Can't get packet size");
        printf("\t\t\t- packet size:\t\t\t\t\t%d\n",packet_size); 
        e = dc1394_format7_get_recommended_packet_size(pCam, video_mode, &packet_size);
        DC1394_ERR(e,"Can't get recommended");
        printf("\t\t\t- recommended packet size:\t\t\t%d\n",packet_size); 
        e = dc1394_format7_get_packets_per_frame(pCam, video_mode, &packet_size_per_frame);
        DC1394_ERR(e,"Can't get packets per frame");
        printf("\t\t\t- packets per frame:\t\t\t\t%d\n",packet_size_per_frame);
        e = dc1394_format7_get_data_depth(pCam, video_mode, &data_depth);
        DC1394_ERR(e,"Can't get data depth");
        printf("\t\t\t- data depth:\t\t\t\t\t%d\n",data_depth);
        e = dc1394_format7_get_frame_interval(pCam, video_mode, &interval);
        DC1394_ERR(e,"Can't get frame interval");
        printf("\t\t\t- frame interval:\t\t\t\t%f\n",interval);
        e = dc1394_format7_get_pixel_number(pCam, video_mode, &pixnum);
        DC1394_ERR(e,"Can't get pixel number");
        printf("\t\t\t- pixel number:\t\t\t\t\t%d\n",pixnum);
        e = dc1394_format7_get_total_bytes(pCam, video_mode, &total_bytes);
        DC1394_ERR(e,"Can't get total bytes");
        printf("\t\t\t- total bytes:\t\t\t\t\t%lu\n",total_bytes);
      }
      else {
        e = dc1394_video_get_supported_framerates(pCam,video_mode,&framerates);
        DC1394_ERR(e,"Can't get framerates");
        printf("\t\t> framerates: %d\n",framerates.num);
        for (unsigned int j=0;j<framerates.num;j++){
          framerate = framerates.framerates[j];
          printf("\t\t\t- %s (%d)\n",str_frame_rate[framerate-DC1394_FRAMERATE_MIN].c_str(),framerate);
        }
        e = dc1394_get_image_size_from_video_mode(pCam, video_mode, &width, &height);
        DC1394_ERR(e,"Can't get image size");
        printf("\t\t> image size [width, height]: [%d, %d]\n", width, height);
        e = dc1394_get_color_coding_from_video_mode(pCam, video_mode, &color_coding);
        DC1394_ERR(e,"Can't get color coding");
        e = dc1394_is_color(color_coding, &is_color);
        DC1394_ERR(e,"Can't get isColor");
        printf("\t\t> is color: %d\n", is_color);
        e = dc1394_get_color_coding_data_depth(color_coding, &color_coding_data_depth);
        DC1394_ERR(e,"Can't get color coding data depth");
        printf("\t\t> color coding data depth: %d\n", color_coding_data_depth);

        e = dc1394_get_color_coding_bit_size(color_coding, &color_coding_bit_size);
        DC1394_ERR(e,"Can't get color coding bit size");
        printf("\t\t> color coding bit size: %d\n", color_coding_bit_size);
      }
    } // for each mode
  } // for each camera
}


/////////////////////////////////////////////////////////////////////////////
bool DC1394Driver::Capture( hal::CameraMsg& vImages )
{
  dc1394video_frame_t* pFrame;
  dc1394error_t e;

	for(size_t ii = 0; ii < m_nNumChannels; ++ii) {
		hal::ImageMsg* pbImg = vImages.add_image();

		e = dc1394_capture_dequeue( m_vCam[ii], DC1394_CAPTURE_POLICY_WAIT, &pFrame );
		if( e != DC1394_SUCCESS ){
			return false;
		}

		//    pbImg->set_data( pFrame->image, pFrame->image_bytes );
		//    pbImg->set_format( hal::PB_RGB );
		//    pbImg->set_type( hal::PB_UNSIGNED_BYTE );

		if(debayer_method_ == DC1394_BAYER_METHOD_DOWNSAMPLE) {
			pbImg->set_width( m_nImageWidth /2 );
			pbImg->set_height( m_nImageHeight / 2 );
			pbImg->mutable_data()->resize( 3 * pbImg->width() * pbImg->height() );
		}
		else{
			pbImg->set_width( m_nImageWidth );
			pbImg->set_height( m_nImageHeight );
			pbImg->mutable_data()->resize( 3 * pbImg->width() * pbImg->height() );
		}

		if( depth_ == 8 ) {
			//    pbImg->set_data( pFrame->image, pFrame->image_bytes );
			uint8_t* in = (uint8_t*)pFrame->image;
      uint8_t* out = (uint8_t*)pbImg->data().data();
			dc1394_bayer_decoding_8bit( in, out, m_nImageWidth, m_nImageHeight,
					debayer_filter_, debayer_method_ );
		}
		else {
			pbImg->set_data( pFrame->image, pFrame->image_bytes );
			std::cerr << "HAL: Error! 16 bit debayering currently not supported." << std::endl;
		}


		// get timestamp out of image
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
	return debayer_method_ == DC1394_BAYER_METHOD_DOWNSAMPLE ? m_nImageWidth/2 : m_nImageWidth;
}

/////////////////////////////////////////////////////////////////////////////
size_t DC1394Driver::Height( size_t /*idx*/ ) const
{
	return debayer_method_ == DC1394_BAYER_METHOD_DOWNSAMPLE ? m_nImageHeight/2 : m_nImageHeight;
}

