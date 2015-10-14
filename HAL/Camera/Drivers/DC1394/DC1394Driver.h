#pragma once

#include <float.h>
#include <dc1394/dc1394.h>

#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Utils/Uri.h>


namespace hal {

  class DC1394Driver : public CameraDriverInterface
  {
    public:
      const static int MAX_FR = -1;
      const static int EXT_TRIG = -1;

      DC1394Driver( PropertyMap& device_properties );

      // CameraDriverInterface requires we implement this function:
      bool Capture( hal::CameraMsg& vImages );


      void PrintInfo();

      std::shared_ptr<CameraDriverInterface> GetInputDriver() 
      { 
        return std::shared_ptr<CameraDriverInterface>(); 
      }

			hal::Type   PixelType()
			{
				return video_type_;
			}

			hal::Format PixelFormat()
			{
				return video_format_;
			}

      size_t NumChannels() const;
      size_t Width( size_t /*idx*/ = 0 ) const;
      size_t Height( size_t /*idx*/ = 0 ) const;

    private:
      void _SetImageMetaDataFromCamera( hal::ImageMsg* img, dc1394camera_t* pCam );
      inline int _NearestValue(int value, int step, int min, int max);

      // Point grey timestamp variables.
      bool                              m_PtGreyTimestamp;
      double                            m_DeviceTimestampOffset;
      double                            m_PreviousTimestamp;

      // This refers to the container (not parent) CameraDevice's property map 
      PropertyMap&                      device_properties_;

      dc1394_t*                         m_pBus;
      hal::Type                         video_type_;
      hal::Format                       video_format_;
      unsigned int                      m_nImageWidth;
      unsigned int                      m_nImageHeight;
      unsigned int                      m_nNumChannels;
      std::vector<dc1394camera_t*>      m_vCam;
			dc1394bayer_method_t              debayer_method_;
			dc1394color_filter_t              debayer_filter_;
      unsigned int                      depth_;
  };

} /* namespace */
