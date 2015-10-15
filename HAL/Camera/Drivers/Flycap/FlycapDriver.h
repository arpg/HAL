#pragma once

#include <vector>

#include <dc1394/dc1394.h>
#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Utils/Uri.h>

#include <flycapture/FlyCapture2.h>

namespace hal
 {
	class FlycapDriver : public CameraDriverInterface
	{
		public:
			FlycapDriver( PropertyMap& device_properties );
			~FlycapDriver();

			bool Capture( hal::CameraMsg& vImages );
			std::shared_ptr<CameraDriverInterface> GetInputDriver() { return std::shared_ptr<CameraDriverInterface>(); }

			void PrintInfo()
			{
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
			dc1394bayer_method_t              debayer_method_;
			dc1394color_filter_t              debayer_filter_;
      hal::Type                           video_type_;
      hal::Format                         video_format_;
			void _CheckError( FlyCapture2::Error error );
			std::vector<FlyCapture2::Camera*>   m_vCams;
			unsigned int                        image_width_;
			unsigned int                        image_height_;
			unsigned int                        depth_;
      // This refers to the container (not parent) CameraDevice's property map 
      PropertyMap&                        device_properties_;
	};
}

