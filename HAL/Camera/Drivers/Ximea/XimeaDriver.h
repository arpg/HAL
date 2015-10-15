#pragma once

#include <deque>
#include <vector>

#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Utils/Uri.h>

#include <xiApi.h>

namespace hal {


  class XimeaDriver : public CameraDriverInterface
  {
    public:

      XimeaDriver( PropertyMap& default_properties );

      ~XimeaDriver();

      bool Capture(hal::CameraMsg& images);

      void PrintInfo() {}

			hal::Type   PixelType()
			{
				return video_type_;
			}

			hal::Format PixelFormat()
			{
				return video_format_;
			}

      std::shared_ptr<CameraDriverInterface> GetInputDriver() {
        return std::shared_ptr<CameraDriverInterface>();
      }

      size_t NumChannels() const;
      size_t Width(size_t /*idx*/ = 0) const;
      size_t Height(size_t /*idx*/ = 0) const;

    private:
      void _CheckError(XI_RETURN err, std::string place);

    private:
      hal::Type                                       video_type_;
      hal::Format                                     video_format_;

      size_t                              num_channels_;
      std::vector<HANDLE>                 cam_handles_;
      XI_IMG                              image_;
      XI_IMG_FORMAT                       image_format_;
      unsigned int                        image_width_;
      unsigned int                        image_height_;
      int                                 sync_;
      uint8_t                             binning_;
      std::deque<std::vector<double> >    sync_times_;

      // This refers to the containing CameraDevice's property map (NB this is
      // not the C++ parent class).
      PropertyMap&                      device_properties_;
  };

} /* namespace */
