#pragma once

#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Utils/Uri.h>
#include <opencv2/opencv.hpp>

namespace hal {

class OpenCVDriver : public CameraDriverInterface
{
public:
  OpenCVDriver( const Uri& uri );

  OpenCVDriver( PropertyMap& device_properties );

  ~OpenCVDriver();

  bool Capture( hal::CameraMsg& vImages );
  std::shared_ptr<CameraDriverInterface> GetInputDriver()
  {
    return std::shared_ptr<CameraDriverInterface>();
  }

  void PrintInfo() {}

  hal::Type PixelType() {
    return video_type_;
  }

  hal::Format PixelFormat() {
    return video_format_;
  }

  size_t NumChannels() const;
  size_t Width( size_t /*idx*/ = 0 ) const;
  size_t Height( size_t /*idx*/ = 0 ) const;
private:
  hal::Type					video_type_;
  hal::Format				video_format_;
  PropertyMap&			device_properties_;
  size_t            img_height_;
  size_t            img_width_;
  int               num_channels_;
  bool              force_greyscale_;
  cv::VideoCapture  cam_;
  std::vector<std::string> regexs_;

};
}  // namespace hal
