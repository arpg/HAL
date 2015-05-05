#pragma once

#include <HAL/Camera/CameraDriverInterface.h>
#include <opencv2/opencv.hpp>

namespace hal {

class OpenCVDriver : public CameraDriverInterface
{
 public:
  OpenCVDriver(unsigned int nCamId, bool bForceGrey);
  ~OpenCVDriver();

  bool Capture( hal::CameraMsg& vImages );
  std::shared_ptr<CameraDriverInterface> GetInputDevice() { return std::shared_ptr<CameraDriverInterface>(); }

  size_t NumChannels() const;
  size_t Width( size_t /*idx*/ = 0 ) const;
  size_t Height( size_t /*idx*/ = 0 ) const;
 private:
  size_t img_height_;
  size_t img_width_;
  int num_channels_;
  bool force_greyscale_;
  cv::VideoCapture cam_;
};
}  // namespace hal
