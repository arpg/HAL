#include "PhotoCalibDriver.h"
#include <HAL/Messages/Image.h>

namespace hal
{

PhotoCalibDriver::PhotoCalibDriver(std::shared_ptr<CameraDriverInterface> input,
    calibu::PhotoCalibd& calib, int channel, int colors, const std::string& in,
    const std::string& out) :
  input_(input),
  calib_(calib),
  channel_(channel),
  colors_(colors),
  in_depth_(in),
  out_depth_(out)
{
}

bool PhotoCalibDriver::Capture(CameraMsg& images)
{
  const bool success = input_->Capture(images);
  if (success) ApplyCorrection(images);
  return success;
}

std::shared_ptr<CameraDriverInterface> PhotoCalibDriver::GetInputDevice()
{
  return input_;
}

size_t PhotoCalibDriver::NumChannels() const
{
  return input_->NumChannels();
}

size_t PhotoCalibDriver::Width(size_t index) const
{
  return input_->Width(index);
}

size_t PhotoCalibDriver::Height(size_t index) const
{
  return input_->Height(index);
}

void PhotoCalibDriver::ApplyCorrection(CameraMsg& images)
{
  // ImageMsg& image = *images.mutable_image(channel_);

  // check input type
  // check output type
  // check image channels
}

void PhotoCalibDriver::Initialize()
{
  CreateResponseTables();
  CreateVignettingTables();
}

void PhotoCalibDriver::CreateResponseTables()
{

}

void PhotoCalibDriver::CreateVignettingTables()
{

}

} // namespace hal