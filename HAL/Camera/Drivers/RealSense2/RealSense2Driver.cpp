#include <HAL/Devices/DeviceException.h>
#include <glog/logging.h>
#include "RealSense2Driver.h"
#include <iostream>
#include <HAL/Utils/TicToc.h>

#define DECLARE_CAM_FROM_CHANNEL(channel) auto cam = cams_[(channel) / num_streams_per_cam_]
#define CAM_SPECIFIC_CHANNEL(channel) ((channel) % num_streams_per_cam_)

namespace hal
{

RealSense2Driver::RealSense2Driver(int width, int height, int frame_rate,
    bool capture_color, bool capture_depth, bool capture_ir0,
    bool capture_ir1) :
  width_(width),
  height_(height),
  capture_color_(capture_color),
  capture_depth_(capture_depth),
  capture_ir0_(capture_ir0),
  capture_ir1_(capture_ir1),
  frame_rate_(frame_rate)
{
  Initialize();
}

RealSense2Driver::~RealSense2Driver()
{
}

bool RealSense2Driver::Capture(CameraMsg& images)
{
  images.Clear();
  images.set_device_time(Tic());
  for (auto&& cam : cams_) {
    cam->Capture(images);
  }

  return true;
}

std::shared_ptr<CameraDriverInterface> RealSense2Driver::GetInputDevice()
{
  return std::shared_ptr<CameraDriverInterface>();
}

std::string RealSense2Driver::GetDeviceProperty(const std::string&)
{
  return "";
}

size_t RealSense2Driver::NumChannels() const
{
  return num_streams_per_cam_ * cams_.size();
}

size_t RealSense2Driver::Width(size_t index) const
{
  DECLARE_CAM_FROM_CHANNEL(index);
  return static_cast<size_t>(cam->Width(CAM_SPECIFIC_CHANNEL(index)));
}

size_t RealSense2Driver::Height(size_t index) const
{
  DECLARE_CAM_FROM_CHANNEL(index);
  return static_cast<size_t>(cam->Height(CAM_SPECIFIC_CHANNEL(index)));
}

double RealSense2Driver::MaxExposure(int channel) const
{
  DECLARE_CAM_FROM_CHANNEL(channel);
  return cam->MaxExposure(CAM_SPECIFIC_CHANNEL(channel));
}

double RealSense2Driver::MinExposure(int channel) const
{
  DECLARE_CAM_FROM_CHANNEL(channel);
  return cam->MinExposure(CAM_SPECIFIC_CHANNEL(channel));
}

double RealSense2Driver::MaxGain(int channel) const
{
  DECLARE_CAM_FROM_CHANNEL(channel);
  return cam->MaxGain(CAM_SPECIFIC_CHANNEL(channel));
}

double RealSense2Driver::MinGain(int channel) const
{
  DECLARE_CAM_FROM_CHANNEL(channel);
  return cam->MinGain(CAM_SPECIFIC_CHANNEL(channel));
}

double RealSense2Driver::Exposure(int channel)
{
  DECLARE_CAM_FROM_CHANNEL(channel);
  return cam->Exposure(CAM_SPECIFIC_CHANNEL(channel));
}

void RealSense2Driver::SetExposure(double exposure, int channel)
{
  DECLARE_CAM_FROM_CHANNEL(channel);
  cam->SetExposure(exposure, CAM_SPECIFIC_CHANNEL(channel));
}

double RealSense2Driver::Gain(int channel)
{
  DECLARE_CAM_FROM_CHANNEL(channel);
  return cam->Gain(CAM_SPECIFIC_CHANNEL(channel));
}

void RealSense2Driver::SetGain(double gain, int channel)
{
  DECLARE_CAM_FROM_CHANNEL(channel);
  cam->SetGain(gain, CAM_SPECIFIC_CHANNEL(channel));
}

double RealSense2Driver::ProportionalGain(int channel) const
{
  DECLARE_CAM_FROM_CHANNEL(channel);
  return cam->ProportionalGain(CAM_SPECIFIC_CHANNEL(channel));
}

double RealSense2Driver::IntegralGain(int channel) const
{
  DECLARE_CAM_FROM_CHANNEL(channel);
  return cam->IntegralGain(CAM_SPECIFIC_CHANNEL(channel));
}

double RealSense2Driver::DerivativeGain(int channel) const
{
  DECLARE_CAM_FROM_CHANNEL(channel);
  return cam->DerivativeGain(CAM_SPECIFIC_CHANNEL(channel));
}

double RealSense2Driver::Emitter() const
{
  for (auto &&cam : cams_) {
    return cam->Emitter();
  }
  return 0.0;
}

void RealSense2Driver::SetEmitter(double emitter) const
{
  for (auto &&cam : cams_) {
    cam->SetEmitter(emitter);
  }
}

void RealSense2Driver::EnableAutoExposure(int channel)
{
  DECLARE_CAM_FROM_CHANNEL(channel);
  cam->EnableAutoExposure(CAM_SPECIFIC_CHANNEL(channel));
}

void RealSense2Driver::DisableAutoExposure(int channel, double exposure)
{
  DECLARE_CAM_FROM_CHANNEL(channel);
  cam->DisableAutoExposure(CAM_SPECIFIC_CHANNEL(channel), exposure);
}


bool RealSense2Driver::IsColorStream(int channel) const
{
  DECLARE_CAM_FROM_CHANNEL(channel);
  return cam->IsColorStream(CAM_SPECIFIC_CHANNEL(channel));
}

const rs2::sensor& RealSense2Driver::GetSensor(int channel) const
{
  DECLARE_CAM_FROM_CHANNEL(channel);
  return cam->GetSensor(CAM_SPECIFIC_CHANNEL(channel));
}

rs2::sensor&RealSense2Driver::GetSensor(int channel)
{
  DECLARE_CAM_FROM_CHANNEL(channel);
  return cam->GetSensor(CAM_SPECIFIC_CHANNEL(channel));
}

void RealSense2Driver::Initialize()
{
    rs2::context ctx;
    num_streams_per_cam_ = 0;
    // Query the list of connected RealSense devices
    for (auto &&device : ctx.query_devices()) {
      auto cam = new RealSense2Cam(device, width_, height_, frame_rate_,
        capture_color_, capture_depth_, capture_ir0_, capture_ir1_);
      num_streams_per_cam_ = static_cast<int>(cam->NumStreams());

      cams_.emplace_back(cam);
    }

    LOG(INFO) << "Intialize Done" << std::endl;
}



} // namespace hal
