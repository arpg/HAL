#include "RealSense2Driver.h"
#include "RealSense2Device.h"

#include <iostream>

namespace hal
{

RealSense2Driver::RealSense2Driver(int width, int height, int frame_rate,
    bool capture_color, bool capture_depth, bool capture_ir0,
    bool capture_ir1, const std::vector<std::string>& ids) :
  ids_(ids),
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
  for (size_t i = 0; i < devices_.size(); ++i)
  {
    if (!devices_[i]->Capture(images)) return false;
  }

  return true;
}

std::shared_ptr<CameraDriverInterface> RealSense2Driver::GetInputDevice()
{
  return nullptr;
}

std::string RealSense2Driver::GetDeviceProperty(const std::string&)
{
  return "";
}

size_t RealSense2Driver::NumChannels() const
{
  return channel_count_;
}

size_t RealSense2Driver::Width(size_t index) const
{
  return Device(index)->Width(channel_map_[index]);
}

size_t RealSense2Driver::Height(size_t index) const
{
  return Device(index)->Height(channel_map_[index]);
}

double RealSense2Driver::MaxExposure(int channel) const
{
  return Device(channel)->MaxExposure(channel_map_[channel]);
}

double RealSense2Driver::MinExposure(int channel) const
{
  return Device(channel)->MinExposure(channel_map_[channel]);
}

double RealSense2Driver::MaxGain(int channel) const
{
  return Device(channel)->MaxGain(channel_map_[channel]);
}

double RealSense2Driver::MinGain(int channel) const
{
  return Device(channel)->MinGain(channel_map_[channel]);
}

double RealSense2Driver::Exposure(int channel)
{
  return Device(channel)->Exposure(channel_map_[channel]);
}

void RealSense2Driver::SetExposure(double exposure, int channel)
{
  Device(channel)->SetExposure(exposure, channel_map_[channel]);
}

double RealSense2Driver::Gain(int channel)
{
  return Device(channel)->Gain(channel_map_[channel]);
}

void RealSense2Driver::SetGain(double gain, int channel)
{
  Device(channel)->SetGain(gain, channel_map_[channel]);
}

double RealSense2Driver::ProportionalGain(int channel) const
{
  return Device(channel)->ProportionalGain(channel_map_[channel]);
}

double RealSense2Driver::IntegralGain(int channel) const
{
  return Device(channel)->IntegralGain(channel_map_[channel]);
}

double RealSense2Driver::DerivativeGain(int channel) const
{
  return Device(channel)->DerivativeGain(channel_map_[channel]);
}

double RealSense2Driver::Emitter(int device) const
{
  return devices_[device]->Emitter();
}

void RealSense2Driver::SetEmitter(int device, double emitter) const
{
  devices_[device]->SetEmitter(emitter);
}

size_t RealSense2Driver::NumDevices() const
{
  return devices_.size();
}

std::shared_ptr<RealSense2Device> RealSense2Driver::Device(int channel)
{
  return devices_[device_map_[channel]];
}

std::shared_ptr<const RealSense2Device> RealSense2Driver::Device(
    int channel) const
{
  return devices_[device_map_[channel]];
}

bool RealSense2Driver::ValidDevice(rs2::device& device, const std::string& id)
{
  if (!ValidDevice(device)) return false;
  return (id == device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
}

bool RealSense2Driver::ValidDevice(rs2::device& device)
{
  static const std::string prefix = "Intel RealSense";
  if (!device.supports(RS2_CAMERA_INFO_NAME)) return false;
  const std::string name = device.get_info(RS2_CAMERA_INFO_NAME);
  auto result = std::mismatch(prefix.begin(), prefix.end(), name.begin());
  return result.first == prefix.end();
}

void RealSense2Driver::Initialize()
{
  CreateDevices();
  SetChannelCount();
  CreateMapping();
}

void RealSense2Driver::CreateDevices()
{
  (ids_.empty()) ? CreateAllDevices() : CreateSelectedDevices();
}

void RealSense2Driver::CreateSelectedDevices()
{
  rs2::context context;
  rs2::device_list devices = context.query_devices();

  for (const std::string& id : ids_)
  {
    for (rs2::device&& device : devices)
    {
      if (ValidDevice(device, id))
      {
        devices_.push_back(std::make_shared<RealSense2Device>(device, width_,
            height_, frame_rate_, capture_color_, capture_depth_, capture_ir0_,
            capture_ir1_));
      }
    }
  }
}

void RealSense2Driver::CreateAllDevices()
{
  rs2::context context;
  rs2::device_list devices = context.query_devices();

  for (rs2::device&& device : devices)
  {
    if (ValidDevice(device))
    {
      devices_.push_back(std::make_shared<RealSense2Device>(device, width_,
          height_, frame_rate_, capture_color_, capture_depth_, capture_ir0_,
          capture_ir1_));
    }
  }
}

void RealSense2Driver::SetChannelCount()
{
  channel_count_ = 0;

  for (size_t i = 0; i < devices_.size(); ++i)
  {
    channel_count_ += devices_[i]->NumChannels();
  }
}

void RealSense2Driver::CreateMapping()
{
  size_t index = 0;
  device_map_.resize(channel_count_);
  channel_map_.resize(channel_count_);

  for (size_t i = 0; i < devices_.size(); ++i)
  {
    const size_t count = devices_[i]->NumChannels();

    for (size_t j = 0; j < count; ++j)
    {
      device_map_[index] = i;
      channel_map_[index] = j;
      ++index;
    }
  }
}

} // namespace hal