#include <librealsense2/rs.hpp>
#include <glog/logging.h>
#include "RealSense2Cam.h"
#include <iostream>

namespace hal {


RealSense2Cam::RealSense2Cam(rs2::device &device, int width, int height,
                             int frame_rate, bool capture_color,
                             bool capture_depth,
                             bool capture_ir0, bool capture_ir1) :
  width_(width),
  height_(height),
  frame_rate_(frame_rate),
  capture_color_(capture_color),
  capture_depth_(capture_depth),
  capture_ir0_(capture_ir0),
  capture_ir1_(capture_ir1) {
  std::string serial_number(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
  LOG(INFO) << "registering device, serialNumber : " << serial_number
            << std::endl;

  serialNumberStr = serial_number;
  serialNumber = std::strtoull(serial_number.c_str(), nullptr, 10);

  Initialize();
}

void RealSense2Cam::Initialize() {
  CreatePipeline();
  ConfigurePipeline();
  CreateSensors();
  CreateStreams();
}

RealSense2Cam::~RealSense2Cam() {
}


void RealSense2Cam::CreatePipeline()
{
  rs2::pipeline* pointer = new rs2::pipeline;
  pipeline_ = std::unique_ptr<rs2::pipeline>(pointer);
}

void RealSense2Cam::ConfigurePipeline()
{
  CreateConfiguration();
  if (capture_ir0_) ConfigureInfraredStream(1);
  if (capture_ir1_) ConfigureInfraredStream(2);
  if (capture_color_) ConfigureColorStream();
  if (capture_depth_) ConfigureDepthStream();
  pipeline_->start(*configuration_);
}

void RealSense2Cam::CreateConfiguration()
{
  auto pointer = new rs2::config();
  configuration_ = std::unique_ptr<rs2::config>(pointer);
  configuration_->enable_device(serialNumberStr);
  //configuration_->disable_all_streams();
}

void RealSense2Cam::ConfigureInfraredStream(int index)
{
  const int rate = frame_rate_;
  const rs2_format format = RS2_FORMAT_Y8;
  const rs2_stream stream = RS2_STREAM_INFRARED;
  configuration_->enable_stream(stream, index, width_, height_, format, rate);
}

void RealSense2Cam::ConfigureColorStream()
{
  const int rate = frame_rate_;
  const rs2_format format = RS2_FORMAT_RGB8;
  const rs2_stream stream = RS2_STREAM_COLOR;
  configuration_->enable_stream(stream, width_, height_, format, rate);
}

void RealSense2Cam::ConfigureDepthStream()
{
  const int rate = frame_rate_;
  const rs2_format format = RS2_FORMAT_Z16;
  const rs2_stream stream = RS2_STREAM_DEPTH;
  configuration_->enable_stream(stream, width_, height_, format, rate);
}

void RealSense2Cam::CreateSensors()
{
  rs2::pipeline_profile pipeline_profile = pipeline_->get_active_profile();
  const rs2::device device = pipeline_profile.get_device();
  std::vector<rs2::sensor> sensors = device.query_sensors();

  for (rs2::sensor sensor: sensors)
  {
    (sensor.is<rs2::depth_sensor>()) ?
      depth_sensor_ = sensor : color_sensor_ = sensor;
  }
}

void RealSense2Cam::CreateStreams()
{
  if (capture_ir0_) CreateInfraredStream(1);
  if (capture_ir1_) CreateInfraredStream(2);
  if (capture_color_) CreateColorStream();
  if (capture_depth_) CreateDepthStream();
}

void RealSense2Cam::CreateInfraredStream(int index)
{
  rs2::pipeline_profile profile = pipeline_->get_active_profile();
  rs2::stream_profile stream = profile.get_stream(RS2_STREAM_INFRARED, index);
  streams_.push_back(stream.as<rs2::video_stream_profile>());
}

void RealSense2Cam::CreateColorStream()
{
  rs2::pipeline_profile profile = pipeline_->get_active_profile();
  rs2::stream_profile stream = profile.get_stream(RS2_STREAM_COLOR);
  streams_.push_back(stream.as<rs2::video_stream_profile>());
}

void RealSense2Cam::CreateDepthStream()
{
  rs2::pipeline_profile profile = pipeline_->get_active_profile();
  rs2::stream_profile stream = profile.get_stream(RS2_STREAM_DEPTH);
  streams_.push_back(stream.as<rs2::video_stream_profile>());
}

size_t RealSense2Cam::NumStreams() {
  return streams_.size();
}

bool RealSense2Cam::Capture(CameraMsg &images) {
  frameset_ = pipeline_->wait_for_frames();
  if (capture_ir0_) CaptureInfraredStream(1, images);
  if (capture_ir1_) CaptureInfraredStream(2, images);
  if (capture_color_) CaptureColorStream(images);
  if (capture_depth_) CaptureDepthStream(images);
  return true;
}


void RealSense2Cam::CaptureInfraredStream(int index, CameraMsg& images)
{
  ImageMsg* image = images.add_image();
  rs2::video_frame frame = frameset_.get_infrared_frame(index);
  const size_t bytes = frame.get_stride_in_bytes() * frame.get_height();
  image->set_width(frame.get_width());
  image->set_height(frame.get_height());
  image->set_data(frame.get_data(), bytes);
  image->set_format(PB_LUMINANCE);
  image->set_type(PB_UNSIGNED_BYTE);
  image->set_serial_number(serialNumber);
}

void RealSense2Cam::CaptureColorStream(CameraMsg& images)
{
  ImageMsg* image = images.add_image();
  rs2::video_frame frame = frameset_.get_color_frame();
  const size_t bytes = frame.get_stride_in_bytes() * frame.get_height();
  image->set_width(frame.get_width());
  image->set_height(frame.get_height());
  image->set_data(frame.get_data(), bytes);
  image->set_type(PB_UNSIGNED_BYTE);
  image->set_format(PB_RGB);
  image->set_serial_number(serialNumber);
}

void RealSense2Cam::CaptureDepthStream(CameraMsg& images)
{
  ImageMsg* image = images.add_image();
  rs2::video_frame frame = frameset_.get_depth_frame();
  const size_t bytes = frame.get_stride_in_bytes() * frame.get_height();
  image->set_width(frame.get_width());
  image->set_height(frame.get_height());
  image->set_data(frame.get_data(), bytes);
  image->set_format(PB_LUMINANCE);
  image->set_type(PB_SHORT);
  image->set_serial_number(serialNumber);
}



size_t RealSense2Cam::Width(size_t index)
{
  return streams_[index].width();
}

size_t RealSense2Cam::Height(size_t index)
{
  return streams_[index].height();
}

double RealSense2Cam::MaxExposure(int channel)
{
  const rs2_option option = RS2_OPTION_EXPOSURE;
  const rs2::sensor& sensor = GetSensor(channel);
  const rs2::option_range range = sensor.get_option_range(option);
  return range.max;
}

double RealSense2Cam::MinExposure(int channel)
{
  const rs2_option option = RS2_OPTION_EXPOSURE;
  const rs2::sensor& sensor = GetSensor(channel);
  const rs2::option_range range = sensor.get_option_range(option);
  return range.min;
}

double RealSense2Cam::MaxGain(int channel)
{
  const rs2_option option = RS2_OPTION_GAIN;
  const rs2::sensor& sensor = GetSensor(channel);
  const rs2::option_range range = sensor.get_option_range(option);
  return range.max;
}

double RealSense2Cam::MinGain(int channel)
{
  const rs2_option option = RS2_OPTION_GAIN;
  const rs2::sensor& sensor = GetSensor(channel);
  const rs2::option_range range = sensor.get_option_range(option);
  return range.min;
}

double RealSense2Cam::Exposure(int channel)
{
  const rs2_option option = RS2_OPTION_EXPOSURE;
  const rs2::sensor& sensor = GetSensor(channel);
  return sensor.get_option(option);
}

void RealSense2Cam::SetExposure(double exposure, int channel)
{
  (exposure > 0) ?
  DisableAutoExposure(channel, exposure) :
  EnableAutoExposure(channel);
}

double RealSense2Cam::Gain(int channel)
{
  const rs2_option option = RS2_OPTION_GAIN;
  const rs2::sensor& sensor = GetSensor(channel);
  return sensor.get_option(option);
}

void RealSense2Cam::SetGain(double gain, int channel)
{
  gain_ = gain;
  const rs2_option option = RS2_OPTION_GAIN;
  const rs2::sensor& sensor = GetSensor(channel);
  sensor.set_option(option, gain_);
}

double RealSense2Cam::ProportionalGain(int channel)
{
  return IsColorStream(channel) ? 0.45 : 2.5;
}

double RealSense2Cam::IntegralGain(int channel)
{
  return IsColorStream(channel) ? 0.01 : 0.1;
}

double RealSense2Cam::DerivativeGain(int channel)
{
  return IsColorStream(channel) ? 0.5 : 3.0;
}

double RealSense2Cam::Emitter()
{
  if (depth_sensor_.get_option(RS2_OPTION_EMITTER_ENABLED))
  {
    const rs2_option option = RS2_OPTION_LASER_POWER;
    const double value = depth_sensor_.get_option(option);
    const rs2::option_range range = depth_sensor_.get_option_range(option);
    return (value - range.min) / (range.max - range.min);
  }

  return 0.0;
}

void RealSense2Cam::SetEmitter(double emitter)
{
  emitter = std::max(0.0, std::min(1.0, emitter));
  const rs2_option option = RS2_OPTION_LASER_POWER;

  depth_sensor_.set_option(RS2_OPTION_EMITTER_ENABLED, emitter <= 0.0);
  const rs2::option_range range = depth_sensor_.get_option_range(option);
  emitter = range.min + emitter * (range.max - range.min);
  depth_sensor_.set_option(option, emitter);
}

void RealSense2Cam::EnableAutoExposure(int channel)
{
  rs2::sensor& sensor = GetSensor(channel);
  sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, true);

  if (IsColorStream(channel))
  {
    sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, true);
    sensor.set_option(RS2_OPTION_BACKLIGHT_COMPENSATION, true);
  }
}

void RealSense2Cam::DisableAutoExposure(int channel, double exposure)
{
  rs2::sensor& sensor = GetSensor(channel);
  sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, false);
  sensor.set_option(RS2_OPTION_EXPOSURE, exposure);

  if (IsColorStream(channel))
  {
    sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, false);
    sensor.set_option(RS2_OPTION_BACKLIGHT_COMPENSATION, false);
    sensor.set_option(RS2_OPTION_WHITE_BALANCE, 3250);
  }
}


bool RealSense2Cam::IsColorStream(int channel)
{
  return streams_[channel].stream_type() == RS2_STREAM_COLOR;
}

rs2::sensor& RealSense2Cam::GetSensor(int channel)
{
  return IsColorStream(channel) ? color_sensor_ : depth_sensor_;
}

} // namespace hal
