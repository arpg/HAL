#include "RealSense2Driver.h"
#include <iostream>

namespace hal
{

RealSense2Driver::RealSense2Driver(int width, int height, int frame_rate,
    bool capture_color, bool capture_depth, bool capture_ir0, bool capture_ir1,
    bool emit_ir, double exposure, double gain) :
  width_(width),
  height_(height),
  capture_color_(capture_color),
  capture_depth_(capture_depth),
  capture_ir0_(capture_ir0),
  capture_ir1_(capture_ir1),
  emit_ir_(emit_ir),
  frame_rate_(frame_rate),
  exposure_(exposure),
  gain_(gain)
{
  Initialize();
}

RealSense2Driver::~RealSense2Driver()
{
}

bool RealSense2Driver::Capture(CameraMsg& images)
{
  rs2::frameset frameset = pipeline_->wait_for_frames();
  images.set_device_time(frameset.get_timestamp());

  if (capture_ir0_)
  {
    ImageMsg* image = images.add_image();
    rs2::video_frame frame = frameset.get_infrared_frame(1);
    const size_t bytes = frame.get_stride_in_bytes() * frame.get_height();
    image->set_width(frame.get_width());
    image->set_height(frame.get_height());
    image->set_data(frame.get_data(), bytes);
    image->set_format(PB_LUMINANCE);
    image->set_type(PB_UNSIGNED_BYTE);
  }

  if (capture_ir1_)
  {
    ImageMsg* image = images.add_image();
    rs2::video_frame frame = frameset.get_infrared_frame(2);
    const size_t bytes = frame.get_stride_in_bytes() * frame.get_height();
    image->set_width(frame.get_width());
    image->set_height(frame.get_height());
    image->set_data(frame.get_data(), bytes);
    image->set_format(PB_LUMINANCE);
    image->set_type(PB_UNSIGNED_BYTE);
  }

  if (capture_color_)
  {
    ImageMsg* image = images.add_image();
    rs2::video_frame frame = frameset.get_color_frame();
    const size_t bytes = frame.get_stride_in_bytes() * frame.get_height();
    image->set_width(frame.get_width());
    image->set_height(frame.get_height());
    image->set_data(frame.get_data(), bytes);
    image->set_type(PB_UNSIGNED_BYTE);
    image->set_format(PB_RGB);
  }

  if (capture_depth_)
  {
    ImageMsg* image = images.add_image();
    rs2::video_frame frame = frameset.get_depth_frame();
    const size_t bytes = frame.get_stride_in_bytes() * frame.get_height();
    image->set_width(frame.get_width());
    image->set_height(frame.get_height());
    image->set_data(frame.get_data(), bytes);
    image->set_format(PB_LUMINANCE);
    image->set_type(PB_SHORT);
  }

  return true;
}

std::shared_ptr<CameraDriverInterface> RealSense2Driver::GetInputDevice()
{
  return std::shared_ptr<CameraDriverInterface>();
}

std::string RealSense2Driver::GetDeviceProperty(const std::string& property)
{
  return "";
}

size_t RealSense2Driver::NumChannels() const
{
  return streams_.size();
}

size_t RealSense2Driver::Width(size_t index) const
{
  return streams_[index].width();
}

size_t RealSense2Driver::Height(size_t index) const
{
  return streams_[index].height();
}

bool RealSense2Driver::AutoExposure() const
{
  return exposure_ <= 0;
}

double RealSense2Driver::Exposure() const
{
  rs2::pipeline_profile pipeline_profile = pipeline_->get_active_profile();
  std::vector<rs2::sensor> sensors = pipeline_profile.get_device().query_sensors();

  for (rs2::sensor sensor: sensors)
  {
    if (!sensor.is<rs2::depth_sensor>())
    {
      return sensor.get_option(RS2_OPTION_EXPOSURE);
    }
  }

  return exposure_;
}

void RealSense2Driver::SetExposure(double exposure)
{
  exposure_ = exposure;

  rs2::pipeline_profile pipeline_profile = pipeline_->get_active_profile();
  std::vector<rs2::sensor> sensors = pipeline_profile.get_device().query_sensors();

  for (rs2::sensor sensor: sensors)
  {
    if (!sensor.is<rs2::depth_sensor>())
    {
      std::cout.flush();

      if (exposure_ > 0)
      {
        sensor.set_option(RS2_OPTION_BACKLIGHT_COMPENSATION, false);
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, false);
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, false);
        sensor.set_option(RS2_OPTION_WHITE_BALANCE, 3500);
        sensor.set_option(RS2_OPTION_EXPOSURE, exposure_);
        sensor.set_option(RS2_OPTION_GAIN, gain_);
        sensor.set_option(RS2_OPTION_GAMMA, 300);
        sensor.set_option(RS2_OPTION_HUE, 0);
        sensor.set_option(RS2_OPTION_SATURATION, 68);
      }
      else
      {
        sensor.set_option(RS2_OPTION_BACKLIGHT_COMPENSATION, true);
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, true);
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, true);
      }
    }
    else
    {
      std::cout.flush();
    }
  }
}

double RealSense2Driver::Gain() const
{
  return gain_;
}

void RealSense2Driver::SetGain(double gain)
{
  gain_ = gain;

  rs2::pipeline_profile pipeline_profile = pipeline_->get_active_profile();
  std::vector<rs2::sensor> sensors = pipeline_profile.get_device().query_sensors();

  for (rs2::sensor sensor: sensors)
  {
    if (!sensor.is<rs2::depth_sensor>())
    {
      sensor.set_option(RS2_OPTION_GAIN, gain_);
    }
  }
}

void RealSense2Driver::Initialize()
{
  CreatePipeline();
  ConfigurePipeline();
  CreateStreams();
}

void RealSense2Driver::CreatePipeline()
{
  rs2::pipeline* pointer = new rs2::pipeline;
  pipeline_ = std::unique_ptr<rs2::pipeline>(pointer);
}

void RealSense2Driver::ConfigurePipeline()
{
  rs2::config configuration;
  configuration.disable_all_streams();

  if (capture_color_)
  {
    configuration.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_RGB8, frame_rate_);
    // configuration.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RGB8, frame_rate_);
  }

  if (capture_depth_)
  {
    configuration.enable_stream(RS2_STREAM_DEPTH, width_, height_, RS2_FORMAT_Z16, frame_rate_);
  }

  if (capture_ir0_)
  {
    configuration.enable_stream(RS2_STREAM_INFRARED, 1, width_, height_, RS2_FORMAT_Y8, frame_rate_);
  }

  if (capture_ir1_)
  {
    configuration.enable_stream(RS2_STREAM_INFRARED, 2, width_, height_, RS2_FORMAT_Y8, frame_rate_);
  }

  pipeline_->start(configuration);
}

void RealSense2Driver::CreateStreams()
{
  rs2::pipeline_profile pipeline_profile = pipeline_->get_active_profile();
  std::vector<rs2::sensor> sensors = pipeline_profile.get_device().query_sensors();

  for (rs2::sensor sensor: sensors)
  {
    const int count = RS2_OPTION_COUNT;

    for (int i = 0; i < count; ++i)
    {
      const rs2_option option = rs2_option(i);

      if (!sensor.supports(option) ||
          sensor.is_option_read_only(option) ||
          option == RS2_OPTION_POWER_LINE_FREQUENCY ||
          option == RS2_OPTION_EXPOSURE)
      {
        continue;
      }

      const rs2::option_range range = sensor.get_option_range(option);

      std::cout << "Option: " << rs2_option_to_string(option) << " -> " << range.def << std::endl;

      // const float value = sensor.get_option(option);
      // const std::string desc = sensor.get_option_description(option);
      // // const std::string value_desc = sensor.get_option_value_description(option, value);

      // std::cout << "  Description:   " << desc << std::endl;
      // std::cout << "  Current value: " << value << std::endl;
      // // std::cout << "  Current desc:  " << value_desc << std::endl;
      std::cout << "  Value range:   " << range.min << ".." << range.max << std::endl;
      // std::cout << "  Value default: " << range.def << std::endl;
      // std::cout << "  Value step:    " << range.step << std::endl;

      sensor.set_option(option, range.def);
    }

    if (sensor.is<rs2::depth_sensor>())
    {
      std::cout << "====== no exp name: " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
      rs2::depth_sensor depth_sensor = sensor.as<rs2::depth_sensor>();
      depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, emit_ir_);

      if (emit_ir_)
      {
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, 360);
      }
    }
    else
    {
      std::cout << "====== exp name: " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
      if (exposure_ > 0)
      {
        sensor.set_option(RS2_OPTION_BACKLIGHT_COMPENSATION, false);
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, false);
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, false);
        sensor.set_option(RS2_OPTION_WHITE_BALANCE, 3500);
        sensor.set_option(RS2_OPTION_EXPOSURE, exposure_);
        sensor.set_option(RS2_OPTION_GAIN, gain_);
        sensor.set_option(RS2_OPTION_GAMMA, 300);
        sensor.set_option(RS2_OPTION_HUE, 0);
        sensor.set_option(RS2_OPTION_SATURATION, 68);
      }
    }
  }

  if (capture_color_)
  {
    rs2::stream_profile stream = pipeline_profile.get_stream(RS2_STREAM_COLOR);
    streams_.push_back(stream.as<rs2::video_stream_profile>());

    printf("Color calibration (%s):\n", streams_.back().stream_name().c_str());

    printf("  fx: %f, fy: %f, cx: %f, cy: %f\n",
        streams_.back().get_intrinsics().fx,
        streams_.back().get_intrinsics().fy,
        streams_.back().get_intrinsics().ppx,
        streams_.back().get_intrinsics().ppy);

    printf("  distortion model type:   %d\n", (int)streams_.back().get_intrinsics().model);

    printf("  distortion model params: %f %f %f %f %f\n",
        streams_.back().get_intrinsics().coeffs[0],
        streams_.back().get_intrinsics().coeffs[1],
        streams_.back().get_intrinsics().coeffs[2],
        streams_.back().get_intrinsics().coeffs[3],
        streams_.back().get_intrinsics().coeffs[4]);
  }

  if (capture_depth_)
  {
    rs2::stream_profile stream = pipeline_profile.get_stream(RS2_STREAM_DEPTH);
    streams_.push_back(stream.as<rs2::video_stream_profile>());

    printf("Depth calibration (%s):\n", streams_.back().stream_name().c_str());

    printf("  fx: %f, fy: %f, cx: %f, cy: %f\n",
        streams_.back().get_intrinsics().fx,
        streams_.back().get_intrinsics().fy,
        streams_.back().get_intrinsics().ppx,
        streams_.back().get_intrinsics().ppy);

    printf("  distortion model type:   %d\n", (int)streams_.back().get_intrinsics().model);

    printf("  distortion model params: %f %f %f %f %f\n",
        streams_.back().get_intrinsics().coeffs[0],
        streams_.back().get_intrinsics().coeffs[1],
        streams_.back().get_intrinsics().coeffs[2],
        streams_.back().get_intrinsics().coeffs[3],
        streams_.back().get_intrinsics().coeffs[4]);
  }

  if (capture_ir0_)
  {
    rs2::stream_profile stream = pipeline_profile.get_stream(RS2_STREAM_INFRARED, 1);
    streams_.push_back(stream.as<rs2::video_stream_profile>());
  }

  if (capture_ir1_)
  {
    rs2::stream_profile stream = pipeline_profile.get_stream(RS2_STREAM_INFRARED, 2);
    streams_.push_back(stream.as<rs2::video_stream_profile>());
  }
}


} // namespace hal
