#include "AutoExposureDriver.h"
#include <HAL/Messages/Image.h>

namespace hal
{

AutoExposureDriver::AutoExposureDriver(std::shared_ptr<AutoExposureInterface> input, double p, double i, double d,
    double target, const ImageRoi& roi, double limit, double gain, bool sync,
    int channel, int color) :
  m_input(input),
  m_p(p),
  m_i(i),
  m_d(d),
  m_target(target),
  m_roi(roi),
  m_limit(limit),
  m_gain(gain),
  m_sync(sync),
  m_channel(channel),
  m_color(color),
  m_integral(0.0),
  m_last_error(0.0)
{
  Initialize();
}

bool AutoExposureDriver::Capture(CameraMsg& images)
{
  bool result = m_input->Capture(images);
  if (result) SetExposure(GetExposure(images));
  return result;
}

std::shared_ptr<CameraDriverInterface> AutoExposureDriver::GetInputDevice()
{
  return m_input;
}

std::string AutoExposureDriver::GetDeviceProperty(const std::string& property)
{
  return m_input->GetDeviceProperty(property);
}

size_t AutoExposureDriver::NumChannels() const
{
  return m_input->NumChannels();
}

size_t AutoExposureDriver::Width(size_t index) const
{
  return m_input->Width(index);
}

size_t AutoExposureDriver::Height(size_t index) const
{
  return m_input->Height(index);
}

double AutoExposureDriver::GetTargetIntensity() const
{
  return m_target;
}

void AutoExposureDriver::SetTargetIntensity(double target)
{
  m_target = target;
}

double AutoExposureDriver::GetExposure(const CameraMsg& images)
{
  double bound;
  const double state = GetState();
  const double feedback = GetFeedback(images);
  const double error = m_target - feedback;
  const double update = GetUpdate(error);
  const bool constrained = IsConstrained(error, state, bound);
  return (constrained) ? bound : ClampExposure(state + update);
}

double AutoExposureDriver::GetState() const
{
  return m_input->Exposure(m_channel);
}

double AutoExposureDriver::GetFeedback(const CameraMsg& images) const
{
  int count = 0;
  double sum = 0;

  Image image(images.image(m_channel));
  const cv::Mat& mat = image.Mat();
  cv::Scalar means = cv::mean(mat, m_roi_mask);

  for (int i = 0; i < mat.channels(); ++i)
  {
    if (m_color < 0 || m_color == i)
    {
      sum += means[i];
      ++count;
    }
  }

  return sum / count;
}

bool AutoExposureDriver::IsConstrained(double error, double state,
    double& bound) const
{
  bound = (error > 0) ? m_upperbound : m_lowerbound;
  return IsLowerConstrained(error, state) || IsUpperConstrained(error, state);
}

bool AutoExposureDriver::IsLowerConstrained(double error, double state) const
{
  const double min = m_input->MinExposure(m_channel);
  return (error < 0 && state < min + 1E-8);
}

bool AutoExposureDriver::IsUpperConstrained(double error, double state) const
{
  const double max = m_input->MaxExposure(m_channel);
  return (error > 0 && state > max - 1E-8);
}

double AutoExposureDriver::GetUpdate(double error)
{
  double update = 0;
  update += GetProportionalUpdate(error);
  update += GetIntegralUpdate(error);
  update += GetDerivativeUpdate(error);
  return update;
}

double AutoExposureDriver::GetProportionalUpdate(double error)
{
  return m_p * error;
}

double AutoExposureDriver::GetIntegralUpdate(double error)
{
  m_integral += error;
  return m_i * m_integral;
}

double AutoExposureDriver::GetDerivativeUpdate(double error)
{
  const double delta = error - m_last_error;
  m_last_error = error;
  return m_d * delta;
}

double AutoExposureDriver::ClampExposure(double exposure) const
{
  return std::min(m_upperbound, std::max(m_lowerbound, exposure));
}

void AutoExposureDriver::SetExposure(double exposure)
{
  (m_sync) ? SetAllExposures(exposure) : SetExposure(m_channel, exposure);
}

void AutoExposureDriver::SetAllExposures(double exposure)
{
  for (size_t channel = 0; channel < m_input->NumChannels(); ++channel)
  {
    SetExposure(channel, exposure);
  }
}

void AutoExposureDriver::SetExposure(int channel, double exposure)
{
  m_input->SetExposure(exposure, channel);
  m_input->SetGain(m_gain, channel);
}

void AutoExposureDriver::Initialize()
{
  CreateGains();
  CreateBounds();
  CreateRoiMask();
  CreateCameraGain();
}

void AutoExposureDriver::CreateGains()
{
  if (m_p < 0) m_p = m_input->ProportionalGain(m_channel);
  if (m_i < 0) m_i = m_input->IntegralGain(m_channel);
  if (m_d < 0) m_d = m_input->DerivativeGain(m_channel);
}

void AutoExposureDriver::CreateBounds()
{
  m_limit = std::min(1.0, std::max(0.0, m_limit));
  m_lowerbound = m_input->MinExposure(m_channel);
  m_upperbound = m_input->MaxExposure(m_channel);
  const double range = m_upperbound - m_lowerbound;
  m_upperbound = m_limit * range + m_lowerbound;
}

void AutoExposureDriver::CreateRoiMask()
{
  if (m_roi.w > 0 && m_roi.h > 0)
  {
    const int w = m_input->Width(m_channel);
    const int h = m_input->Height(m_channel);
    cv::Rect roi(m_roi.x, m_roi.y, m_roi.w, m_roi.h);
    m_roi_mask = cv::Mat(h, w, CV_8UC1);
    m_roi_mask = cv::Scalar(0);
    m_roi_mask(roi) = cv::Scalar(1);
  }
}

void AutoExposureDriver::CreateCameraGain()
{
  m_gain = std::min(1.0, std::max(0.0, m_gain));
  const double min = m_input->MinGain(m_channel);
  const double max = m_input->MaxGain(m_channel);
  m_gain = m_gain * (max - min) + min;
}

} // namespace hal