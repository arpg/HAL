#include <HAL/Camera/Drivers/AutoExposure/AutoExposureDriver.h>
#include <functional>

namespace hal
{

AutoExposureDriver::AutoExposureDriver(
    std::shared_ptr<CameraDriverInterface> input) :
  m_input(std::dynamic_pointer_cast<AutoExposureCamera>(input)),
  m_exposureController(GetExposureControlFunc()),
  m_gainController(GetGainControlFunc()),
  m_autoExposureEnabled(true),
  m_desiredLuminance(100),
  m_changingExposure(true)
{
  Initialize();
}

AutoExposureDriver::~AutoExposureDriver()
{
}

bool AutoExposureDriver::GetAutoExposureEnabled() const
{
  return m_autoExposureEnabled;
}

void AutoExposureDriver::SetAutoExposureEnabled(bool enabled)
{
  m_autoExposureEnabled = enabled;
  if (!m_autoExposureEnabled) Reset();
}

double AutoExposureDriver::GetDesiredLuminance() const
{
  return m_desiredLuminance;
}

void AutoExposureDriver::SetDesiredLuminance(double luminance)
{
  m_desiredLuminance = luminance;
  m_exposureController.setpoint = m_desiredLuminance;
  m_gainController.setpoint = m_desiredLuminance;
}

bool AutoExposureDriver::Capture(CameraMsg& images)
{
  const bool success = m_input->Capture(images);

  if (success && m_autoExposureEnabled)
  {
    const int index = m_input->GetAutoExposureImageIndex();
    ImageMsg& image = *images.mutable_image(index);
    ProcessImage(image);
  }

  return success;
}

std::shared_ptr<CameraDriverInterface> AutoExposureDriver::GetInputDevice()
{
  return m_input;
}

size_t AutoExposureDriver::NumChannels() const
{
  return m_input->NumChannels();
}

size_t AutoExposureDriver::Width(size_t) const
{
  return m_input->Width();
}

size_t AutoExposureDriver::Height(size_t) const
{
  return m_input->Height();
}

void AutoExposureDriver::Reset()
{
  m_exposureController.Reset();
  m_gainController.Reset();
  m_lightMeter.Reset();
}

void AutoExposureDriver::ProcessImage(const hal::ImageMsg& image)
{
  const double luminance = m_lightMeter.Measure(image);

  (luminance < m_desiredLuminance) ?
      BrightenImage(luminance) : DarkenImage(luminance);
}

void AutoExposureDriver::BrightenImage(double luminance)
{
  (m_input->GetActualFramerate() < m_input->GetDesiredFramerate()) ?
      ChangeExposure(luminance) : IncreaseGain(luminance);
}

void AutoExposureDriver::DarkenImage(double luminance)
{
  (m_input->GetActualFramerate() > m_input->GetDesiredFramerate()) ?
      ChangeExposure(luminance) : DecreaseGain(luminance);
}

void AutoExposureDriver::IncreaseGain(double luminance)
{
  (m_input->GetGain() < m_input->GetMaxGain()) ?
        ChangeGain(luminance) : ChangeExposure(luminance);
}

void AutoExposureDriver::DecreaseGain(double luminance)
{
  (m_input->GetGain() > 0) ? ChangeGain(luminance) : ChangeExposure(luminance);
}

void AutoExposureDriver::ChangeExposure(double luminance)
{
  if (!m_changingExposure)
  {
    m_exposureController.Reset();
    m_changingExposure = true;
  }

  m_exposureController.Update(luminance);
}

void AutoExposureDriver::ChangeGain(double luminance)
{
  if (m_changingExposure)
  {
    m_gainController.Reset();
    m_changingExposure = false;
  }

  m_gainController.Update(luminance);
}

void AutoExposureDriver::Initialize()
{
  InitExposureController();
  InitGainController();
}

PIDController::ControlFunc AutoExposureDriver::GetExposureControlFunc()
{
  using std::placeholders::_1;
  return std::bind(&AutoExposureCamera::SetExposure, m_input, _1);
}

PIDController::ControlFunc AutoExposureDriver::GetGainControlFunc()
{
  using std::placeholders::_1;
  return std::bind(&AutoExposureCamera::SetGain, m_input, _1);
}

void AutoExposureDriver::InitExposureController()
{
  m_exposureController.setpoint = m_desiredLuminance;
  m_exposureController.Kp = 0.010;
  m_exposureController.Ki = 0.005;
  m_exposureController.Kd = 0.005;
}

void AutoExposureDriver::InitGainController()
{
  m_gainController.setpoint = m_desiredLuminance;
  m_gainController.Kp = 0.010;
  m_gainController.Ki = 0.005;
  m_gainController.Kd = 0.007;
}

}