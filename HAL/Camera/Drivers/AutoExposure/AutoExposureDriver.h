#pragma once

#include <HAL/Camera/AutoExposureCamera.h>
#include <HAL/Camera/Drivers/AutoExposure/LightMeter.h>
#include <HAL/Camera/Drivers/AutoExposure/PIDController.h>

namespace hal
{

class AutoExposureDriver : public CameraDriverInterface
{
  public:

    AutoExposureDriver(std::shared_ptr<CameraDriverInterface> input);

    ~AutoExposureDriver();

    bool GetAutoExposureEnabled() const;

    void SetAutoExposureEnabled(bool enabled);

    double GetDesiredLuminance() const;

    void SetDesiredLuminance(double luminance);

    bool Capture(hal::CameraMsg& images) override;

    std::shared_ptr<CameraDriverInterface> GetInputDevice() override;

    size_t NumChannels() const override;

    size_t Width(size_t) const override;

    size_t Height(size_t) const override;

    void Reset();

  private:

    inline void ProcessImage(const hal::ImageMsg& image);

    inline void BrightenImage(double luminance);

    inline void DarkenImage(double luminance);

    inline void IncreaseGain(double luminance);

    inline void DecreaseGain(double luminance);

    inline void ChangeExposure(double luminance);

    inline void ChangeGain(double luminance);

    inline PIDController::ControlFunc GetExposureControlFunc();

    inline PIDController::ControlFunc GetGainControlFunc();

    inline void Initialize();

    inline void InitExposureController();

    inline void InitGainController();

  protected:

    std::shared_ptr<AutoExposureCamera> m_input;

    PIDController m_exposureController;

    PIDController m_gainController;

    LightMeter m_lightMeter;

  private:

    bool m_autoExposureEnabled;

    double m_desiredLuminance;

    bool m_changingExposure;
};

} // namespace hal