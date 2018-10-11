#pragma once

#include <opencv2/opencv.hpp>
#include <HAL/Camera/AutoExposureInterface.h>
#include <HAL/Utils/Uri.h>

namespace hal
{

class AutoExposureDriver : public CameraDriverInterface
{
  public:

    AutoExposureDriver(std::shared_ptr<AutoExposureInterface> input,
        double p, double i, double d, double target, const ImageRoi& roi,
        double limit, double gain, bool sync, int channel, int color);

    bool Capture(hal::CameraMsg& images) override;

    std::shared_ptr<CameraDriverInterface> GetInputDevice() override;

    std::string GetDeviceProperty(const std::string& property) override;

    size_t NumChannels() const override;

    size_t Width(size_t index = 0) const override;

    size_t Height(size_t index = 0) const override;

    double GetTargetIntensity() const;

    void SetTargetIntensity(double target);

  protected:

    double GetExposure(const hal::CameraMsg& images);

    double GetState() const;

    double GetFeedback(const hal::CameraMsg& images) const;

    bool IsConstrained(double error, double state, double& bound) const;

    bool IsLowerConstrained(double error, double state) const;

    bool IsUpperConstrained(double error, double state) const;

    double GetUpdate(double error);

    double GetProportionalUpdate(double error);

    double GetIntegralUpdate(double error);

    double GetDerivativeUpdate(double error);

    double ClampExposure(double exposure) const;

    void SetExposure(double exposure);

    void SetAllExposures(double exposure);

    void SetExposure(int channel, double exposure);

  private:

    void Initialize();

    void CreateGains();

    void CreateBounds();

    void CreateRoiMask();

    void CreateCameraGain();

  protected:

    std::shared_ptr<AutoExposureInterface> m_input;

    double m_p;

    double m_i;

    double m_d;

    double m_target;

    ImageRoi m_roi;

    double m_limit;

    double m_gain;

    bool m_sync;

    int m_channel;

    int m_color;

    cv::Mat m_roi_mask;

    double m_integral;

    double m_last_error;

    double m_lowerbound;

    double m_upperbound;
};

} // namespace hal