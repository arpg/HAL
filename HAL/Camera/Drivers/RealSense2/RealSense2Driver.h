#pragma once

#include <librealsense2/rs.hpp>
#include <HAL/Camera/AutoExposureInterface.h>
#include "RealSense2Cam.h"

namespace hal
{

class RealSense2Driver : public AutoExposureInterface
{
  public:

    RealSense2Driver(int width, int height, int frame_rate, bool capture_color,
        bool capture_depth, bool capture_ir0, bool capture_ir1);

    virtual ~RealSense2Driver();

    bool Capture(CameraMsg& images) override;

    std::shared_ptr<CameraDriverInterface> GetInputDevice() override;

    std::string GetDeviceProperty(const std::string& property) override;

    size_t NumChannels() const override;

    size_t Width(size_t index = 0) const override;

    size_t Height(size_t index = 0) const override;

    double MaxExposure(int channel = 0) const override;

    double MinExposure(int channel = 0) const override;

    double MaxGain(int channel = 0) const override;

    double MinGain(int channel = 0) const override;

    double Exposure(int channel = 0) override;

    void SetExposure(double exposure, int channel = 0) override;

    double Gain(int channel = 0) override;

    void SetGain(double gain, int channel = 0) override;

    double ProportionalGain(int channel = 0) const override;

    double IntegralGain(int channel = 0) const override;

    double DerivativeGain(int channel = 0) const override;

    double Emitter() const;

    void SetEmitter(double emitter) const;

  protected:

    void EnableAutoExposure(int channel);

    void DisableAutoExposure(int channel, double exposure);



    bool IsColorStream(int channel) const;

    const rs2::sensor& GetSensor(int channel) const;

    rs2::sensor& GetSensor(int channel);

  private:

    void Initialize();


  protected:

    std::vector<RealSense2Cam*> cams_;
    int num_streams_per_cam_;

    int width_;

    int height_;

    bool capture_color_;

    bool capture_depth_;

    bool capture_ir0_;

    bool capture_ir1_;

    int frame_rate_;
};

} // namespace hal
