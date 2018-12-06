#pragma once

#include <librealsense2/rs.hpp>
#include <HAL/Camera/AutoExposureInterface.h>

namespace hal
{

class RealSense2Device : public AutoExposureInterface
{
  public:

    RealSense2Device(rs2::device& device, int width, int height, int frame_rate,
        bool capture_color, bool capture_depth, bool capture_ir0,
        bool capture_ir1);

    virtual ~RealSense2Device();

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

    void CaptureInfraredStream(int index, CameraMsg& images);

    void CaptureColorStream(CameraMsg& images);

    void CaptureDepthStream(CameraMsg& images);

    bool IsColorStream(int channel) const;

    const rs2::sensor& GetSensor(int channel) const;

    rs2::sensor& GetSensor(int channel);

  private:

    void Initialize();

    void CreateSerialNumber();

    void CreatePipeline();

    void ConfigurePipeline();

    void CreateConfiguration();

    void ConfigureInfraredStream(int index);

    void ConfigureColorStream();

    void ConfigureDepthStream();

    void CreateSensors();

    void CreateStreams();

    void CreateInfraredStream(int index);

    void CreateColorStream();

    void CreateDepthStream();

  protected:

    std::unique_ptr<rs2::pipeline> pipeline_;

    std::unique_ptr<rs2::config> configuration_;

    std::vector<rs2::video_stream_profile> streams_;

    rs2::sensor depth_sensor_;

    rs2::sensor color_sensor_;

    rs2::frameset frameset_;

    rs2::device device_;

    std::string serial_string_;

    uint64_t serial_number_;

    int width_;

    int height_;

    bool capture_color_;

    bool capture_depth_;

    bool capture_ir0_;

    bool capture_ir1_;

    int frame_rate_;

    double exposure_;

    double gain_;

    double emitter_;
};

} // namespace hal