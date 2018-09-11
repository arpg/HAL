#pragma once

#include <librealsense2/rs.hpp>
#include <HAL/Camera/CameraDriverInterface.h>

namespace hal
{

class RealSense2Driver : public CameraDriverInterface
{
  public:

    RealSense2Driver(int width, int height, int frame_rate, bool capture_color,
        bool capture_depth, bool capture_ir0, bool capture_ir1, bool emit_ir,
        double exposure, double gain);

    virtual ~RealSense2Driver();

    bool Capture(CameraMsg& images) override;

    std::shared_ptr<CameraDriverInterface> GetInputDevice() override;

    std::string GetDeviceProperty(const std::string& property) override;

    size_t NumChannels() const override;

    size_t Width(size_t index = 0) const override;

    size_t Height(size_t index = 0) const override;

    bool AutoExposure() const;

    double Exposure() const;

    void SetExposure(double exposure);

    double Gain() const;

    void SetGain(double gain);

  private:

    void Initialize();

    void CreatePipeline();

    void ConfigurePipeline();

    void CreateStreams();

  protected:

    std::unique_ptr<rs2::pipeline> pipeline_;

    std::vector<rs2::video_stream_profile> streams_;

    int width_;

    int height_;

    bool capture_color_;

    bool capture_depth_;

    bool capture_ir0_;

    bool capture_ir1_;

    bool emit_ir_;

    int frame_rate_;

    double exposure_;

    double gain_;
};

} // namespace hal