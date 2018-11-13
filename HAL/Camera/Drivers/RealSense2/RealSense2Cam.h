#pragma once

#include <librealsense2/rs.hpp>
#include <HAL/Camera/CameraDriverInterface.h>

namespace hal {

class RealSense2Cam {

public:
    RealSense2Cam(rs2::device &device, int width, int height, int frame_rate,
                  bool capture_color, bool capture_depth,
                  bool capture_ir0, bool capture_ir1);

    ~RealSense2Cam();

    size_t NumStreams();
    bool Capture(CameraMsg& images);

    size_t Width(size_t index = 0);

    size_t Height(size_t index = 0);

    double MaxExposure(int channel = 0);

    double MinExposure(int channel = 0);

    double MaxGain(int channel = 0);

    double MinGain(int channel = 0);

    double Exposure(int channel = 0);

    void SetExposure(double exposure, int channel = 0);

    double Gain(int channel = 0);

    void SetGain(double gain, int channel = 0);

    double ProportionalGain(int channel = 0);

    double IntegralGain(int channel = 0);

    double DerivativeGain(int channel = 0);

    double Emitter();

    void SetEmitter(double emitter);

    void EnableAutoExposure(int channel);

    void DisableAutoExposure(int channel, double exposure);

    bool IsColorStream(int channel);

    rs2::sensor& GetSensor(int channel);



private:
    std::string serialNumberStr;
    uint64_t serialNumber;
    std::unique_ptr<rs2::pipeline> pipeline_;
    std::unique_ptr<rs2::config> configuration_;
    std::vector<rs2::video_stream_profile> streams_;
    rs2::sensor depth_sensor_;
    rs2::sensor color_sensor_;
    rs2::frameset frameset_;


    void Initialize();

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

    void CaptureInfraredStream(int index, CameraMsg& images);

    void CaptureColorStream(CameraMsg& images);

    void CaptureDepthStream(CameraMsg& images);


    int width_;

    int height_;

    int frame_rate_;

    bool capture_color_;

    bool capture_depth_;

    bool capture_ir0_;

    bool capture_ir1_;

    double exposure_;

    double gain_;

    double emitter_;
};

} // namespace hal
