#pragma once

#include <opencv2/opencv.hpp>
#include <calibu/pcalib/pcalib.h>
#include <HAL/Camera/CameraDriverInterface.h>

namespace hal
{

class PhotoCalibDriver : public CameraDriverInterface
{
  public:

    PhotoCalibDriver(std::shared_ptr<CameraDriverInterface> input,
        calibu::PhotoCalibd& calib, int channel, int colors,
        const std::string& in, const std::string& out);

    bool Capture(hal::CameraMsg& images) override;

    std::shared_ptr<CameraDriverInterface> GetInputDevice() override;

    size_t NumChannels() const override;

    size_t Width(size_t index) const override;

    size_t Height(size_t index) const override;

  protected:

    void ApplyCorrection(hal::CameraMsg& images);

  private:

    void Initialize();

    void CreateResponseTables();

    void CreateVignettingTables();

  protected:

    std::shared_ptr<CameraDriverInterface> input_;

    calibu::PhotoCalibd calib_;

    int channel_;

    int colors_;

    std::string in_depth_;

    std::string out_depth_;
};

} // namespace hal