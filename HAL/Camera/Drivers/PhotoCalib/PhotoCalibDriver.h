#pragma once

#include <opencv2/opencv.hpp>
#include <calibu/pcalib/pcalib.h>
#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Messages/Image.h>

namespace hal
{

class PhotoCorrection;

class PhotoCalibDriver : public CameraDriverInterface
{
  public:

    PhotoCalibDriver(std::shared_ptr<CameraDriverInterface> input,
        calibu::PhotoRigd& rig, Type type);

    bool Capture(CameraMsg& images) override;

    std::shared_ptr<CameraDriverInterface> GetInputDevice() override;

    size_t NumChannels() const override;

    size_t Width(size_t index) const override;

    size_t Height(size_t index) const override;

  protected:

    void Correct(CameraMsg& images);

    bool ShouldCorrect(int index) const;

  private:

    void Initialize();

    void CaptureFormats();

    void ValidateResponse();

    void ValidateVignetting();

    void CreateCorrections();

    bool CorrectionNeeded(int index) const;

    std::shared_ptr<PhotoCorrection> CreateCorrection(size_t index) const;

    template <int channels>
    std::shared_ptr<PhotoCorrection> CreateCorrection(size_t index) const;

    template <int channels, typename in_type>
    std::shared_ptr<PhotoCorrection> CreateCorrection(int index) const;

    template <int channels, typename in_type, typename out_type>
    std::shared_ptr<PhotoCorrection> CreateCorrection(int index) const;

  protected:

    std::shared_ptr<CameraDriverInterface> input_;

    calibu::PhotoRigd rig_;

    Type out_type_;

    std::vector<Type> in_types_;

    std::vector<Format> in_formats_;

    std::vector<std::shared_ptr<PhotoCorrection>> corrections_;
};

} // namespace hal