#pragma once

#include <HAL/Camera/AutoExposureInterface.h>
#include <HAL/Utils/Uri.h>
#include <HAL/Messages/Reader.h>
#include <HAL/Messages/Image.h>

namespace hal {

class ProtoReaderDriver : public AutoExposureInterface {
 public:
  ProtoReaderDriver(std::string filename, int camID, size_t imageID,
                    bool realtime);
  ~ProtoReaderDriver();

  bool Capture( hal::CameraMsg& vImages );

  std::shared_ptr<CameraDriverInterface> GetInputDevice() {
    return std::shared_ptr<CameraDriverInterface>();
  }

  std::string GetDeviceProperty(const std::string& sProperty);

  size_t NumChannels() const;
  size_t Width( size_t /*idx*/ = 0 ) const;
  size_t Height( size_t /*idx*/ = 0 ) const;

  // // From OpenNI2
  // bool AutoExposure() const;
  // unsigned int Exposure() const;
  // void SetExposure(unsigned int exposure);
  // unsigned int Gain() const;
  // void SetGain(unsigned int gain);
  // // End OpenNI2

  double MaxExposure(int channel) const override;

  double MinExposure(int channel) const override;

  double MaxGain(int channel) const override;

  double MinGain(int channel) const override;

  double Exposure(int channel) override;

  void SetExposure(double exposure, int channel) override;

  double Gain(int channel) override;

  void SetGain(double gain, int channel) override;

  double ProportionalGain(int channel) const override;

  double IntegralGain(int channel) const override;

  double DerivativeGain(int channel) const override;

 protected:
  bool ReadNextCameraMessage(hal::CameraMsg& msg);

  bool                    m_first;
  bool                    m_realtime;
  int                     m_camId;
  hal::Reader&             m_reader;
  hal::CameraMsg           m_nextMsg;

  std::vector<size_t>     m_width;
  std::vector<size_t>     m_height;
  size_t                  m_numChannels;
  std::chrono::steady_clock::time_point m_start_time;
  double                  m_first_frame_time;

  // Autoexposure params
  std::vector<double> m_gains;

  std::vector<double> m_exposures;
  // End autoexposure params
};

}  // end namespace hal
