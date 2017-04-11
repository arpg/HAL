#pragma once

#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Messages/Reader.h>

namespace hal {

class ProtoReaderDriver : public CameraDriverInterface {
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
};

}  // end namespace hal
