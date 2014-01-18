#pragma once

#include <HAL/Camera/CameraDriverInterface.h>
#include <PbMsgs/Reader.h>

namespace hal {

class ProtoReaderDriver : public CameraDriverInterface {
 public:
  ProtoReaderDriver(std::string filename, size_t imageID);
  ~ProtoReaderDriver();

  bool Capture( pb::CameraMsg& vImages );

  std::shared_ptr<CameraDriverInterface> GetInputDevice() {
    return std::shared_ptr<CameraDriverInterface>();
  }

  std::string GetDeviceProperty(const std::string& sProperty);

  size_t NumChannels() const;
  size_t Width( size_t /*idx*/ = 0 ) const;
  size_t Height( size_t /*idx*/ = 0 ) const;

 protected:
  bool ReadNextCameraMessage(pb::CameraMsg& msg);

  bool                    m_first;
  pb::Reader&             m_reader;
  pb::CameraMsg           m_nextMsg;

  std::vector<size_t>     m_width;
  std::vector<size_t>     m_height;
  size_t                  m_numChannels;
};

}  // end namespace hal
