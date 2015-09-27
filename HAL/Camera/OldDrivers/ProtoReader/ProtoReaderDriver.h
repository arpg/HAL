#pragma once

#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Utils/Uri.h>
#include <HAL/Messages/Reader.h>

namespace hal {

class ProtoReaderDriver : public CameraDriverInterface {
 public:
  ProtoReaderDriver( const Uri& uri );
  ~ProtoReaderDriver();

  bool Capture( hal::CameraMsg& vImages );

  std::shared_ptr<CameraDriverInterface> GetInputDriver() {
    return std::shared_ptr<CameraDriverInterface>();
  }

  size_t NumChannels() const;
  size_t Width( size_t /*idx*/ = 0 ) const;
  size_t Height( size_t /*idx*/ = 0 ) const;

 protected:
  bool ReadNextCameraMessage(hal::CameraMsg& msg);

  bool                    m_first;
  int                     m_camId;
  hal::Reader&            m_reader;
  hal::CameraMsg          m_nextMsg;
  std::vector<size_t>     m_width;
  std::vector<size_t>     m_height;
  size_t                  m_numChannels;
};

}  // end namespace hal
