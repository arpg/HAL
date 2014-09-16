#pragma once

#include <vector>
#include <memory>
#include <string>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>

#include "HAL/Camera/CameraDriverInterface.h"

namespace hal {

class Freenect2Driver : public CameraDriverInterface
{
  public:
    Freenect2Driver(
            unsigned int            nWidth,
            unsigned int            nHeight,
            bool                    bCaptureRGB,
            bool                    bCaptureDepth,
            bool                    bCaptureIR,
            bool                    bColor
            );

    virtual ~Freenect2Driver();

    bool Capture( pb::CameraMsg& vImages );
    std::shared_ptr<CameraDriverInterface> GetInputDevice() {
      return std::shared_ptr<CameraDriverInterface>();
    }

    std::string GetDeviceProperty(const std::string& sProperty);

    size_t NumChannels() const;
    size_t Width( size_t idx = 0 ) const;
    size_t Height( size_t idx = 0 ) const;

  private:
    static uint64_t ParseSerialNumber(const std::string& serial);

  private:
    unsigned int  m_nImgWidth;
    unsigned int  m_nImgHeight;
    bool          m_bRGB, m_bDepth, m_bIR, m_bColor;
    std::vector<std::pair<int,int>> m_dimensions;
    libfreenect2::Freenect2 m_freenect2;
    std::vector<std::shared_ptr<libfreenect2::Freenect2Device>> m_devices;
    std::vector<std::shared_ptr
      <libfreenect2::SyncMultiFrameListener>> m_listeners;
    std::vector<uint64_t> m_lSerialNumbers;
};

}
