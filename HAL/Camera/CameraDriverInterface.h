#pragma once

#include <memory>

#include <HAL/Devices/DriverInterface.h>

#include <HAL/Camera.pb.h>

namespace hal {

///////////////////////////////////////////////////////////////////////////////
// Generic camera driver interface
class CameraDriverInterface : public DriverInterface
{
public:
    // Pure virtual functions driver writers must implement:
    virtual ~CameraDriverInterface() {}
    virtual bool Capture( hal::CameraMsg& vImages ) = 0;
    virtual std::shared_ptr<CameraDriverInterface> GetInputDevice() = 0;

    virtual size_t NumChannels() const = 0;
    virtual size_t Width( size_t /*idx*/ = 0 ) const = 0;
    virtual size_t Height( size_t /*idx*/ = 0 ) const = 0;
};

}
