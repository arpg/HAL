#pragma once

#include <memory>
#include <HAL/Camera.pb.h>
#include <HAL/Utils/Uri.h>
#include "HAL/Camera/CameraDriverInterface.h"
#include "HAL/Camera/Drivers/UVC/UvcDriver.h"

// Usage autoexposure://uvc://
// At the moment the autoexposure driver is only configured to work with the UVC driver, specifically
// to control the exposure of the twizzler device. More devices should be supported in the future,
// as well as providing functionality to change PD gains from the uri

namespace hal
{

class AutoExposureDriver : public CameraDriverInterface
{
public:
    AutoExposureDriver(const int nTarget,std::shared_ptr<CameraDriverInterface> Input);

    bool Capture( hal::CameraMsg& vImages );
    std::shared_ptr<CameraDriverInterface> GetInputDevice() { return m_Input; }

    std::string GetDeviceProperty(const std::string& sProperty);

    size_t NumChannels() const { return m_Input->NumChannels(); }
    size_t Width( size_t /*idx*/ = 0 ) const { return m_Input->Width(); }
    size_t Height( size_t /*idx*/ = 0 ) const { return m_Input->Height(); }


protected:
    std::shared_ptr<CameraDriverInterface>  m_Input;
    int m_nTarget;
    int m_nExposure;
    float m_fLastError;
    UvcDriver* m_pUvcDriver;
};

}
