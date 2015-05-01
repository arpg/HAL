#pragma once

#include <memory>
#include <HAL/Camera.pb.h>
#include <HAL/Utils/Uri.h>
#include "HAL/Camera/CameraDriverInterface.h"


namespace hal
{

class SplitDriver : public CameraDriverInterface
{
public:
    SplitDriver(std::shared_ptr<CameraDriverInterface> Input, std::vector<hal::ImageRoi>& vROIs);

    bool Capture( hal::CameraMsg& vImages );
    std::shared_ptr<CameraDriverInterface> GetInputDevice() { return m_Input; }

    std::string GetDeviceProperty(const std::string& sProperty);

    size_t NumChannels() const;
    size_t Width( size_t /*idx*/ = 0 ) const;
    size_t Height( size_t /*idx*/ = 0 ) const;


protected:
    std::shared_ptr<CameraDriverInterface>  m_Input;
    hal::CameraMsg                           m_InMsg;
    std::vector<hal::ImageRoi>              m_vROIs;
};

}
