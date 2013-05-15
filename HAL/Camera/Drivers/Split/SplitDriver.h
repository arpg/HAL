#pragma once

#include <memory>
#include <HAL/Utils/Uri.h>
#include "HAL/Camera/CameraDriverInterface.h"


namespace hal
{

class SplitDriver : public CameraDriverInterface
{
public:
     SplitDriver( std::shared_ptr<CameraDriverInterface> Input, std::vector<hal::ImageRoi>& vROIs, bool bCopy );

    bool Capture( pb::CameraMsg& vImages );
    
    std::string GetDeviceProperty(const std::string& sProperty);
    
protected:
    std::shared_ptr<CameraDriverInterface>  m_Input;
    std::vector<hal::ImageRoi>              m_vROIs;
    bool                                    m_bCopy;
};

}
