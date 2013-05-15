#pragma once

#include <memory>
#include <PbMsgs/Camera.pb.h>
#include <HAL/Utils/Uri.h>
#include "HAL/Camera/CameraDriverInterface.h"


namespace hal
{

class SplitDriver : public CameraDriverInterface
{
public:
    SplitDriver(std::shared_ptr<CameraDriverInterface> Input, std::vector<hal::ImageRoi>& vROIs);

    bool Capture( pb::CameraMsg& vImages );

    std::string GetDeviceProperty(const std::string& sProperty);

    unsigned int Width( unsigned int idx = 0 );

    unsigned int Height( unsigned int idx = 0 );


protected:
    std::shared_ptr<CameraDriverInterface>  m_Input;
    pb::CameraMsg                           m_InMsg;
    std::vector<hal::ImageRoi>              m_vROIs;
};

}
