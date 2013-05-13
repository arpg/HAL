#pragma once

#include <memory>
#include "HAL/Camera/CameraDriverInterface.h"

#include <Sophus/se3.hpp>
#include <calibu/cam/CameraModel.h>

namespace hal
{

class RectifyDriver : public CameraDriverInterface
{
public:
    RectifyDriver(std::shared_ptr<CameraDriverInterface> input);
    
    bool Capture( pb::CameraMsg& vImages );

protected:
    std::shared_ptr<CameraDriverInterface> m_input;
};

}
