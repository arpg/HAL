#pragma once

#include <memory>
#include <HAL/Camera/CameraDriverInterface.h>

#include <calibu/cam/CameraRig.h>

namespace hal
{

typedef Eigen::Matrix<Eigen::Vector2f, Eigen::Dynamic, Eigen::Dynamic> lut;

class RectifyDriver : public CameraDriverInterface
{
public:
    RectifyDriver(std::shared_ptr<CameraDriverInterface> input, const calibu::CameraRig& rig);
    
    bool Capture( pb::CameraMsg& vImages );

protected:
    std::shared_ptr<CameraDriverInterface> m_input;
};

}
