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
    
    size_t NumChannels() const;
    size_t Width( size_t /*idx*/ = 0 ) const;
    size_t Height( size_t /*idx*/ = 0 ) const;    
    
    std::string GetDeviceProperty(const std::string& sProperty);

protected:
    std::shared_ptr<CameraDriverInterface> m_input;
    std::vector<Eigen::Matrix<Eigen::Vector2f, Eigen::Dynamic, Eigen::Dynamic> > lookups;
    
};

}
