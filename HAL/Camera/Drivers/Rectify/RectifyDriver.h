#pragma once

#include <memory>
#include <HAL/Camera/CameraDriverInterface.h>

#pragma GCC system_header
#include <calibu/cam/camera_crtp.h>
#include <calibu/cam/camera_models_crtp.h>
#include <calibu/cam/camera_rig.h>
#include <calibu/cam/rectify_crtp.h>
#include <calibu/cam/stereo_rectify.h>

namespace hal
{

class RectifyDriver : public CameraDriverInterface
{
public:
    RectifyDriver(std::shared_ptr<CameraDriverInterface> input,
            const std::shared_ptr<calibu::Rig<double>> rig);

    bool Capture( pb::CameraMsg& vImages );
    std::shared_ptr<CameraDriverInterface> GetInputDevice() { return m_input; }

    size_t NumChannels() const;
    size_t Width( size_t /*idx*/ = 0 ) const;
    size_t Height( size_t /*idx*/ = 0 ) const;

    std::string GetDeviceProperty(const std::string& sProperty);

    /// Return rectified right-from-left camera transform.
    inline const Sophus::SE3d& T_rl() const {
        return m_T_nr_nl;
    }

    /// Return rectified camera model for both left and right images.
    const std::shared_ptr<calibu::CameraInterface<double>> CameraModel() const {
        return m_cam;
    }

protected:
    Sophus::SE3d                            m_T_nr_nl;
    std::shared_ptr<calibu::CameraInterface<double>>   m_cam;
    std::shared_ptr<CameraDriverInterface>  m_input;
    std::vector<calibu::LookupTable>        m_vLuts;

};

}
