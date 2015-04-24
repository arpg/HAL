#pragma once

#include <memory>
#include <HAL/Camera/CameraDriverInterface.h>

#pragma GCC system_header
#include <calibu/cam/camera_crtp.h>
#include <calibu/cam/rectify_crtp.h>
#include <calibu/cam/stereo_rectify.h>

namespace hal
{

class UndistortDriver : public CameraDriverInterface
{
public:
    UndistortDriver(std::shared_ptr<CameraDriverInterface> input,
            const std::shared_ptr<calibu::Rig<double> > rig);

    bool Capture( hal::CameraMsg& vImages );
    std::shared_ptr<CameraDriverInterface> GetInputDevice() { return m_Input; }

    size_t NumChannels() const;
    size_t Width( size_t idx = 0 ) const;
    size_t Height( size_t idx = 0 ) const;

    std::string GetDeviceProperty(const std::string& sProperty);

    /// Return rectified camera model.
    const std::shared_ptr<calibu::CameraInterface<double>> CameraModel(size_t idx = 0) const {
      if(idx < m_CamModel.size()) {
        return m_CamModel[idx];
      }
      return m_CamModel[0];
    }

protected:
    hal::CameraMsg                                       m_InMsg;
    std::shared_ptr<CameraDriverInterface>              m_Input;
    std::vector<std::shared_ptr<calibu::CameraInterface<double>>>  m_CamModel;
    std::vector<calibu::LookupTable>                    m_vLuts;

};

}
