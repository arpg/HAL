#pragma once

#include <memory>
#include <HAL/Camera/CameraDriverInterface.h>

#pragma GCC system_header
#include <calibu/cam/CameraRig.h>
#include <calibu/cam/Rectify.h>
#include <calibu/cam/StereoRectify.h>

namespace hal
{

class UndistortDriver : public CameraDriverInterface
{
public:
    UndistortDriver(std::shared_ptr<CameraDriverInterface> input,
            const calibu::CameraRig& rig);

    bool Capture( pb::CameraMsg& vImages );
    std::shared_ptr<CameraDriverInterface> GetInputDevice() { return m_Input; }

    size_t NumChannels() const;
    size_t Width( size_t idx = 0 ) const;
    size_t Height( size_t idx = 0 ) const;

    std::string GetDeviceProperty(const std::string& sProperty);

    /// Return rectified camera model.
    const calibu::CameraModelT<calibu::Pinhole>& CameraModel(size_t idx = 0) const {
      if(idx < m_CamModel.size()) {
        return m_CamModel[idx];
      }
      return m_CamModel[0];
    }

protected:
    pb::CameraMsg                                       m_InMsg;
    std::shared_ptr<CameraDriverInterface>              m_Input;
    std::vector<calibu::CameraModelT<calibu::Pinhole>>  m_CamModel;
    std::vector<calibu::LookupTable>                    m_vLuts;

};

}
