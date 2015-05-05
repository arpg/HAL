#pragma once

#include <memory>

#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Utils/Uri.h>

namespace hal
{

class ConvertDriver : public CameraDriverInterface
{
public:
    ConvertDriver(std::shared_ptr<CameraDriverInterface> Input,
                   const std::string& sFormat,
                   double dRange,
                   ImageDim dims,
                  int channel);

    bool Capture( hal::CameraMsg& vImages );
    std::shared_ptr<CameraDriverInterface> GetInputDevice() { return m_Input; }

    std::string GetDeviceProperty(const std::string& sProperty);

    size_t NumChannels() const;
    size_t Width( size_t idx = 0 ) const;
    size_t Height( size_t idx = 0 ) const;

protected:
    std::shared_ptr<CameraDriverInterface>  m_Input;
    hal::CameraMsg                           m_Message;
    std::string                             m_sFormat;
    std::vector<int>                        m_nCvType;
    std::vector<hal::Format>                 m_nPbType;
    int                                     m_nOutCvType;
    hal::Format                              m_nOutPbType;
    std::vector<unsigned int>               m_nImgWidth;
    std::vector<unsigned int>               m_nOrigImgWidth;
    std::vector<unsigned int>               m_nImgHeight;
    std::vector<unsigned int>               m_nOrigImgHeight;
    unsigned int                            m_nNumChannels;
    double                                  m_dRange;
    ImageDim                                m_Dims;
    int                                     m_iChannel;
};

}
