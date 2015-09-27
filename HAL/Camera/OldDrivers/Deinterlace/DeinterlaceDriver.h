#pragma once

#include <memory>

#include <dc1394/conversions.h>

#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Utils/Uri.h>


namespace hal
{

class DeinterlaceDriver : public CameraDriverInterface
{
public:
    DeinterlaceDriver( 
        std::shared_ptr<CameraDriverInterface> input_cam,
        const Uri& uri 
        );
    ~DeinterlaceDriver();

    bool Capture( hal::CameraMsg& vImages );
    std::shared_ptr<CameraDriverInterface> GetInputDriver() { return m_Input; }

    std::string GetProperty(const std::string& sProperty);

    size_t NumChannels() const;
    size_t Width( size_t /*idx*/ = 0 ) const;
    size_t Height( size_t /*idx*/ = 0 ) const;

protected:
    std::shared_ptr<CameraDriverInterface>  m_Input;
    hal::CameraMsg                           m_Message;
    unsigned char*                          m_Buffer;
    unsigned int                            m_nImgWidth;
    unsigned int                            m_nImgHeight;
};

}
