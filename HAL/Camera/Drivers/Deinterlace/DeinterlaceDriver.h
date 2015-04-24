#pragma once

#include <memory>

#include <dc1394/conversions.h>

#include <HAL/Camera/CameraDriverInterface.h>


namespace hal
{

class DeinterlaceDriver : public CameraDriverInterface
{
public:
    DeinterlaceDriver( std::shared_ptr<CameraDriverInterface> Input
                 );

    bool Capture( hal::CameraMsg& vImages );
    std::shared_ptr<CameraDriverInterface> GetInputDevice() { return m_Input; }

    std::string GetDeviceProperty(const std::string& sProperty);

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
