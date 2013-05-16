#pragma once

#include <memory>

#include <dc1394/conversions.h>

#include <HAL/Camera/CameraDriverInterface.h>


namespace hal
{

class DebayerDriver : public CameraDriverInterface
{
public:
    DebayerDriver( std::shared_ptr<CameraDriverInterface>    Input,
                   const std::string&                        sMethod,
                   const std::string&                        sFilter,
                   unsigned int                              nDepth
                 );

    bool Capture( pb::CameraMsg& vImages );

    std::string GetDeviceProperty(const std::string& sProperty);

    unsigned int Width( unsigned int idx = 0 );

    unsigned int Height( unsigned int idx = 0 );


protected:
    std::shared_ptr<CameraDriverInterface>  m_Input;
    pb::CameraMsg                           m_Message;
    unsigned int                            m_nImgWidth;
    unsigned int                            m_nImgHeight;
    dc1394bayer_method_t                    m_Method;
    dc1394color_filter_t                    m_Filter;
    unsigned int                            m_nDepth;

};

}
