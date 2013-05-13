#include "RectifyDriver.h"

namespace hal
{

RectifyDriver::RectifyDriver(std::shared_ptr<CameraDriverInterface> input)
    : m_input(input)
{
    
}

bool RectifyDriver::Capture( pb::CameraMsg& vImages )
{
    return m_input->Capture( vImages );
}

}
