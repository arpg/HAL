#include "ProtoReaderDriver.h"

#include <iostream>

namespace hal
{

ProtoReaderDriver::ProtoReaderDriver(std::string filename)
    : m_reader(filename)
{
    std::cout << "Here1" << std::endl;
    ReadNextCameraMessage(m_nextMsg);
    
    std::cout << "Here2" << std::endl;
    
    m_numChannels = m_nextMsg.camera().image_size();
    for(size_t c=0; c < m_numChannels; ++c) {
        m_width.push_back(m_nextMsg.camera().image(c).width());
        m_height.push_back(m_nextMsg.camera().image(c).height());
    }
    std::cout << "Here3" << std::endl;
}

bool ProtoReaderDriver::ReadNextCameraMessage(pb::Msg& msg)
{
    msg.Clear();
//    while(!msg.has_camera()) {
        std::unique_ptr<pb::Msg> readmsg = m_reader.ReadMessage();
        msg.Swap(readmsg.get());
//    }
    return true;
}

bool ProtoReaderDriver::Capture( pb::CameraMsg& vImages )
{
    m_nextMsg.mutable_camera()->Swap(&vImages);
    ReadNextCameraMessage(m_nextMsg);
    return true;
}

size_t ProtoReaderDriver::NumChannels() const
{
    return m_numChannels;
}

size_t ProtoReaderDriver::Width( size_t idx ) const
{
    return m_width[idx];
}

size_t ProtoReaderDriver::Height( size_t idx ) const
{
    return m_height[idx];
}

}
