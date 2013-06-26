#include "ProtoReaderDriver.h"

#include <iostream>

namespace hal
{

ProtoReaderDriver::ProtoReaderDriver(std::string filename)
{
    m_reader = pb::Reader::Instance(filename);
    pb::Reader::ReadCamera = true;

    if( !ReadNextCameraMessage(m_nextMsg) ) {
        std::cerr << "Error reading next message!" << std::endl;
    }

    m_numChannels = m_nextMsg.image_size();
    for(size_t c=0; c < m_numChannels; ++c) {
        m_width.push_back(m_nextMsg.image(c).width());
        m_height.push_back(m_nextMsg.image(c).height());
    }
}

bool ProtoReaderDriver::ReadNextCameraMessage(pb::CameraMsg& msg)
{
    msg.Clear();
    std::unique_ptr<pb::CameraMsg> readmsg = m_reader->ReadCameraMsg();
    if(readmsg) {
        msg.Swap(readmsg.get());
        return true;
    }else{
        return false;
    }
}

bool ProtoReaderDriver::Capture( pb::CameraMsg& vImages )
{
    m_nextMsg.Swap(&vImages);
    if( !ReadNextCameraMessage(m_nextMsg) ) {
        std::cerr << "Error reading next message!" << std::endl;
    }
    return vImages.image_size() > 0;
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
