#include "ProtoReaderDriver.h"

#include <HAL/Utils/StringUtils.h>

#include <iostream>
#include <unistd.h>

namespace hal {
ProtoReaderDriver::ProtoReaderDriver(std::string filename, int camID, size_t imageID,
                                     const std::vector<int>& channels)
    : m_first(true),
      m_camId(camID),
      m_channelIds(channels),
      m_reader( hal::Reader::Instance(filename,hal::Msg_Type_Camera) ) {
  m_reader.SetInitialImage(imageID);
  while( !ReadNextCameraMessage(m_nextMsg) ) {
    std::cout << "HAL: Initializing proto-reader..." << std::endl;
    usleep(100);
  }

  const hal::Header pbHdr = m_reader.GetHeader();
  time_t log_date((long)pbHdr.date());
  std::cout << "- Log dated " << ctime(&log_date);

  if (channels.empty()) {
    m_numChannels = m_nextMsg.image_size();
    for(size_t c=0; c < m_numChannels; ++c) {
      m_width.push_back(m_nextMsg.image(c).width());
      m_height.push_back(m_nextMsg.image(c).height());
    }
  } else {
    m_numChannels = 0;
    for(int id : channels) {
      if (id >= 0 && id < m_nextMsg.image_size()) {
        ++m_numChannels;
        m_width.push_back(m_nextMsg.image(id).width());
        m_height.push_back(m_nextMsg.image(id).height());
      }
    }
  }
}

ProtoReaderDriver::~ProtoReaderDriver() {
  //    m_reader.StopBuffering();
}

bool ProtoReaderDriver::ReadNextCameraMessage(hal::CameraMsg& msg) {
  msg.Clear();
  std::unique_ptr<hal::CameraMsg> readmsg = m_reader.ReadCameraMsg(m_camId);
  if(readmsg) {
    msg.Swap(readmsg.get());
    return true;
  }else{
    return false;
  }
}

bool ProtoReaderDriver::Capture( hal::CameraMsg& vImages ) {
  bool success = true;
  if (m_first) {
    m_nextMsg.Swap(&vImages);
    m_first = false;
  } else {
    success = ReadNextCameraMessage(vImages);
  }
  if (success && !m_channelIds.empty()) {
    pb::CameraMsg vSubImages;
    for (int id : m_channelIds) {
      if (id >= 0 && id < vImages.image_size())
        vSubImages.add_image()->Swap(vImages.mutable_image(id));
    }
    vImages.Swap(&vSubImages);
    return true;
  } else {
    return success && vImages.image_size() > 0;
  }
}

std::string ProtoReaderDriver::GetDeviceProperty(const std::string& sProperty) {
  if(sProperty == hal::DeviceDirectory) {
    return DirUp(m_reader.GetFilename());
  }
  return std::string();
}

size_t ProtoReaderDriver::NumChannels() const {
  return m_numChannels;
}

size_t ProtoReaderDriver::Width( size_t idx ) const {
  return m_width[idx];
}

size_t ProtoReaderDriver::Height( size_t idx ) const {
  return m_height[idx];
}
}  // end namespace hal
