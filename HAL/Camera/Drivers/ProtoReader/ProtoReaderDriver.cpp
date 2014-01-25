#include "ProtoReaderDriver.h"

#include <HAL/Utils/StringUtils.h>

#include <iostream>
#include <unistd.h>

namespace hal {
ProtoReaderDriver::ProtoReaderDriver(std::string filename, size_t imageID)
    : m_first(true),
      m_reader( pb::Reader::Instance(filename,pb::Msg_Type_Camera) ) {
  m_reader.SetInitialImage(imageID);
  while( !ReadNextCameraMessage(m_nextMsg) ) {
    std::cout << "HAL: Initializing proto-reader..." << std::endl;
    usleep(100);
  }

  const pb::Header pbHdr = m_reader.GetHeader();
  time_t log_date((long)pbHdr.date());
  std::cout << "- Log dated " << ctime(&log_date);

  m_numChannels = m_nextMsg.image_size();
  for(size_t c=0; c < m_numChannels; ++c) {
    m_width.push_back(m_nextMsg.image(c).width());
    m_height.push_back(m_nextMsg.image(c).height());
  }
}

ProtoReaderDriver::~ProtoReaderDriver() {
  //    m_reader.StopBuffering();
}

bool ProtoReaderDriver::ReadNextCameraMessage(pb::CameraMsg& msg) {
  msg.Clear();
  std::unique_ptr<pb::CameraMsg> readmsg = m_reader.ReadCameraMsg();
  if(readmsg) {
    msg.Swap(readmsg.get());
    return true;
  }else{
    return false;
  }
}

bool ProtoReaderDriver::Capture( pb::CameraMsg& vImages ) {
  bool success = true;
  if (m_first) {
    m_nextMsg.Swap(&vImages);
    m_first = false;
  } else {
    success = ReadNextCameraMessage(vImages);
  }
  return success && vImages.image_size() > 0;
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
