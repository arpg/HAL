#include "ProtoReaderDriver.h"

#include <HAL/Utils/StringUtils.h>

#include <iostream>
#include <unistd.h>
#include <iomanip>

namespace hal {
ProtoReaderDriver::ProtoReaderDriver(std::string filename, int camID, size_t imageID,
                                     bool realtime)
    : m_first(true),
      m_camId(camID),
      m_realtime(realtime),
      m_reader( hal::Reader::Instance(filename,hal::Msg_Type_Camera) ) {
  m_reader.SetInitialImage(imageID);
  while( !ReadNextCameraMessage(m_nextMsg) ) {
    std::cout << "HAL: Initializing proto-reader..." << std::endl;
    usleep(100);
  }

  const hal::Header pbHdr = m_reader.GetHeader();
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
    if(m_realtime){
      // start a clock with the first call to capture
      // for real-time playback of log file.
      m_start_time = std::chrono::steady_clock::now();

      m_first_frame_time = vImages.device_time();
      }
  } else {
    success = ReadNextCameraMessage(vImages);
    if(success && m_realtime){
      // check if enought/too much time has elapsed
      std::chrono::steady_clock::time_point now_time =
          std::chrono::steady_clock::now();

      double time_elapsed_since_start =
          std::chrono::duration_cast<std::chrono::duration<double> >
          (now_time - m_start_time).count();

      double time_since_frist_frame =
          vImages.device_time() - m_first_frame_time;


      if(time_since_frist_frame < time_elapsed_since_start){
        // this frame is in the past, skip until we catch up to the current time

        /**
        double T = time_elapsed_since_start - time_since_frist_frame;

        std::cerr << std::fixed << std::setprecision(6) <<
                     "skipping ahead - current is " << T << "s behind" <<
                     std::endl;
        **/

        while(time_since_frist_frame < time_elapsed_since_start){
          success = ReadNextCameraMessage(vImages);
          time_since_frist_frame =
                    vImages.device_time() - m_first_frame_time;
        }


      }
      else if (time_since_frist_frame > time_elapsed_since_start){
        // next frame is in the future, wait until enough time has elapsed
        double T = time_since_frist_frame - time_elapsed_since_start;

        /**
        std::cerr << std::fixed << std::setprecision(6) <<
                     "waiting "
                  << T << "s" << std::endl;
        **/

        usleep(T*1e6);
      }

    }

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
