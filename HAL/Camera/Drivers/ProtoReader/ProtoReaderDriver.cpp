#include "ProtoReaderDriver.h"

#include <HAL/Utils/StringUtils.h>

#include <iostream>
#include <unistd.h>
#include <iomanip>

namespace hal {
ProtoReaderDriver::ProtoReaderDriver(std::string filename, int camID, size_t imageID, bool realtime)
    : m_first(true),
      m_camId(camID),
      m_realtime(realtime),
      m_reader( hal::Reader::Instance(filename,hal::Msg_Type_Camera) ){
  m_reader.SetInitialImage(imageID);
  while( !ReadNextCameraMessage(m_nextMsg) ) {
    std::cout << "HAL: Initializing proto-reader..." << std::endl;
    std::cout << "HAL: proto-reader with autoepo setup..." << std::endl;
    usleep(100);
  }

  const hal::Header pbHdr = m_reader.GetHeader();
  time_t log_date((long)pbHdr.date());
  std::cout << "- Log dated " << ctime(&log_date);

  m_numChannels = m_nextMsg.image_size();
  for(size_t c=0; c < m_numChannels; ++c) {
    m_width.push_back(m_nextMsg.image(c).width());
    m_height.push_back(m_nextMsg.image(c).height());
    m_gains.push_back(1.0);
    m_exposures.push_back(1.0);
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

        while(time_since_frist_frame < time_elapsed_since_start){
          success = ReadNextCameraMessage(vImages);
          time_since_frist_frame =
                    vImages.device_time() - m_first_frame_time;
        }
      }
      else if (time_since_frist_frame > time_elapsed_since_start){
        // next frame is in the future, wait until enough time has elapsed
        double T = time_since_frist_frame - time_elapsed_since_start;

        usleep(T*1e6);
      }
    }
  }
  // Apply autoexposure params on the images
  for (int i=0; i<vImages.image_size(); i++){
    std::cout << "exposure: " << m_exposures[i] << std::endl;
    Image(vImages.image(i)).Mat() *= m_gains[i] * m_exposures[i];
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

double ProtoReaderDriver::MaxExposure(int) const {
  return 2.0;
}

double ProtoReaderDriver::MinExposure(int) const {
  return 0.1;
}

double ProtoReaderDriver::MaxGain(int) const {
  return 1.0;
}

double ProtoReaderDriver::MinGain(int ) const {
  return 1.0;
}

double ProtoReaderDriver::Exposure(int channel) {
  return m_exposures[channel];
}

void ProtoReaderDriver::SetExposure(double exposure, int channel) {
  m_exposures[channel] = exposure;
}

double ProtoReaderDriver::Gain(int channel) {
  return m_gains[channel];
}

void ProtoReaderDriver::SetGain(double gain, int channel) {
  m_gains[channel] = gain;
}

double ProtoReaderDriver::ProportionalGain(int) const {
  return 0.075;
}

double ProtoReaderDriver::IntegralGain(int) const {
  return 0.001;
}

double ProtoReaderDriver::DerivativeGain(int) const {
  return 0.05;
}
}  // end namespace hal
