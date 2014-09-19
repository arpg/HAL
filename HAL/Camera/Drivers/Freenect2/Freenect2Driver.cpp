#include <iostream>
#include <memory>
#include <string>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/threading.h>
#include <libfreenect2/frame_listener_impl.h>

#include <opencv2/opencv.hpp>

#include <HAL/Devices/DeviceException.h>
#include <HAL/Utils/TicToc.h>

#include "Freenect2Driver.h"

using namespace hal;

Freenect2Driver::Freenect2Driver(
        unsigned int            nWidth,
        unsigned int            nHeight,
        bool                    bCaptureRGB,
        bool                    bCaptureDepth,
        bool                    bCaptureIR,
        bool                    bColor)
  : m_nImgWidth(nWidth), m_nImgHeight(nHeight), m_bRGB(bCaptureRGB),
    m_bDepth(bCaptureDepth), m_bIR(bCaptureIR), m_bColor(bColor)
{
  const int N = m_freenect2.enumerateDevices();
  if(N == 0) {
    throw DeviceException("Freenect2 could not find any camera");
  }

  unsigned int types = 0;
  if(bCaptureRGB) types |= libfreenect2::Frame::Color;
  if(bCaptureIR) types |= libfreenect2::Frame::Ir;
  if(bCaptureDepth) types |= libfreenect2::Frame::Depth;
  if(types == 0) {
    throw DeviceException("Freenect2: no channel given (rgb, ir, depth)");
  }

  m_devices.reserve(N);
  m_listeners.reserve(N);
  m_lSerialNumbers.reserve(N);

  for(int i = 0; i < N; ++i) {
    std::shared_ptr<libfreenect2::Freenect2Device> d(m_freenect2.openDevice(i));
    if(!d) throw DeviceException("Freenect2 could not open device " +
                                 m_freenect2.getDeviceSerialNumber(i));

    std::shared_ptr<libfreenect2::SyncMultiFrameListener> listener =
        std::make_shared<libfreenect2::SyncMultiFrameListener>(types);

    if(bCaptureRGB) {
      d->setColorFrameListener(listener.get());
      m_dimensions.push_back(std::make_pair(m_nImgWidth, m_nImgHeight));
    }
    if(bCaptureIR || bCaptureDepth) {
      d->setIrAndDepthFrameListener(listener.get());
      if(bCaptureIR) m_dimensions.push_back(std::make_pair(512, 424));
      if(bCaptureDepth) m_dimensions.push_back(std::make_pair(512, 424));
    }

    m_devices.emplace_back(d);
    m_listeners.emplace_back(listener);
    m_lSerialNumbers.push_back(ParseSerialNumber(d->getSerialNumber()));

    d->start();
  }
}

Freenect2Driver::~Freenect2Driver()
{
  for(auto& pdev : m_devices) {
    pdev->stop();
    pdev->close();
  }
}

uint64_t Freenect2Driver::ParseSerialNumber(const std::string& serial)
{
  try {
    return std::strtoull(serial.c_str(), nullptr, 10);
  } catch (...) {
    return 0;
  }
}

bool Freenect2Driver::Capture( pb::CameraMsg& vImages )
{
  vImages.Clear();
  vImages.set_device_time(Tic());

  libfreenect2::FrameMap frames;
  libfreenect2::FrameMap::const_iterator fit;
  for(size_t i = 0; i < m_devices.size(); ++i) {
    m_listeners[i]->waitForNewFrame(frames);
    const double time = Tic();

    if((fit = frames.find(libfreenect2::Frame::Color)) != frames.end()) {
      pb::ImageMsg* pbImg = vImages.add_image();
      pbImg->set_timestamp(time);
      pbImg->set_width(m_nImgWidth);
      pbImg->set_height(m_nImgHeight);
      pbImg->set_serial_number(m_lSerialNumbers[i]);

      pbImg->set_type(pb::PB_UNSIGNED_BYTE);
      if(m_bColor) pbImg->set_format(pb::PB_BGR);
      else pbImg->set_format(pb::PB_LUMINANCE);

      const libfreenect2::Frame* frame = fit->second;
      cv::Mat trg(frame->height, frame->width, CV_8UC3, frame->data);
      if(frame->height != m_nImgHeight || frame->width != m_nImgWidth)
        cv::resize(trg, trg, cv::Size(m_nImgWidth, m_nImgHeight));
      if(!m_bColor) cv::cvtColor(trg, trg, CV_BGR2GRAY);
      cv::flip(trg, trg, 1);

      pbImg->set_data(trg.ptr<unsigned char>(), trg.rows * trg.cols *
                      trg.channels());
    }

    if((fit = frames.find(libfreenect2::Frame::Ir)) != frames.end()) {
      const libfreenect2::Frame* frame = fit->second;
      pb::ImageMsg* pbImg = vImages.add_image();
      pbImg->set_timestamp(time);
      pbImg->set_width(frame->width);
      pbImg->set_height(frame->height);
      pbImg->set_serial_number(m_lSerialNumbers[i]);

      pbImg->set_type(pb::PB_FLOAT);
      pbImg->set_format(pb::PB_LUMINANCE);

      cv::Mat trg(frame->height, frame->width, CV_32F, frame->data);
      cv::flip(trg, trg, 1);
      pbImg->set_data(trg.ptr<float>(), trg.rows * trg.cols * sizeof(float));
    }

    if((fit = frames.find(libfreenect2::Frame::Depth)) != frames.end()) {
      const libfreenect2::Frame* frame = fit->second;
      pb::ImageMsg* pbImg = vImages.add_image();
      pbImg->set_timestamp(time);
      pbImg->set_width(frame->width);
      pbImg->set_height(frame->height);
      pbImg->set_serial_number(m_lSerialNumbers[i]);

      pbImg->set_type(pb::PB_FLOAT);
      pbImg->set_format(pb::PB_LUMINANCE);

      cv::Mat trg(frame->height, frame->width, CV_32F, frame->data);
      cv::flip(trg, trg, 1);
      pbImg->set_data(trg.ptr<float>(), trg.rows * trg.cols * sizeof(float));
    }
  }

  return true;
}

std::string Freenect2Driver::GetDeviceProperty(const std::string& sProperty)
{
  return std::string();
}

size_t Freenect2Driver::NumChannels() const
{
  int channels_per_device = 0;
  if(m_bRGB) ++channels_per_device;
  if(m_bIR) ++channels_per_device;
  if(m_bDepth) ++channels_per_device;
  return m_devices.size() * channels_per_device;
}

size_t Freenect2Driver::Width( size_t idx ) const
{
  return m_dimensions[idx].first;
}

size_t Freenect2Driver::Height( size_t idx ) const
{
  return m_dimensions[idx].second;
}
