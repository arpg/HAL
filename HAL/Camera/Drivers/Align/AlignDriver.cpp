#include "AlignDriver.h"
#include "HAL/Devices/DeviceException.h"
#include "depth_registration.h"
#include "HAL/Messages/Image.h"

#include <iostream>

#include <opencv2/opencv.hpp>

namespace hal
{

AlignDriver::AlignDriver(
    const std::shared_ptr<calibu::Rig<double>> rig, int idx,
    std::shared_ptr<CameraDriverInterface> Input)
  : m_Rig(rig),
    m_RefIdx(idx),
    m_Input(Input)
{
  for (size_t i = 0; i < Input->NumChannels(); ++i) {
    m_nImgWidth.push_back(Input->Width(i));
    m_nImgHeight.push_back(Input->Height(i));
  }

  if (Input->NumChannels() > 1) {
    m_DepthReg.resize(Input->NumChannels());
    for (size_t i = 0; i < Input->NumChannels(); ++i) {
      if (static_cast<int>(i) != m_RefIdx) {
        m_DepthReg[i].reset(DepthRegistration::New
           (cv::Size(m_nImgWidth[m_RefIdx], m_nImgHeight[m_RefIdx]),
            cv::Size(m_nImgWidth[i], m_nImgHeight[i]),
            cv::Size(m_nImgWidth[i], m_nImgHeight[i]),
            0.5f, 20.0f, 0.015f, DepthRegistration::DEFAULT));

        const std::shared_ptr<calibu::CameraInterface<double>> ref_cam = rig->cameras_[m_RefIdx];
        const std::shared_ptr<calibu::CameraInterface<double>> sub_cam =
            (i < rig->cameras_.size() ? rig->cameras_[i] : rig->cameras_.back());

        m_DepthReg[i]->init
            (ref_cam, sub_cam,
             ref_cam->Pose().inverse() * sub_cam->Pose(),
             cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(1, 3, CV_64F),
             cv::Mat::zeros(m_nImgHeight[i], m_nImgWidth[i], CV_32F),
             cv::Mat::zeros(m_nImgHeight[i], m_nImgWidth[i], CV_32F));

        // all the images will be resized to the reference image size
        m_nImgWidth[i] = m_nImgWidth[m_RefIdx];
        m_nImgHeight[i] = m_nImgHeight[m_RefIdx];
      }
    }
  }
}

bool AlignDriver::Capture( hal::CameraMsg& vImages )
{
  hal::CameraMsg message;
  m_Input->Capture( message );

  vImages.set_device_time(message.device_time());
  vImages.set_system_time(message.system_time());

  vImages.clear_image();
  if ( message.image_size() == 1 ) {
    // nothing we can do
    vImages.add_image()->Swap( message.mutable_image(0) ); // passthrough
    return true;
  }

  // align others now
  for (int ii = 0; ii < message.image_size(); ++ii) {
    hal::ImageMsg* pbImg = vImages.add_image();
    if (ii == m_RefIdx) {
      pbImg->Swap(message.mutable_image(m_RefIdx));
    } else {
      hal::ImageMsg& src = *message.mutable_image(ii);
      cv::Mat trg = hal::WriteCvMat(src);
      m_DepthReg[ii]->depthToRGBResolution(trg, trg);

      pbImg->set_width( trg.cols );
      pbImg->set_height( trg.rows );
      pbImg->set_type( src.type() );
      pbImg->set_format( src.format() );
      pbImg->set_serial_number( src.serial_number() );
      pbImg->set_timestamp( src.timestamp() );
      pbImg->set_data( trg.ptr<void>(), trg.rows * trg.cols *
                       sizeofType(src.type()) * sizeofFormat(src.format()) );
    }
  }
  return true;
}

std::string AlignDriver::GetDeviceProperty(const std::string& sProperty)
{
  return m_Input->GetDeviceProperty(sProperty);
}

size_t AlignDriver::NumChannels() const
{
  return m_nImgWidth.size();
}

size_t AlignDriver::Width( size_t idx ) const
{
  return m_nImgWidth[idx];
}

size_t AlignDriver::Height( size_t idx ) const
{
  return m_nImgHeight[idx];
}

unsigned int AlignDriver::sizeofType(hal::Type type) const
{
  switch(type) {
    case hal::PB_BYTE: return sizeof(char);
    case hal::PB_UNSIGNED_BYTE: return sizeof(unsigned char);
    case hal::PB_SHORT: return sizeof(short);
    case hal::PB_UNSIGNED_SHORT: return sizeof(unsigned short);
    case hal::PB_INT: return sizeof(int);
    case hal::PB_UNSIGNED_INT: return sizeof(unsigned int);
    case hal::PB_FLOAT: return sizeof(float);
    case hal::PB_DOUBLE: return sizeof(double);
  }
  return 0;
}

unsigned int AlignDriver::sizeofFormat(hal::Format format) const
{
  switch(format) {
    case hal::PB_LUMINANCE:
    case hal::PB_RAW:
      return 1;
    case hal::PB_RGB:
    case hal::PB_BGR:
      return 3;
    case hal::PB_RGBA:
    case hal::PB_BGRA:
      return 4;
  }
  return 0;
}

} // namespace
