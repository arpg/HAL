#include "ConvertDriver.h"

#include <iostream>

#include <opencv2/opencv.hpp>

namespace hal
{

ConvertDriver::ConvertDriver(
    std::shared_ptr<CameraDriverInterface> Input,
    const std::string& sFormat,
    double dRange
    )
  : m_Input(Input),
    m_sFormat(sFormat),
    m_nCvType(0),
    m_nImgWidth(Input->Width()),
    m_nImgHeight(Input->Height()),
    m_nNumChannels(Input->NumChannels()),
    m_dRange(dRange)
{
}

bool ConvertDriver::Capture( pb::CameraMsg& vImages )
{
  m_Message.Clear();
  m_Input->Capture( m_Message );

  // Guess source color coding.
  if( m_nCvType == 0 ) {
    if(m_Message.image(0).type() == pb::PB_BYTE ||
       m_Message.image(0).type() == pb::PB_UNSIGNED_BYTE) {
      if(m_Message.image(0).format() == pb::PB_LUMINANCE) {
        m_nCvType = CV_8UC1;
      } else if(m_Message.image(0).format() == pb::PB_RGB) {
        m_nCvType = CV_8UC3;
      }
    } else if(m_Message.image(0).type() == pb::PB_UNSIGNED_SHORT ||
              m_Message.image(0).type() == pb::PB_SHORT) {
      if(m_Message.image(0).format() == pb::PB_LUMINANCE) {
        m_nCvType = CV_16UC1;
      } else if(m_Message.image(0).format() == pb::PB_RGB) {
        m_nCvType = CV_16UC3;
      }
    } else if(m_Message.image(0).type() == pb::PB_FLOAT) {
      if (m_Message.image(0).format() == pb::PB_LUMINANCE) {
        m_nCvType = CV_32FC1;
      } else if(m_Message.image(0).format() == pb::PB_RGB) {
        m_nCvType = CV_32FC3;
      }
      // float images are usually in range [0, 1]
    }
  }

  if( m_nCvType == 0 ) {
    std::cerr << "HAL: Error! Could not guess source color coding. Is it RAW?" << std::endl;
    return false;
  }

  // Prepare return images.
  vImages.set_device_time( m_Message.device_time() );

  for(size_t ii = 0; ii < m_nNumChannels; ++ii) {
    pb::ImageMsg* pbImg = vImages.add_image();
    pbImg->set_width( m_nImgWidth );
    pbImg->set_height( m_nImgHeight );
    pbImg->set_type( pb::PB_UNSIGNED_BYTE );
    pbImg->set_format( pb::PB_LUMINANCE );
    pbImg->mutable_data()->resize(m_nImgWidth*m_nImgHeight);
    pbImg->set_timestamp( m_Message.image(ii).timestamp() );
    pbImg->set_serialno( m_Message.image(ii).serialno() );

    cv::Mat sImg(m_nImgHeight, m_nImgWidth, m_nCvType,
                   (void*)m_Message.mutable_image(ii)->data().data());

    cv::Mat dImg(m_nImgHeight, m_nImgWidth, CV_8UC1,
                   (void*)pbImg->data().data());

    // note: cv::cvtColor cannot convert between depth types
    cv::Mat aux;
    switch(m_nCvType) {
      case CV_8UC1:
        std::copy(sImg.begin<unsigned char>(), sImg.end<unsigned char>(),
                  dImg.begin<unsigned char>());
        break;
      case CV_8UC3:
        cv::cvtColor(sImg, dImg, CV_RGB2GRAY); // add support for BGR?
        break;
      case CV_16UC1:
        sImg.convertTo(aux, CV_64FC1);
        aux.convertTo(dImg, CV_8UC1, 255. / m_dRange);
        break;
      case CV_16UC3:
        sImg.convertTo(aux, CV_64FC3);
        aux.convertTo(dImg, CV_8UC3, 255. / m_dRange);
        cv::cvtColor(dImg, dImg, CV_RGB2GRAY);
        break;
      case CV_32FC1:
        sImg.convertTo(dImg, CV_8UC1, 255. / m_dRange);
        break;
      case CV_32FC3:
        sImg.convertTo(aux, CV_8UC3, 255. / m_dRange);
        cv::cvtColor(aux, dImg, CV_RGB2GRAY);
        break;
    }
  }

  return true;
}

std::string ConvertDriver::GetDeviceProperty(const std::string& sProperty)
{
  return m_Input->GetDeviceProperty(sProperty);
}

size_t ConvertDriver::NumChannels() const
{
  return m_nNumChannels;
}

size_t ConvertDriver::Width( size_t /*idx*/ ) const
{
  return m_nImgWidth;
}

size_t ConvertDriver::Height( size_t /*idx*/ ) const
{
  return m_nImgHeight;
}

} // namespace
