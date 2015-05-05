#include "DebayerDriver.h"

#include <iostream>

namespace hal
{

DebayerDriver::DebayerDriver( std::shared_ptr<CameraDriverInterface> Input,
                              dc1394bayer_method_t                   Method,
                              dc1394color_filter_t                   Filter,
                              unsigned int                           nDepth
                              )
  : m_Input(Input),
    m_nImgWidth(Input->Width()),
    m_nImgHeight(Input->Height()),
    m_nNumChannels(Input->NumChannels()),
    m_Method(Method),
    m_Filter(Filter),
    m_nDepth(nDepth)
{
}

bool DebayerDriver::Capture( hal::CameraMsg& vImages )
{
  m_Message.Clear();
  m_Input->Capture( m_Message );

  vImages.set_device_time( m_Message.device_time() );

  for(size_t ii = 0; ii < m_nNumChannels; ++ii) {
    hal::ImageMsg* pbImg = vImages.add_image();
    pbImg->set_format( hal::PB_RGB );
    pbImg->set_type( hal::PB_UNSIGNED_BYTE );
    if(m_Method == DC1394_BAYER_METHOD_DOWNSAMPLE) {
      pbImg->set_width( m_nImgWidth / 2 );
      pbImg->set_height( m_nImgHeight / 2 );
      pbImg->mutable_data()->resize( 3 * m_nImgHeight * m_nImgWidth / 4);
    } else {
      pbImg->set_width( m_nImgWidth );
      pbImg->set_height( m_nImgHeight );
      pbImg->mutable_data()->resize( 3 * m_nImgHeight * m_nImgWidth );
    }
    pbImg->set_timestamp( m_Message.mutable_image(ii)->timestamp() );
    pbImg->mutable_data()->resize( 3 * m_nImgHeight * m_nImgWidth );

    if( m_nDepth == 8 ) {
      dc1394_bayer_decoding_8bit( (uint8_t*)m_Message.mutable_image(ii)->data().data(),
                                  (uint8_t*)pbImg->data().data(), m_nImgWidth, m_nImgHeight,
                                  m_Filter, m_Method );
    } else {
      std::cerr << "HAL: Error! 16 bit debayering currently not supported." << std::endl;
    }
  }

  return true;
}

std::string DebayerDriver::GetDeviceProperty(const std::string& sProperty)
{
  return m_Input->GetDeviceProperty(sProperty);
}

size_t DebayerDriver::NumChannels() const
{
  return m_nNumChannels;
}

size_t DebayerDriver::Width( size_t /*idx*/ ) const
{
  if(m_Method == DC1394_BAYER_METHOD_DOWNSAMPLE) {
    return m_nImgWidth / 2 ;
  } else {
    return m_nImgWidth;
  }
}

size_t DebayerDriver::Height( size_t /*idx*/ ) const
{
  if(m_Method == DC1394_BAYER_METHOD_DOWNSAMPLE) {
    return m_nImgHeight / 2 ;
  } else {
    return m_nImgHeight;
  }
}

} // namespace
