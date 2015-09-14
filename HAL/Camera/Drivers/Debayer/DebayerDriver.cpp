#include "DebayerDriver.h"

#include <iostream>

namespace hal
{

  DebayerDriver::DebayerDriver( 
      std::shared_ptr<CameraDriverInterface> input_cam,
      const Uri& uri
      )
    : m_Input(input_cam)
  {
    // sat sane default parameters
    CameraDriverInterface::SetDefaultProperties({
        {"method","downsample","Debayer method: nearest, simple, bilinear, hqlinear, downsample"},
        {"filter","rggb","Debayer filter: rggb, gbrg, grbg, bggr"},
        {"depth","8","Pixel depth: 8 or 16."}
        });
    if( !CameraDriverInterface::ParseUriProperties( uri.properties ) ){
      std::cerr << "DebayerReaderDriver knows about the following properties:\n";
      CameraDriverInterface::PrintPropertyMap();
      return;
    }

    std::string sMethod = GetProperty<std::string>("method", "downsample");
    std::string sFilter = GetProperty<std::string>("filter", "rggb");

    // dc1394bayer_method_t Method;
    if( sMethod == "nearest" ) {
      m_Method = DC1394_BAYER_METHOD_NEAREST;
    } else if( sMethod == "simple" ) {
      m_Method = DC1394_BAYER_METHOD_SIMPLE;
    } else if( sMethod == "bilinear" ) {
      m_Method = DC1394_BAYER_METHOD_BILINEAR;
    } else if( sMethod == "hqlinear" ) {
      m_Method = DC1394_BAYER_METHOD_HQLINEAR;
    } else {
      m_Method = DC1394_BAYER_METHOD_DOWNSAMPLE;
    }
    
    // dc1394color_filter_t Filter;
    if( sFilter == "rggb" ) {
      m_Filter = DC1394_COLOR_FILTER_RGGB;
    } else if( sFilter == "gbrg" ) {
      m_Filter = DC1394_COLOR_FILTER_GBRG;
    } else if( sFilter == "grbg" ) {
      m_Filter = DC1394_COLOR_FILTER_GRBG;
    } else {
      m_Filter = DC1394_COLOR_FILTER_BGGR;
    }

    m_nImgWidth    = input_cam->Width();
    m_nImgHeight   = input_cam->Height();
    m_nNumChannels = input_cam->NumChannels();
    m_nDepth       = GetProperty("depth", 8);
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
