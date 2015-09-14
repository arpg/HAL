#include "SplitDriver.h"

namespace hal
{

SplitDriver::SplitDriver(
    std::shared_ptr<CameraDriverInterface> input_cam, 
    const Uri& uri
    ) : m_Input(input_cam)
{
  // sat sane default parameters
  CameraDriverInterface::SetDefaultProperties({
      {"roiN", "0+0+WidthxHeight", "Specifies the Nth region of interest."},
      });
  if( !CameraDriverInterface::ParseUriProperties( uri.properties ) ){
    std::cerr << "SplitDriver knows about the following properties:\n";
    CameraDriverInterface::PrintPropertyMap();
    return;
  }

  // construct regions of interests to split
  m_vROIs.clear();
  const ImageRoi default_roi( 0, 0, input_cam->Width(), input_cam->Height() );
  if( !uri.scheme.compare("split") ) {
    while(true) {
      std::stringstream ss;
      ss << "roi" << (m_vROIs.size() + 1);
      const std::string key = ss.str();
      if(!uri.properties.Contains(key)) {
        break;
      }
      m_vROIs.push_back( CameraDriverInterface::GetProperty<ImageRoi>( key, default_roi ) );
    }

    if( m_vROIs.empty() ) {
      unsigned int nImgWidth = input_cam->Width();
      unsigned int nImgHeight = input_cam->Height();
      if( nImgWidth > nImgHeight ) {
        nImgWidth = nImgWidth / 2;
      } else {
        nImgHeight = nImgHeight / 2;
      }

      m_vROIs.push_back( ImageRoi( 0, 0, nImgWidth, nImgHeight ) );
      m_vROIs.push_back( ImageRoi( nImgWidth, 0, nImgWidth, nImgHeight ) );
    }
  }
}

bool SplitDriver::Capture( hal::CameraMsg& vImages )
{
    m_InMsg.Clear();
    if( m_Input->Capture( m_InMsg ) == false ) {
        return false;
    }

    if( m_InMsg.image_size() > 1 ) {
        std::cerr << "error: Split is expecting 1 image but instead got " << m_InMsg.image_size() << "." << std::endl;
        return false;
    }

    const hal::ImageMsg& InImg = m_InMsg.image(0);

    for( unsigned int ii = 0; ii < m_vROIs.size(); ++ii ) {

        hal::ImageMsg* pImg = vImages.add_image();

        pImg->set_format( InImg.format() );
        pImg->set_type( InImg.type() );
        pImg->set_timestamp( InImg.timestamp() );

        const unsigned int nChannels = InImg.format() == hal::PB_LUMINANCE ? 1 : 3;
        unsigned int nDepth = 1;
        if( InImg.type() == hal::PB_SHORT || InImg.type() == hal::PB_UNSIGNED_SHORT ) {
            nDepth = 2;
        } else if( InImg.type() == hal::PB_INT || InImg.type() == hal::PB_UNSIGNED_INT ) {
            nDepth = 4;
        } else if( InImg.type() == hal::PB_FLOAT ) {
            nDepth = 4;
        } else if( InImg.type() == hal::PB_DOUBLE ) {
            nDepth = 8;
        }

        const unsigned int nBytesPerPixel = nChannels * nDepth;

        const ImageRoi& ROI = m_vROIs[ii];
        const unsigned int nBytesPerRow = ROI.w * nBytesPerPixel;

        pImg->set_width( ROI.w );
        pImg->set_height( ROI.h );
        pImg->mutable_data()->resize( ROI.h * nBytesPerRow );

        unsigned char* pS = (unsigned char*)&InImg.data().front();
        unsigned char* pD = (unsigned char*)&pImg->mutable_data()->front();
        for( unsigned int ii = 0; ii < ROI.h; ++ii ) {
            memcpy( pD, pS + (InImg.width()*(ii+ROI.y)) + ROI.x, nBytesPerRow);
            pD = pD + ROI.w;
        }
    }

    return true;
}

size_t SplitDriver::NumChannels() const
{
    return m_vROIs.size();
}

size_t SplitDriver::Width( size_t idx ) const
{
    if( idx < m_vROIs.size() ) {
        return m_vROIs[idx].w;
    }
    return 0;
}

size_t SplitDriver::Height( size_t idx ) const
{
    if( idx < m_vROIs.size() ) {
        return m_vROIs[idx].h;
    }
    return 0;
}


}
