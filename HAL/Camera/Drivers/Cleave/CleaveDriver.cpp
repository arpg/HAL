#include "CleaveDriver.h"



namespace hal {

  CleaveDriver::CleaveDriver(std::shared_ptr<CameraDriverInterface> Input, int m_maxChannel, int m_minChannel )
    : maxChannel(m_maxChannel), minChannel(m_minChannel), inputCamera(Input)
  {

    Start();
  }

  CleaveDriver::~CleaveDriver()
  {
    Stop();

  }


  size_t CleaveDriver::NumChannels() const
  {
    return maxChannel-minChannel+1;
  }

  size_t CleaveDriver::Width( size_t idx) const
  {
    return widths[idx];
  }

  size_t CleaveDriver::Height( size_t idx) const
  {
    return heights[idx];
  }

  void CleaveDriver::Start()
  {
    //to get the sizes, capture one round of imagery and throw it away
    std::cout << "Cleave: Passing through channels " << minChannel << " to " << maxChannel << std::endl;
    m_InMsg.Clear();
    widths.clear();
    heights.clear();
    if( inputCamera->Capture( m_InMsg ) == false ) {
        return;
    }

    for( unsigned int ii = minChannel; ii <= maxChannel; ++ii ) {
      const hal::ImageMsg& InImg = m_InMsg.image(ii);
      widths.push_back(InImg.width());
      heights.push_back(InImg.height());
    }
  }

  void CleaveDriver::Stop()
  {
    std::cout << "Stopping Cleave driver" << std::endl;

    std::cout << "Stop of Cleave driver complete" << std::endl;
  }


  bool CleaveDriver::Capture( hal::CameraMsg& vImages )
  {
    vImages.Clear();
    m_InMsg.Clear();

    if( inputCamera->Capture( m_InMsg ) == false ) {
        return false;
    }

    vImages.set_device_time(m_InMsg.device_time());
    vImages.set_system_time(m_InMsg.system_time());

    for( unsigned int ii = minChannel; ii <= maxChannel; ++ii ) {
      const hal::ImageMsg& InImg = m_InMsg.image(ii);

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


        pImg->set_width( InImg.width());
        pImg->set_height( InImg.height() );
        pImg->mutable_data()->resize( nBytesPerPixel*InImg.width()*InImg.height() );

        unsigned char* pS = (unsigned char*)&InImg.data().front();
        unsigned char* pD = (unsigned char*)&pImg->mutable_data()->front();
  memcpy(pD, pS, nBytesPerPixel*InImg.width()*InImg.height());
        }

    return true;
  }

} //namespace
