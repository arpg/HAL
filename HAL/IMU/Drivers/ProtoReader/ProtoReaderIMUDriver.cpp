#include "ProtoReaderIMUDriver.h"

using namespace hal;


/////////////////////////////////////////////////////////////////////////////////////////
ProtoReaderIMUDriver::ProtoReaderIMUDriver(std::string filename)
    : m_reader(pb::Reader::Instance(filename,pb::Msg_Type_IMU)), m_running(false), m_callback(nullptr)
{
}


/////////////////////////////////////////////////////////////////////////////////////////
void ProtoReaderIMUDriver::_ThreadFunc()
{
  while( m_running ) {
    if (std::unique_ptr<pb::ImuMsg> readmsg = m_reader.ReadImuMsg()) {
      m_callback( *readmsg );
    } else {
      break;
    }
  }

  m_running = false;
}

/////////////////////////////////////////////////////////////////////////////////////////
ProtoReaderIMUDriver::~ProtoReaderIMUDriver()
{
    m_running = false;
    m_reader.StopBuffering();
    if( m_callbackThread.joinable() ) {
        m_callbackThread.join();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void ProtoReaderIMUDriver::RegisterIMUDataCallback(IMUDriverDataCallback callback)
{
    m_callback = callback;
    if( !m_callbackThread.joinable() ) {
        // start capture thread
        m_running = true;
        m_callbackThread = std::thread( &ProtoReaderIMUDriver::_ThreadFunc, this );
    }
}
