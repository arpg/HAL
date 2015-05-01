#include "ProtoReaderLIDARDriver.h"

using namespace hal;


/////////////////////////////////////////////////////////////////////////////////////////
ProtoReaderLIDARDriver::ProtoReaderLIDARDriver(std::string filename)
    : m_reader(hal::Reader::Instance(filename, hal::Msg_Type_LIDAR)), m_running(false), m_callback(nullptr)
{
}


/////////////////////////////////////////////////////////////////////////////////////////
void ProtoReaderLIDARDriver::_ThreadFunc()
{
    while( m_running ) {
        std::unique_ptr<hal::LidarMsg> readmsg = m_reader.ReadLidarMsg();
        if(readmsg) {
            m_callback( *readmsg );
        } else {
            break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
ProtoReaderLIDARDriver::~ProtoReaderLIDARDriver()
{
    m_running = false;
    m_reader.StopBuffering();
    if( m_callbackThread.joinable() ) {
        m_callbackThread.join();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void ProtoReaderLIDARDriver::RegisterLIDARDataCallback(LIDARDriverDataCallback callback)
{
    m_callback = callback;
    m_running = true;
    m_callbackThread = std::thread( &ProtoReaderLIDARDriver::_ThreadFunc, this );
}


