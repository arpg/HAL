#include "ProtoReaderEncoderDriver.h"

using namespace hal;


/////////////////////////////////////////////////////////////////////////////////////////
ProtoReaderEncoderDriver::ProtoReaderEncoderDriver(std::string filename)
    : m_reader(hal::Reader::Instance(filename,hal::Msg_Type_Encoder)), m_running(false), m_callback(nullptr)
{
}


/////////////////////////////////////////////////////////////////////////////////////////
void ProtoReaderEncoderDriver::_ThreadFunc()
{
    while( m_running ) {
        std::unique_ptr<hal::EncoderMsg> readmsg = m_reader.ReadEncoderMsg();
        if(readmsg) {
            m_callback( *readmsg );
        } else {
            break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
ProtoReaderEncoderDriver::~ProtoReaderEncoderDriver()
{
    m_running = false;
    m_reader.StopBuffering();
    if( m_callbackThread.joinable() ) {
        m_callbackThread.join();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void ProtoReaderEncoderDriver::RegisterEncoderDataCallback(EncoderDriverDataCallback callback)
{
    m_callback = callback;
    m_running = true;
    m_callbackThread = std::thread( &ProtoReaderEncoderDriver::_ThreadFunc, this );
}


