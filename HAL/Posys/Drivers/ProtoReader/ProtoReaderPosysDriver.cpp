#include "ProtoReaderPosysDriver.h"

using namespace hal;


/////////////////////////////////////////////////////////////////////////////////////////
ProtoReaderPosysDriver::ProtoReaderPosysDriver(std::string filename)
    : m_reader(hal::Reader::Instance(filename,hal::Msg_Type_Posys)), m_running(false), m_callback(nullptr)
{
}


/////////////////////////////////////////////////////////////////////////////////////////
void ProtoReaderPosysDriver::_ThreadFunc()
{
    while( m_running ) {
        std::unique_ptr<hal::PoseMsg> readmsg = m_reader.ReadPoseMsg();
        if(readmsg) {
            m_callback( *readmsg );
        } else {
            break;
        }
    }
    m_running = false;
}

/////////////////////////////////////////////////////////////////////////////////////////
ProtoReaderPosysDriver::~ProtoReaderPosysDriver()
{
    m_running = false;
    m_reader.StopBuffering();
    m_callbackThread.join();
}

/////////////////////////////////////////////////////////////////////////////////////////
void ProtoReaderPosysDriver::RegisterPosysDataCallback(PosysDriverDataCallback callback)
{
    m_callback = callback;
    m_running = true;
    m_callbackThread = std::thread( &ProtoReaderPosysDriver::_ThreadFunc, this );
}
