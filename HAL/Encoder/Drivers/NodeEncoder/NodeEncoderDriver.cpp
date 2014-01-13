// hack to enable sleep_for (GCC < 4.8)
#ifndef _GLIBCXX_USE_NANOSLEEP
#define _GLIBCXX_USE_NANOSLEEP
#endif  // _GLIBCXX_USE_NANOSLEEP

#include "NodeEncoderDriver.h"

using namespace hal;


/////////////////////////////////////////////////////////////////////////////////////////
NodeEncoderDriver::NodeEncoderDriver(const std::string& sHost)
    : m_host(sHost), m_running(false), m_callback(nullptr)
{
    if( m_node.Subscribe("Encoder", sHost) == false ) {
        std::cerr << "HAL: Error subscribing to remote node." << std::endl;
    }
}


/////////////////////////////////////////////////////////////////////////////////////////
void NodeEncoderDriver::_ThreadFunc()
{
    pb::EncoderMsg pbMsg;
    while( m_running ) {
        pbMsg.Clear();
        if( m_node.ReadBlocking("Encoder", pbMsg) == false ) {
            std::cerr << "HAL: Error reading node publisher." << std::endl;
            continue;
        }
        m_callback(pbMsg);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
NodeEncoderDriver::~NodeEncoderDriver()
{
    m_running = false;
    if( m_callbackThread.joinable() ) {
        m_callbackThread.join();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void NodeEncoderDriver::RegisterEncoderDataCallback(EncoderDriverDataCallback callback)
{
    m_callback = callback;
    m_running = true;
    m_callbackThread = std::thread( &NodeEncoderDriver::_ThreadFunc, this );
}
