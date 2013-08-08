// hack to enable sleep_for (GCC < 4.8)
#define _GLIBCXX_USE_NANOSLEEP

#include "NodeIMUDriver.h"

using namespace hal;


/////////////////////////////////////////////////////////////////////////////////////////
NodeIMUDriver::NodeIMUDriver(const std::string& sHost)
    : m_running(false), m_callback(nullptr)
{
    m_Node.Subscribe("IMU", sHost);
}


/////////////////////////////////////////////////////////////////////////////////////////
void NodeIMUDriver::_ThreadFunc()
{
    pb::ImuMsg pbMsg;
    while( m_running ) {
        pbMsg.Clear();
        if( m_Node.Read("IMU", pbMsg) == false ) {
            std::cerr << "HAL: Error reading Node publisher." << std::endl;
            continue;
        }
        m_callback(pbMsg);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
NodeIMUDriver::~NodeIMUDriver()
{
    m_running = false;
    m_callbackThread.join();
}

/////////////////////////////////////////////////////////////////////////////////////////
void NodeIMUDriver::RegisterIMUDataCallback(IMUDriverDataCallback callback)
{
    m_callback = callback;
    m_running = true;
    m_callbackThread = std::thread( &NodeIMUDriver::_ThreadFunc, this );
}
