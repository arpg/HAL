// hack to enable sleep_for (GCC < 4.8)
#ifndef _GLIBCXX_USE_NANOSLEEP
#define _GLIBCXX_USE_NANOSLEEP
#endif  // _GLIBCXX_USE_NANOSLEEP

#include "NodeEncoderDriver.h"
#include <HAL/Devices/DeviceException.h>

using namespace hal;


/////////////////////////////////////////////////////////////////////////////////////////
NodeEncoderDriver::NodeEncoderDriver(const std::string& sLocalNode,
                                     const std::string& sRemoteNode,
                                     const std::string& sTopicName)
    : m_local_node(sLocalNode), m_remote_node(sRemoteNode), m_topic(sTopicName),
      m_sAddress(sRemoteNode + "/" + sTopicName),
      m_running(false), m_callback(nullptr)
{
  m_node.set_verbosity(2);
  if(m_node.init(m_local_node))
  {
    if(m_node.subscribe(m_sAddress))
    {
      // this initiates the number of channels and image dimensions
      // hal::CameraMsg vImages;
      // Capture(vImages);
    }
    else
    {
      throw DeviceException("NodeEncoderDriver could not subscribe to \"" +
                            m_sAddress + "\"");
    }
  }
  else
  {
    throw DeviceException("NodeEncoderDriver could not initiatate node \"" +
                          m_local_node + "\"");
  }

}


/////////////////////////////////////////////////////////////////////////////////////////
void NodeEncoderDriver::_ThreadFunc()
{
    hal::EncoderMsg pbMsg;
    while( m_running ) {
        pbMsg.Clear();

        bool ok = m_node.receive(m_sAddress, pbMsg);
        while (!ok) {
          ok = m_node.receive(m_sAddress, pbMsg);
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
