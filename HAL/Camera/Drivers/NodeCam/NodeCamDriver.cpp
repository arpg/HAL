// hack to enable sleep_for (GCC < 4.8)
#define _GLIBCXX_USE_NANOSLEEP

#include "NodeCamDriver.h"

using namespace hal;


/////////////////////////////////////////////////////////////////////////////////////////
NodeCamDriver::NodeCamDriver(const std::string& sHost)
    : m_host(sHost)
{
    if( m_node.Subscribe("Camera", sHost) == false ) {
        std::cerr << "HAL: Error subscribing to remote node." << std::endl;
    }
}


/////////////////////////////////////////////////////////////////////////////////////////
NodeCamDriver::~NodeCamDriver()
{
}
