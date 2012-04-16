/*
   \file NodeCamDriver.h

 */

#ifndef _NODECAM_H_
#define _NODECAM_H_

#include "RPG/Devices/Camera/CameraDriverInterface.h"

#include <zmq.hpp>

class NodeCamDriver : public CameraDriver
{
    public:
        NodeCamDriver();
        virtual ~NodeCamDriver();
        bool Capture( std::vector<rpg::ImageWrapper>& vImages );
        bool Init();
    private:
        zmq::context_t*             m_pContext;     // ZMQ context
        zmq::socket_t*              m_pSocket;		// ZMQ socket
		std::string					m_sHost;		// Host we are connecting to
};

#endif
