/*
   \file NodeCamDriver.h

 */

#ifndef _NODECAM_H_
#define _NODECAM_H_

#include "RPG/Devices/Camera/CameraDriverInterface.h"

#include "Node.h"

class NodeCamDriver : public CameraDriver
{
    public:
        NodeCamDriver();
        virtual ~NodeCamDriver();
        bool Capture( std::vector<rpg::ImageWrapper>& vImages );
        bool Init();
    private:
        unsigned int        m_nNumNodes;
        rpg::Node           m_Node;
};

#endif
