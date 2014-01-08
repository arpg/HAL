#pragma once

#include <HAL/Camera/CameraDriverInterface.h>
#include "Node2CamMessage.pb.h"
#include <string>

#pragma GCC system_header
#include <Node/Node.h>

namespace hal {

class Node2CamDriver : public CameraDriverInterface
{
    public:
        Node2CamDriver(std::string& sDeviceName,
                       std::string& sHostName
                      );

        virtual ~Node2CamDriver();

        bool Capture( pb::CameraMsg& vImages );
        std::shared_ptr<CameraDriverInterface> GetInputDevice() { return std::shared_ptr<CameraDriverInterface>(); }

        std::string GetDeviceProperty(const std::string& sProperty);

        bool InitNode();
        bool RegisterInHost();

        size_t NumChannels() const;
        size_t Width( size_t /*idx*/ = 0 ) const;
        size_t Height( size_t /*idx*/ = 0 ) const;

private:
        unsigned int        m_nImgHeight;
        unsigned int        m_nImgWidth;
        unsigned int        m_nChannels;
        unsigned int        m_nNumNodes;

        hal::node           m_Node;
        std::string         m_sHostName;
        std::string         m_sDeviceName;
        int                 m_nTimeStep;
};

}
