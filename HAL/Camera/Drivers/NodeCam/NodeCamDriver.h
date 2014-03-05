#pragma once

#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Utils/Uri.h>
#include "NodeCamMessage.pb.h"
#include <string>

#pragma GCC system_header
#include <Node/Node.h>

namespace hal {

class NodeCamDriver : public CameraDriverInterface
{
    public:
        NodeCamDriver(const hal::Uri &uri);

        virtual ~NodeCamDriver();

        bool Capture( pb::CameraMsg& vImages );
        std::shared_ptr<CameraDriverInterface> GetInputDevice() {
          return std::shared_ptr<CameraDriverInterface>();
        }

        std::string GetDeviceProperty(const std::string& sProperty);

        bool InitNode();
        bool RegisterInHost(const Uri& uri);

        size_t NumChannels() const;
        size_t Width( size_t /*idx*/ = 0 ) const;
        size_t Height( size_t /*idx*/ = 0 ) const;

private:
        unsigned int        m_nImgHeight;
        unsigned int        m_nImgWidth;
        unsigned int        m_nChannels;

        hal::node           m_Node;
        std::string         m_sSimNodeName;
        std::string         m_sDeviceName;
        std::string         m_sDeviceId;
        std::string         m_sTopic;
        int                 m_nTimeStep;
};

}
