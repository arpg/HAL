#pragma once

#include <vector>
#include <memory>
#include <string>
#include <chrono>

#include <HAL/Camera/CameraDriverInterface.h>
#include <node/Node.h>

namespace hal {

class NodeCamDriver : public CameraDriverInterface
{
public:
    NodeCamDriver(const std::string& sLocalNode, const std::string& sRemoteNode,
                  const std::string& sTopicName, double timeout);
    ~NodeCamDriver();

    bool Capture( hal::CameraMsg& vImages );
    std::shared_ptr<CameraDriverInterface> GetInputDevice() {
        return std::shared_ptr<CameraDriverInterface>();
    }

    std::string GetDeviceProperty(const std::string& sProperty);

    size_t NumChannels() const;
    size_t Width( size_t idx = 0 ) const;
    size_t Height( size_t idx = 0 ) const;

private:

    bool InitNode();

    // Returns ms from epoch
    std::chrono::milliseconds Now() const;

    // Returns difference in seconds
    double EllapsedSeconds(const std::chrono::milliseconds& begin,
        const std::chrono::milliseconds& end) const;

private:
    node::node              m_Node;
    std::string             m_sLocalNode;
    std::string             m_sRemoteNode;
    std::string             m_sTopicName;
    double                  m_dTimeout; // seconds
    std::string             m_sAddress; // RemoteNode/Topic
    size_t                  m_nChannels;
    std::vector<size_t>     m_nImgWidth;
    std::vector<size_t>     m_nImgHeight;
};

} /* namespace */

