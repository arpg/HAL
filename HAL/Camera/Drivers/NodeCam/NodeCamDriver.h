#pragma once

#include <thread>

#include <HAL/Utils/Node.h>

#include <HAL/Camera/CameraDriverInterface.h>

namespace hal {

class NodeCamDriver : public CameraDriverInterface
{
public:
    NodeCamDriver(const std::string& sHost);
    ~NodeCamDriver();

private:

private:
    rpg::Node               m_node;
    std::string             m_host;
};

} /* namespace */
