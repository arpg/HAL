#pragma once

#include <thread>

#include <Node/Node.h>

#include <HAL/Encoder/EncoderDriverInterface.h>

namespace hal {

class NodeEncoderDriver : public EncoderDriverInterface
{
public:
    NodeEncoderDriver(const std::string& sLocalNode, const std::string &sRemoteNode, const std::string &sTopicName);
    ~NodeEncoderDriver();
    void RegisterEncoderDataCallback(EncoderDriverDataCallback callback);

private:
    void _ThreadFunc();

private:
    node::node                  m_node;
    std::string                 m_sAddress;
    std::string                 m_local_node;
    std::string                 m_remote_node;
    std::string                 m_topic;
    bool                        m_running;
    std::thread                 m_callbackThread;
    EncoderDriverDataCallback   m_callback;
};

} /* namespace */
