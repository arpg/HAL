#pragma once

#include <thread>

#include <HAL/Utils/Node.h>

#include <HAL/Encoder/EncoderDriverInterface.h>

namespace hal {

class NodeEncoderDriver : public EncoderDriverInterface
{
public:
    NodeEncoderDriver(const std::string& sHost);
    ~NodeEncoderDriver();
    void RegisterEncoderDataCallback(EncoderDriverDataCallback callback);

private:
    void _ThreadFunc();

private:
    rpg::Node                   m_node;
    std::string                 m_host;
    bool                        m_running;
    std::thread                 m_callbackThread;
    EncoderDriverDataCallback   m_callback;
};

} /* namespace */
