#pragma once

#include <thread>

#include <node/Node.h>

#include <HAL/IMU/IMUDriverInterface.h>

namespace hal {

class NodeIMUDriver : public IMUDriverInterface
{
public:
    NodeIMUDriver(const std::string& sHost);
    ~NodeIMUDriver();
    void RegisterIMUDataCallback(IMUDriverDataCallback callback);

  bool IsRunning() const override {
    return m_running;
  }

private:
    void _ThreadFunc();

private:
    node::node              m_node;
    std::string             m_host;
    bool                    m_running;
    std::thread             m_callbackThread;
    IMUDriverDataCallback   m_callback;
};

} /* namespace */
