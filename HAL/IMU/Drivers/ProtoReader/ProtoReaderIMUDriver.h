#pragma once

#include <HAL/IMU/IMUDriverInterface.h>

#include <HAL/Messages/Reader.h>

namespace hal {

class ProtoReaderIMUDriver : public IMUDriverInterface
{
public:
    ProtoReaderIMUDriver(std::string filename);
    ~ProtoReaderIMUDriver();
    void RegisterIMUDataCallback(IMUDriverDataCallback callback);
    void RegisterIMUFinishedCallback(IMUDriverFinishedCallback callback);

  bool IsRunning() const override {
    return m_running;
  }

private:
    void _ThreadFunc();

private:
    hal::Reader&             m_reader;
    bool                    m_running;
    std::thread             m_callbackThread;
    IMUDriverDataCallback   m_callback;
    IMUDriverFinishedCallback m_IMUFinishedCallback;

};

} /* namespace */
