#pragma once

#include <HAL/Posys/PosysDriverInterface.h>

#include <HAL/Messages/Reader.h>

namespace hal {

class ProtoReaderPosysDriver : public PosysDriverInterface
{
public:
    ProtoReaderPosysDriver(std::string filename);
    ~ProtoReaderPosysDriver();
    void RegisterPosysDataCallback(PosysDriverDataCallback callback);
  bool IsRunning() const override {
    return m_running;
  }

private:
    void _ThreadFunc();

private:
    hal::Reader&                 m_reader;
    bool                        m_running;
    std::thread                 m_callbackThread;
    PosysDriverDataCallback     m_callback;
};

} /* namespace */
