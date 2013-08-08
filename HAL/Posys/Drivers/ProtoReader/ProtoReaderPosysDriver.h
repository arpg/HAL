#pragma once

#include <HAL/Posys/PosysDriverInterface.h>

#include <PbMsgs/Reader.h>

namespace hal {

class ProtoReaderPosysDriver : public PosysDriverInterface
{
public:
    ProtoReaderPosysDriver(std::string filename);
    ~ProtoReaderPosysDriver();
    void RegisterPosysDataCallback(PosysDriverDataCallback callback);

private:
    void _ThreadFunc();

private:
    pb::Reader&                 m_reader;
    bool                        m_running;
    std::thread                 m_callbackThread;
    PosysDriverDataCallback     m_callback;
};

} /* namespace */