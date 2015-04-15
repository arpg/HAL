#pragma once

#include <HAL/Encoder/EncoderDriverInterface.h>

#include <Messages/Reader.h>

namespace hal {

class ProtoReaderEncoderDriver : public EncoderDriverInterface
{
public:
    ProtoReaderEncoderDriver(std::string filename);
    ~ProtoReaderEncoderDriver();
    void RegisterEncoderDataCallback(EncoderDriverDataCallback callback);

private:
    void _ThreadFunc();

private:
    pb::Reader&                 m_reader;
    bool                        m_running;
    std::thread                 m_callbackThread;
    EncoderDriverDataCallback   m_callback;
};

} /* namespace */
