#pragma once

#include <HAL/Encoder/EncoderDriverInterface.h>

#include <HAL/Messages/Reader.h>

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
    hal::Reader&                 m_reader;
    bool                        m_running;
    std::thread                 m_callbackThread;
    EncoderDriverDataCallback   m_callback;
};

} /* namespace */
