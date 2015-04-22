#pragma once

#include <HAL/LIDAR/LIDARDriverInterface.h>

#include <HAL/Messages/Reader.h>

namespace hal {

class ProtoReaderLIDARDriver : public LIDARDriverInterface
{
public:
    ProtoReaderLIDARDriver(std::string filename);
    ~ProtoReaderLIDARDriver();
    void RegisterLIDARDataCallback(LIDARDriverDataCallback callback);

private:
    void _ThreadFunc();

private:
    hal::Reader&             m_reader;
    bool                    m_running;
    std::thread             m_callbackThread;
    LIDARDriverDataCallback m_callback;
};

} /* namespace */
