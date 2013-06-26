#pragma once

#include <HAL/IMU/IMUDriverInterface.h>

#include <PbMsgs/Reader.h>

namespace hal {

class ProtoReaderIMUDriver : public IMUDriverInterface
{
public:
    ProtoReaderIMUDriver(std::string filename);
    ~ProtoReaderIMUDriver();
    void RegisterIMUDataCallback(IMUDriverDataCallback callback);
    void RegisterGPSDataCallback(GPSDriverDataCallback /*callback*/) { }

private:
    void _ThreadFunc();

private:
    pb::Reader*             m_reader;
    std::thread             m_callbackThread;
    IMUDriverDataCallback   m_callback;
};

} /* namespace */
