#pragma once

#include <thread>

#include <HAL/Encoder/EncoderDriverInterface.h>

#include <HAL/IMU/Drivers/Ninja/FtdiListener.h>

namespace hal {

class NinjaEncoderDriver : public EncoderDriverInterface
{
public:
    NinjaEncoderDriver(const std::string& sCom);
    ~NinjaEncoderDriver();
    void RegisterEncoderDataCallback(EncoderDriverDataCallback Callback);

private:
    FtdiListener&               m_FtdiListener;

};

} /* namespace */
