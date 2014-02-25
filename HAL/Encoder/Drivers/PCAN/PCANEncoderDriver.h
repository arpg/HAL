#pragma once

#include <thread>

#include <HAL/Encoder/EncoderDriverInterface.h>

#include <HAL/Encoder/Drivers/PCAN/PCANListener.h>

namespace hal {

class PCANEncoderDriver : public EncoderDriverInterface
{
public:
    PCANEncoderDriver(const std::string& sCom);
    ~PCANEncoderDriver();
    void RegisterEncoderDataCallback(EncoderDriverDataCallback Callback);

private:
    PCANListener&               m_PCANListener;

};

} /* namespace */
