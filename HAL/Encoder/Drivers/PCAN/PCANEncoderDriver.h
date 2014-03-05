#pragma once

#include <thread>

#include <HAL/Encoder/EncoderDriverInterface.h>

#include <HAL/Encoder/Drivers/PCAN/PCANListener.h>
class PCANListener;
namespace hal {

class PCANEncoderDriver : public EncoderDriverInterface
{
public:
    PCANEncoderDriver(unsigned long int baudrate, const std::string& sCom);
    ~PCANEncoderDriver();
    void RegisterEncoderDataCallback(EncoderDriverDataCallback Callback);

private:
    PCANListener&               m_PCANListener;

};

} /* namespace */
