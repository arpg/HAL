#pragma once

#include <thread>

#include <HAL/IMU/IMUDriverInterface.h>

#include <HAL/Encoder/Drivers/PCAN/PCANListener.h>

namespace hal {

class PCANIMUDriver : public IMUDriverInterface
{
public:
    PCANIMUDriver(const std::string& sCom);
    ~PCANIMUDriver();
    void RegisterIMUDataCallback(IMUDriverDataCallback Callback);

private:
    PCANListener&           m_PCANListener;
};

} /* namespace */
