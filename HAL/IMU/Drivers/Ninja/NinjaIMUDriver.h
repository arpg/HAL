#pragma once

#include <thread>

#include <HAL/IMU/IMUDriverInterface.h>

#include "FtdiDriver.h"

namespace hal {

class NinjaIMUDriver : public IMUDriverInterface
{
public:
    NinjaIMUDriver(const std::string& sCom);
    ~NinjaIMUDriver();
    void RegisterIMUDataCallback(IMUDriverDataCallback Callback);

private:
    void _ThreadFunc();

private:
    IMUDriverDataCallback   m_Callback;
    FtdiDriver              m_FtdiDriver;
    bool                    m_Running;
    std::thread             m_CallbackThread;

};

} /* namespace */
