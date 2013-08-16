#pragma once

#include <thread>

#include <HAL/IMU/IMUDriverInterface.h>

#include "FtdiListener.h"

namespace hal {

class NinjaIMUDriver : public IMUDriverInterface
{
public:
    NinjaIMUDriver(const std::string& sCom);
    ~NinjaIMUDriver();
    void RegisterIMUDataCallback(IMUDriverDataCallback Callback);

private:
    FtdiListener&           m_FtdiListener;

};

} /* namespace */
