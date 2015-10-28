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
  bool IsRunning() const override {
    return m_FtdiListener.IsRunning();
  }

private:
    FtdiListener&           m_FtdiListener;
};

} /* namespace */
