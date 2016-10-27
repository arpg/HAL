#pragma once
#include <memory>
#include <HAL/IMU/IMUDriverInterface.h>
#include <iostream>
#include <vector>
#include <unistd.h>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>
#include <functional>
#include <chrono>
#include "gladiator.h"

namespace hal {

class GladiatorDriver : public IMUDriverInterface
{
public:
  GladiatorDriver(const char* mPort);
  ~GladiatorDriver();
  bool Init(const char* mPort);
  void RegisterIMUDataCallback(IMUDriverDataCallback callback);
  volatile bool m_bShouldRun;
  bool IsRunning() const override {
    return false;
  }
  std::thread             m_DeviceThread;
  void service();
private:
  GladiatorIMU *m_dev;
  
  IMUDriverDataCallback m_IMUCallback;
};

} /* namespace */
