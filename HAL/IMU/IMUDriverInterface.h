#pragma once

#include <functional>
#include <HAL/Devices/DriverInterface.h>
#include <HAL/Imu.pb.h>

namespace hal {

typedef std::function<void (hal::ImuMsg&)> IMUDriverDataCallback;
typedef std::function<void ()> IMUDriverFinishedCallback;

/// Generic IMU driver interface
///
class IMUDriverInterface : public DriverInterface {
 public:
  virtual ~IMUDriverInterface() {}
  virtual void RegisterIMUDataCallback(IMUDriverDataCallback callback) = 0;
  virtual void RegisterIMUFinishedCallback(IMUDriverFinishedCallback ) {}
  virtual bool IsRunning() const = 0;
};

} /* namespace */
