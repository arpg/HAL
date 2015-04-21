#pragma once

#include <functional>
#include <HAL/Devices/DriverInterface.h>
#include <HAL/Messages/Imu.pb.h>

namespace hal {

typedef std::function<void (pb::ImuMsg&)> IMUDriverDataCallback;

/// Generic IMU driver interface
///
class IMUDriverInterface : public DriverInterface {
 public:
  virtual ~IMUDriverInterface() {}
  virtual void RegisterIMUDataCallback(IMUDriverDataCallback callback) = 0;
  virtual bool IsRunning() const = 0;
};

} /* namespace */
