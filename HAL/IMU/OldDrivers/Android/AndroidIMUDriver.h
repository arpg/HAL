#pragma once

#include <thread>

#include <HAL/IMU/IMUDriverInterface.h>
#include <HAL/Posys/PosysDriverInterface.h>

// Android sensor interface classes
class ASensorEventQueue;
class ASensorManager;
class ASensor;

namespace hal {

class AndroidIMUDriver : public IMUDriverInterface {
 public:
  AndroidIMUDriver();
  virtual ~AndroidIMUDriver();
  void RegisterIMUDataCallback(IMUDriverDataCallback callback) override;

  bool IsRunning() const override {
    return should_run_;
  }

 private:
  void SensorLoop();

  IMUDriverDataCallback imu_callback_;

  bool should_run_;
  std::thread device_thread_;
  ASensorManager* sensor_manager_;
  ASensorEventQueue* event_queue_;
  const ASensor* accelerometer_, *gyroscope_, *magnetometer_;
};

}  // end namespace hal
