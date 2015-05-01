#pragma once

#include <deque>
#include <memory>
#include <HAL/IMU/IMUDriverInterface.h>
#include <Eigen/Core>

namespace hal {

class JoinDriver : public IMUDriverInterface {
 public:
  JoinDriver(const std::shared_ptr<IMUDriverInterface>& input);
  virtual ~JoinDriver() {}

  void RegisterIMUDataCallback(IMUDriverDataCallback callback) override {
    callback_ = callback;
  }

  bool IsRunning() const override {
    return input_imu_ && input_imu_->IsRunning();
  }

 private:
  void HandleIMU(hal::ImuMsg& imu);
  inline bool InterpolateAccel(int* gyro_index, Eigen::VectorXd* vec) const;

  std::shared_ptr<IMUDriverInterface> input_imu_;
  IMUDriverDataCallback callback_;

  // The measurements themselves. Back is most recent.
  std::deque<Eigen::VectorXd> accels_, gyros_;

  // Timestamps of measurements. Back is most recent.
  std::deque<double> accel_ts_, gyro_ts_;

  // The timestamp of the last sent message
  double last_sent_time_;
};

}  // end namespace hal
