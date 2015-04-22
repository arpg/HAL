#include "./JoinDriver.h"

#include <functional>
#include <HAL/Messages/Matrix.h>

namespace hal {

using std::placeholders::_1;
static const size_t kBufferSize = 5;

JoinDriver::JoinDriver(const std::shared_ptr<IMUDriverInterface>& input)
    : input_imu_(input), last_sent_time_(0.0) {
  input_imu_->RegisterIMUDataCallback(
      std::bind(&JoinDriver::HandleIMU, this, _1));
}

inline void PopPushVectorMsg(const hal::VectorMsg& msg,
                             double time,
                             std::deque<Eigen::VectorXd>* data,
                             std::deque<double>* times) {
  while (data->size() >= kBufferSize) {
    data->pop_front();
    times->pop_front();
  }

  Eigen::VectorXd vec;
  ReadVector(msg, &vec);
  data->push_back(vec);
  times->push_back(time);
}

bool JoinDriver::InterpolateAccel(int* gyro_index, Eigen::VectorXd* vec) const {
  // Try to interpolate at the oldest gyro message possible
  for (size_t gi = 0; gi < gyro_ts_.size(); ++gi) {
    const double& time = gyro_ts_[gi];
    if (time <= last_sent_time_) continue;

    bool found_eq = false;
    int greater_idx = -1, lesser_idx = -1;

    // From the most recent of accel time
    for (int i = accel_ts_.size() - 1; i >= 0; --i) {
      double accel_time = accel_ts_[i];
      if (accel_time > time) {
        greater_idx = i;
      } else if (accel_time < time) {
        lesser_idx = i;
      } else if (accel_time == time) {
        greater_idx = i;
        found_eq = true;
        break;
      }
    }

    if (found_eq) {
      *vec = accels_[greater_idx];
      *gyro_index = gi;
      return true;
    } else if (greater_idx != -1 && lesser_idx != -1) {
      double fraction = ((time - accel_ts_[lesser_idx]) /
                         (accel_ts_[greater_idx] - accel_ts_[lesser_idx]));
      *vec = accels_[lesser_idx] + fraction * (accels_[greater_idx] -
                                               accels_[lesser_idx]);
      *gyro_index = gi;
      return true;
    }
  }
  return false;
}

/** @todo Unit test me! */
/** @todo Add magnetometer handling! Right now we drop those messages! */
void JoinDriver::HandleIMU(hal::ImuMsg& imu) {
  if (!callback_) return;

  if (imu.has_gyro() ^ imu.has_accel()) {  // If we've only got one of them
    if (imu.has_gyro()) {
      PopPushVectorMsg(imu.gyro(), imu.device_time(),
                       &gyros_, &gyro_ts_);
    } else if (imu.has_accel()) {
      PopPushVectorMsg(imu.accel(), imu.device_time(),
                       &accels_, &accel_ts_);
    } else { abort(); }  // Not sure how we'd get here, but just to be safe.

    Eigen::VectorXd new_accel(3);
    int gyro_index;
    if (InterpolateAccel(&gyro_index, &new_accel)) {
      hal::ImuMsg interpolated_msg;
      interpolated_msg.set_device_time(gyro_ts_[gyro_index]);
      WriteVector(new_accel, interpolated_msg.mutable_accel());
      WriteVector(gyros_[gyro_index], interpolated_msg.mutable_gyro());

      callback_(interpolated_msg);
      last_sent_time_ = interpolated_msg.device_time();
    }
  } else if (imu.has_gyro() && imu.has_accel()) {
    callback_(imu);
    last_sent_time_ = imu.device_time();
  }
}

}  // end namespace hal
