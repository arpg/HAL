#pragma once

#include <HAL/IMU/IMUDriverInterface.h>

#include <phidget22.h>

namespace hal {

void CCONV AttachHandler(PhidgetHandle spatial, void *userptr);
void CCONV DetachHandler(PhidgetHandle spatial, void *userptr);
void CCONV ErrorHandler(PhidgetHandle spatial, void *userptr,
    Phidget_ErrorEventCode ErrorCode, const char *unknown);
void CCONV SpatialDataHandler(PhidgetSpatialHandle spatial, void *userptr,
    const double accel[3], const double angular[3], const double mag_field[3],
    double timestamp);

class PhidgetsDriver : public IMUDriverInterface {
 public:
  PhidgetsDriver();
  ~PhidgetsDriver();
  bool Init();
  void RegisterIMUDataCallback(IMUDriverDataCallback callback);

  void _AttachHandler(PhidgetHandle spatial);
  void _DetachHandler(PhidgetHandle spatial);
  void _ErrorHandler(PhidgetHandle /*spatial*/,
    Phidget_ErrorEventCode ErrorCode, const char *unknown);
  void _SpatialDataHandler(PhidgetSpatialHandle /*spatial*/,
    const double accel[3], const double angular[3], const double mag_field[3],
    double timestamp);

  bool IsRunning() const override {
    return false;
  }

 private:
  PhidgetSpatialHandle m_hSpatial;
  IMUDriverDataCallback m_ImuCallback;
};

} // namespace hal
