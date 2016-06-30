#pragma once

#include <HAL/Devices/SharedLoad.h>

#include <HAL/IMU/IMUDriverInterface.h>
#include <HAL/Devices/DeviceFactory.h>
#include <HAL/Utils/Uri.h>

namespace hal {

// Generic IMU device
class IMU : public IMUDriverInterface {
 public:
  IMU() {}

  IMU(const std::string& uri) : m_URI(uri) {
    m_IMU = DeviceRegistry<IMUDriverInterface>::Instance().Create(m_URI);
  }

  ~IMU() {
    Clear();
  }

  inline void Reset() {
    Clear();
    m_IMU = DeviceRegistry<IMUDriverInterface>::Instance().Create(m_URI);
    RegisterIMUDataCallback(m_callback);
    RegisterIMUFinishedCallback(m_finished_callback);
  }

  void Clear() {
    m_IMU = nullptr;
  }

  void RegisterIMUDataCallback(IMUDriverDataCallback callback) {
    m_callback = callback;
    if( m_IMU ){
      m_IMU->RegisterIMUDataCallback( callback );
    }else{
      std::cerr << "ERROR: no driver initialized!\n";
    }
    return;
  }

  void RegisterIMUFinishedCallback(IMUDriverFinishedCallback callback) {
    m_finished_callback = callback;
    if( m_IMU ){
      m_IMU->RegisterIMUFinishedCallback( callback );
    }else{
      std::cerr << "ERROR: no driver initialized!\n";
    }
    return;
  }

  std::string GetDeviceProperty(const std::string& sProperty) {
    return m_IMU->GetDeviceProperty(sProperty);
  }

  bool IsRunning() const override {
    return m_IMU->IsRunning();
  }

 protected:
  hal::Uri                                m_URI;
  std::shared_ptr<IMUDriverInterface>     m_IMU;
  IMUDriverDataCallback m_callback;
  IMUDriverFinishedCallback m_finished_callback;
};
} /* namespace hal */
