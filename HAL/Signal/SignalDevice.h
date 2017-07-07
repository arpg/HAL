#pragma once

#include <HAL/Devices/SharedLoad.h>

#include <HAL/Signal/SignalDriverInterface.h>
#include <HAL/Devices/DeviceFactory.h>
#include <HAL/Utils/Uri.h>

namespace hal {

// Generic Signal device
class Signal : public SignalDriverInterface {
 public:
  Signal() {}

  Signal(const std::string& uri) : m_URI(uri) {
    m_Signal = DeviceRegistry<SignalDriverInterface>::Instance().Create(m_URI);
  }

  ~Signal() {
    Clear();
  }

  inline void Reset() {
    Clear();
    m_Signal = DeviceRegistry<SignalDriverInterface>::Instance().Create(m_URI);
    RegisterSignalDataCallback(m_callback);
    RegisterSignalFinishedCallback(m_finished_callback);
  }

  void Clear() {
    m_Signal = nullptr;
  }

  void RegisterSignalDataCallback(SignalDriverDataCallback callback) {
    m_callback = callback;
    if( m_Signal ){
      m_Signal->RegisterSignalDataCallback( callback );
    }else{
      std::cerr << "ERROR: no driver initialized!\n";
    }
    return;
  }

  void RegisterSignalFinishedCallback(SignalDriverFinishedCallback callback) {
    m_finished_callback = callback;
    if( m_Signal ){
      m_Signal->RegisterSignalFinishedCallback( callback );
    }else{
      std::cerr << "ERROR: no driver initialized!\n";
    }
    return;
  }

  std::string GetDeviceProperty(const std::string& sProperty) {
    return m_Signal->GetDeviceProperty(sProperty);
  }

  bool IsRunning() const override {
    return m_Signal->IsRunning();
  }

 protected:
  hal::Uri m_URI;
  std::shared_ptr<SignalDriverInterface> m_Signal;
  SignalDriverDataCallback m_callback;
  SignalDriverFinishedCallback m_finished_callback;
};
} /* namespace hal */
