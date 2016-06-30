#pragma once

#include <HAL/Devices/SharedLoad.h>

#include <HAL/Posys/PosysDriverInterface.h>
#include <HAL/Devices/DeviceFactory.h>
#include <HAL/Utils/Uri.h>

namespace hal {

///
/// Generic Positioning System Device
class Posys : public PosysDriverInterface {
 public:
  Posys() {}

  Posys(const std::string& uri) : m_URI(uri) {
    m_Posys = DeviceRegistry<PosysDriverInterface>::Instance().Create(m_URI);
  }

  ~Posys() {
    Clear();
  }

  void Clear() {
    m_Posys = nullptr;
  }

  void RegisterPosysDataCallback(PosysDriverDataCallback callback) {
    if (m_Posys) {
      m_Posys->RegisterPosysDataCallback(callback);
    } else {
      std::cerr << "ERROR: no driver initialized!\n";
    }
  }

  void RegisterPosysFinishedCallback(PosysDriverFinishedCallback callback) {
    if (m_Posys) {
      m_Posys->RegisterPosysFinishedCallback(callback);
    } else {
      std::cerr << "ERROR: no driver initialized!\n";
    }
  }

  std::string GetDeviceProperty(const std::string& sProperty) {
    return m_Posys->GetDeviceProperty(sProperty);
  }

  bool IsRunning() const override {
    return m_Posys && m_Posys->IsRunning();
  }

 protected:
  hal::Uri                                m_URI;
  std::shared_ptr<PosysDriverInterface>   m_Posys;

};

} /* namespace */
