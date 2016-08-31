#pragma once

#include <HAL/Devices/SharedLoad.h>

#include <HAL/Gamepad/GamepadDriverInterface.h>
#include <HAL/Devices/DeviceFactory.h>
#include <HAL/Utils/Uri.h>

namespace hal {

// Generic Gamepad device
class Gamepad : public GamepadDriverInterface {
 public:
	Gamepad() {}

	Gamepad(const std::string& uri) : m_URI(uri) {
		m_Gamepad = DeviceRegistry<GamepadDriverInterface>::Instance().Create(m_URI);
  }

	~Gamepad() {
    Clear();
  }

  inline void Reset() {
    Clear();
		m_Gamepad = DeviceRegistry<GamepadDriverInterface>::Instance().Create(m_URI);
		RegisterGamepadDataCallback(m_callback);
  }

  void Clear() {
		m_Gamepad = nullptr;
  }

  void RegisterGamepadDataCallback(GamepadDriverDataCallback callback) override{
    m_callback = callback;
		if( m_Gamepad ){
			m_Gamepad->RegisterGamepadDataCallback( callback );
    }else{
      std::cerr << "ERROR: no driver initialized!\n";
    }
    return;
  }

	std::string GetDeviceProperty(const std::string& sProperty) override{
		return m_Gamepad->GetDeviceProperty(sProperty);
  }

  bool IsRunning() const override {
		return m_Gamepad->IsRunning();
  }

 protected:
	hal::Uri																		m_URI;
	std::shared_ptr<GamepadDriverInterface>     m_Gamepad;
	GamepadDriverDataCallback m_callback;
};
} /* namespace hal */
