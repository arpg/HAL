#pragma once

#include <functional>
#include <HAL/Devices/DriverInterface.h>
#include <HAL/Gamepad.pb.h>

namespace hal {

typedef std::function<void (hal::GamepadMsg&)> GamepadDriverDataCallback;

/// Generic Gamepad driver interface
///
class GamepadDriverInterface : public DriverInterface {
 public:
	virtual ~GamepadDriverInterface() {}
	virtual void RegisterGamepadDataCallback(GamepadDriverDataCallback callback) = 0;
	virtual bool IsRunning() const = 0;
//	virtual void EnableVerbose(bool enable) const = 0;
};

} /* namespace */
