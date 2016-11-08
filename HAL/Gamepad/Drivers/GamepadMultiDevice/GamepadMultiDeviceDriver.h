#pragma once
#include <thread>
#include <vector>
#include <atomic>
#include <memory>
#include <thread>
#include <mutex>
#include <HAL/Gamepad/GamepadDriverInterface.h>
#include "GSDK/Gamepad.h"

namespace hal {

class GamepadMultiDeviceDriver : public GamepadDriverInterface {
 public:
  GamepadMultiDeviceDriver();
  ~GamepadMultiDeviceDriver();
  void RegisterGamepadDataCallback(GamepadDriverDataCallback callback);
  bool IsRunning() const override { return mShouldRun; }
  //  void EnableVerbose(bool enable) const override { _verbose = enable; }

 private:
  static GamepadDriverDataCallback mGamepadCallback;
  static void _OnButtonDown(struct Gamepad_device* device, unsigned int buttonID, double timestamp, void* context);
  static void _OnButtonUp(struct Gamepad_device* device, unsigned int buttonID, double timestamp, void* context);
  static void _OnAxisMoved(struct Gamepad_device* device, unsigned int axisID, float value, float lastValue, double timestamp, void * context);
  static void _OnDeviceAttached(struct Gamepad_device* device, void* context);
  static void _OnDeviceRemoved(struct Gamepad_device* device, void* context);
  void _GamepadThread();
  bool _Init();
  std::mutex should_run_mutex;
  volatile bool mShouldRun;
  std::thread _GamepadThreadHandle;
  std::string m_sPortName;
  static hal::GamepadMsg pbGamepadMsg;
  static hal::VectorMsg *pbVecButtonsMsg;
  static hal::VectorMsg *pbVecAxesMsg;

  // for debug purposes
  static bool _verbose;
};

} /* namespace */
