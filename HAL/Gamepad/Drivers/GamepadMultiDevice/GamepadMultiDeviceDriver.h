#pragma once

#include <thread>
#include <vector>
#include <atomic>
#include <memory>
#include <HAL/Gamepad/GamepadDriverInterface.h>

#include "GSDK/Gamepad.h"
#include "GSDK/EventDispatcher.h"

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
  static bool _OnButtonDown(void *sender, const char *eventID, void *eventData,
                            void *);
  static bool _OnButtonUp(void *sender, const char *eventID, void *eventData,
                          void *);
  static bool _OnAxisMoved(void *sender, const char *eventID, void *eventData,
                           void *);
  static bool _OnDeviceAttached(void *sender, const char *eventID,
                                void *eventData, void *context);
  static bool _OnDeviceRemoved(void *sender, const char *eventID,
                               void *eventData, void *context);
  void _ThreadFunc();
  void _ThreadUpdateGamepad();
  bool _Init();
  volatile bool mShouldRun;
  std::thread mDeviceThread;
  std::thread mDeviceUpdateThread;
  std::string m_sPortName;
  static hal::GamepadMsg pbGamepadMsg;
  static hal::VectorMsg *pbVecButtonsMsg;
  static hal::VectorMsg *pbVecAxesMsg;

  // for debug purposes
  static bool _verbose;
};

} /* namespace */
