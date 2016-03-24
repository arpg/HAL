#pragma once

#include <thread>
#include <vector>
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
  double GetAxisValue(int id);
  bool IsButtonPressed(int id);
  bool EnVerbose(bool state);

 private:
  static GamepadDriverDataCallback mGamepadCallback;
  static void CallbackFunc();
  static bool _OnButtonDown(void *sender, const char *eventID, void *eventData,
                            void *context);
  static bool _OnButtonUp(void *sender, const char *eventID, void *eventData,
                          void *context);
  static bool _OnAxisMoved(void *sender, const char *eventID, void *eventData,
                           void *context);
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
  static bool _verbose;
  std::vector<double> m_vAxes;
  std::vector<int> m_vButtonStates;
  std::string m_sPortName;

};

} /* namespace */
