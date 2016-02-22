#include <iostream>

#include <HAL/Devices/DeviceException.h>
#include <HAL/Utils/TicToc.h>

#include "GamepadMultiDeviceDriver.h"

using namespace hal;

GamepadDriverDataCallback GamepadMultiDeviceDriver::mGamepadCallback = nullptr;

const int DEFAULT_PACKET_TIMEOUT_MS = 1000;
const bool _verbose = false;
#define _LINUX

#if defined(_LINUX) || defined(_QNX) || defined(_OSX)
#include <time.h>
void Sleep(unsigned int time) {
  struct timespec t, r;
  t.tv_sec = time / 1000;
  t.tv_nsec = (time % 1000) * 1000000;
  while (nanosleep(&t, &r) == -1) t = r;
}
#endif

///////////////////////////////////////////////////////////////////////////////
GamepadMultiDeviceDriver::GamepadMultiDeviceDriver() : mShouldRun(false) {
}

///////////////////////////////////////////////////////////////////////////////
GamepadMultiDeviceDriver::~GamepadMultiDeviceDriver() {
  mShouldRun = false;
  mDeviceThread.join();
}

///////////////////////////////////////////////////////////////////////////////
void GamepadMultiDeviceDriver::CallbackFunc() {}

///////////////////////////////////////////////////////////////////////////////
bool GamepadMultiDeviceDriver::_Init() {
  // Device communication parameters

  mShouldRun = true;
  mDeviceThread = std::thread(
      std::bind(&GamepadMultiDeviceDriver::_ThreadCaptureFunc, this));

  return true;
}

///////////////////////////////////////////////////////////////////////////////
void GamepadMultiDeviceDriver::RegisterGamepadDataCallback(
    GamepadDriverDataCallback callback) {
  mGamepadCallback = callback;
  if (_Init() == false) {
    throw DeviceException("Error initializing Gamepad.");
  }
}

///////////////////////////////////////////////////////////////////////////////
double GamepadMultiDeviceDriver::GetAxisValue(int id) {
  return id < (int)m_vAxes.size() ? m_vAxes[id] : 0;
}

///////////////////////////////////////////////////////////////////////////////
bool GamepadMultiDeviceDriver::IsButtonPressed(int id) {
  return id < (int)m_vButtonStates.size() ? m_vButtonStates[id] == 1 : false;
}

//////////////////////////////////////////////////////////////////////////////
bool GamepadMultiDeviceDriver::_OnButtonDown(void* sender, const char* eventID,
                                             void* eventData, void* context) {
  struct Gamepad_buttonEvent* event;
  GamepadMultiDeviceDriver* pHandler = (GamepadMultiDeviceDriver*)context;

  event = (Gamepad_buttonEvent*)eventData;
  pHandler->m_vButtonStates[event->buttonID] = event->down;
  if (/*_verbose*/true) {
    printf("Button %u down (%d) on device %u at %f\n", event->buttonID,
           (int)event->down, event->device->deviceID, event->timestamp);
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////////
bool GamepadMultiDeviceDriver::_OnButtonUp(void* sender, const char* eventID,
                                           void* eventData, void* context) {
  struct Gamepad_buttonEvent* event;
  GamepadMultiDeviceDriver* pHandler = (GamepadMultiDeviceDriver*)context;

  event = (Gamepad_buttonEvent*)eventData;
  pHandler->m_vButtonStates[event->buttonID] = event->down;
  if (/*_verbose*/true) {
    printf("Button %u up (%d) on device %u at %f\n", event->buttonID,
           (int)event->down, event->device->deviceID, event->timestamp);
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////////
bool GamepadMultiDeviceDriver::_OnAxisMoved(void* sender, const char* eventID,
                                            void* eventData, void* context) {
  struct Gamepad_axisEvent* event;
  GamepadMultiDeviceDriver* pHandler = (GamepadMultiDeviceDriver*)context;

  event = (Gamepad_axisEvent*)eventData;
  pHandler->m_vAxes[event->axisID] = event->value;
  if (/*_verbose*/true) {
    printf("Axis %u moved to %f on device %u at %f\n", event->axisID,
           event->value, event->device->deviceID, event->timestamp);
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////////
bool GamepadMultiDeviceDriver::_OnDeviceAttached(void* sender,
                                                 const char* eventID,
                                                 void* eventData,
                                                 void* context) {
  struct Gamepad_device* device;
  GamepadMultiDeviceDriver* pHandler = (GamepadMultiDeviceDriver*)context;

  device = (Gamepad_device*)eventData;
  if (/*_verbose*/true) {
    printf("Device ID %u attached (vendor = 0x%X; product = 0x%X)\n",
           device->deviceID, device->vendorID, device->productID);
  }
  device->eventDispatcher->registerForEvent(device->eventDispatcher,
                                            GAMEPAD_EVENT_BUTTON_DOWN,
                                            _OnButtonDown, pHandler);
  device->eventDispatcher->registerForEvent(
      device->eventDispatcher, GAMEPAD_EVENT_BUTTON_UP, _OnButtonUp, pHandler);
  device->eventDispatcher->registerForEvent(device->eventDispatcher,
                                            GAMEPAD_EVENT_AXIS_MOVED,
                                            _OnAxisMoved, pHandler);

  std::cout << "Opened joystick zero which has " << device->numAxes << " axes "
            << " and " << device->numButtons << " buttons." << std::endl;
  pHandler->m_vAxes.resize(device->numAxes, 0);
  pHandler->m_vButtonStates.resize(device->numButtons, 0);

  // reset all joystick axes and buttons
  for (size_t ii = 0; ii < pHandler->m_vAxes.size(); ii++) {
    pHandler->m_vAxes[ii] = 0;
  }

  for (size_t ii = 0; ii < pHandler->m_vButtonStates.size(); ii++) {
    pHandler->m_vButtonStates[ii] = 0;
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////////
bool GamepadMultiDeviceDriver::_OnDeviceRemoved(void* sender,
                                                const char* eventID,
                                                void* eventData,
                                                void* context) {
  struct Gamepad_device* device;

  device = (Gamepad_device*)eventData;
  // if (verbose) {
  printf("Device ID %u removed\n", device->deviceID);
  //}
  return true;
}

//////////////////////////////////////////////////////////////////////////////
void GamepadMultiDeviceDriver::_ThreadCaptureFunc() {
  Gamepad_eventDispatcher()->registerForEvent(Gamepad_eventDispatcher(),
                                              GAMEPAD_EVENT_DEVICE_ATTACHED,
                                              _OnDeviceAttached, this);
  Gamepad_eventDispatcher()->registerForEvent(Gamepad_eventDispatcher(),
                                              GAMEPAD_EVENT_DEVICE_REMOVED,
                                              _OnDeviceRemoved, this);
  Gamepad_init();
}
