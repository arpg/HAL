#include <iostream>

#include <HAL/Devices/DeviceException.h>
#include <HAL/Utils/TicToc.h>

#include "GamepadMultiDeviceDriver.h"

using namespace hal;

GamepadDriverDataCallback GamepadMultiDeviceDriver::mGamepadCallback = nullptr;

const int DEFAULT_UPDATE_RATE_TIMEOUT_MS = 10;

bool GamepadMultiDeviceDriver::_verbose;
hal::GamepadMsg GamepadMultiDeviceDriver::pbGamepadMsg;
hal::VectorMsg* GamepadMultiDeviceDriver::pbVecButtonsMsg;
hal::VectorMsg* GamepadMultiDeviceDriver::pbVecAxesMsg;
///////////////////////////////////////////////////////////////////////////////
GamepadMultiDeviceDriver::GamepadMultiDeviceDriver() : mShouldRun(false) {
  _verbose = false;
  pbVecAxesMsg = pbGamepadMsg.mutable_axes();
  pbVecButtonsMsg = pbGamepadMsg.mutable_buttons();
}

///////////////////////////////////////////////////////////////////////////////
GamepadMultiDeviceDriver::~GamepadMultiDeviceDriver() {
  mShouldRun = false;
  mDeviceThread.join();
  mDeviceUpdateThread.join();
  delete(pbVecAxesMsg);
  delete(pbVecButtonsMsg);
}

///////////////////////////////////////////////////////////////////////////////
bool GamepadMultiDeviceDriver::_Init() {
  // Device communication parameters

  mShouldRun = true;
  mDeviceUpdateThread = std::thread(
      std::bind(&GamepadMultiDeviceDriver::_ThreadUpdateGamepad, this));
  mDeviceThread =
      std::thread(std::bind(&GamepadMultiDeviceDriver::_ThreadFunc, this));
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

//////////////////////////////////////////////////////////////////////////////
bool GamepadMultiDeviceDriver::_OnButtonDown(void* /*sender*/,
                                             const char* /*eventID*/,
                                             void* eventData, void* context) {
  pbGamepadMsg.set_system_time(hal::Tic());
  struct Gamepad_buttonEvent* event;
  GamepadMultiDeviceDriver* pHandler = (GamepadMultiDeviceDriver*)context;

  event = (Gamepad_buttonEvent*)eventData;
  pHandler->m_vButtonStates[event->buttonID] = event->down;
  pbVecButtonsMsg->set_data(event->buttonID,event->down);
//  pbGamepadMsg.buttons().set_data(event->buttonID,event->down);
  if (mGamepadCallback) {
    mGamepadCallback(pbGamepadMsg);
  }

  if (_verbose) {
    printf("Button %u down (%d) on device %u at %f\n", event->buttonID,
           (int)event->down, event->device->deviceID, event->timestamp);
  }

//  delete(event);
//  delete(pHandler);
  return true;
}

//////////////////////////////////////////////////////////////////////////////
bool GamepadMultiDeviceDriver::_OnButtonUp(void* /*sender*/,
                                           const char* /*eventID*/,
                                           void* eventData, void* context) {
  pbGamepadMsg.set_system_time(hal::Tic());
  struct Gamepad_buttonEvent* event;
  GamepadMultiDeviceDriver* pHandler = (GamepadMultiDeviceDriver*)context;

  event = (Gamepad_buttonEvent*)eventData;
  pHandler->m_vButtonStates[event->buttonID] = event->down;
  pbVecButtonsMsg->set_data(event->buttonID,event->down);
//  pbGamepadMsg.buttons().set_data(event->buttonID,event->down);
  if (mGamepadCallback) {
    mGamepadCallback(pbGamepadMsg);
  }

  if (_verbose) {
    printf("Button %u up (%d) on device %u at %f\n", event->buttonID,
           (int)event->down, event->device->deviceID, event->timestamp);
  }

//  delete(event);
//  delete(pHandler);
  return true;
}

//////////////////////////////////////////////////////////////////////////////
bool GamepadMultiDeviceDriver::_OnAxisMoved(void* /*sender*/,
                                            const char* /*eventID*/,
                                            void* eventData, void* context) {

  pbGamepadMsg.set_system_time(hal::Tic());
  struct Gamepad_axisEvent* event;
  GamepadMultiDeviceDriver* pHandler = (GamepadMultiDeviceDriver*)context;

  event = (Gamepad_axisEvent*)eventData;
  pHandler->m_vAxes[event->axisID] = event->value;
  pbVecAxesMsg->set_data(event->axisID,event->value);
//  pbGamepadMsg.axes().set_data(event->axisID,event->value);
  if (mGamepadCallback) {
    mGamepadCallback(pbGamepadMsg);
  }

  if (_verbose) {
    printf("Axis %u moved to %f on device %u at %f\n", event->axisID,
           event->value, event->device->deviceID, event->timestamp);
  }

//  delete(event);
  return true;
}

//////////////////////////////////////////////////////////////////////////////
bool GamepadMultiDeviceDriver::_OnDeviceAttached(void* /*sender*/,
                                                 const char* /*eventID*/,
                                                 void* eventData,
                                                 void* context) {
  struct Gamepad_device* device;
  GamepadMultiDeviceDriver* pHandler = (GamepadMultiDeviceDriver*)context;

  device = (Gamepad_device*)eventData;

  pbGamepadMsg.set_system_time(hal::Tic());
  pbGamepadMsg.set_device_id(device->deviceID);
  pbGamepadMsg.set_vendor_id(device->vendorID);
  pbGamepadMsg.set_product_id(device->productID);
  if (_verbose) {
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
    pbVecAxesMsg->add_data(0);
  }

  for (size_t ii = 0; ii < pHandler->m_vButtonStates.size(); ii++) {
    pHandler->m_vButtonStates[ii] = 0;
    pbVecButtonsMsg->add_data(0);
  }
  if (mGamepadCallback) {
    mGamepadCallback(pbGamepadMsg);
  }

//  delete(device);
//  delete(pHandler);
  return true;
}

//////////////////////////////////////////////////////////////////////////////
bool GamepadMultiDeviceDriver::_OnDeviceRemoved(void* /*sender*/,
                                                const char* /*eventID*/,
                                                void* eventData,
                                                void* /*context*/) {
  struct Gamepad_device* device;

  device = (Gamepad_device*)eventData;
  printf("Device ID %u removed\n", device->deviceID);
  delete(device);
  return true;
}

//////////////////////////////////////////////////////////////////////////////
void GamepadMultiDeviceDriver::_ThreadFunc() {
  Gamepad_eventDispatcher()->registerForEvent(Gamepad_eventDispatcher(),
                                              GAMEPAD_EVENT_DEVICE_ATTACHED,
                                              _OnDeviceAttached, this);
  Gamepad_eventDispatcher()->registerForEvent(Gamepad_eventDispatcher(),
                                              GAMEPAD_EVENT_DEVICE_REMOVED,
                                              _OnDeviceRemoved, this);
  Gamepad_init();
  Gamepad_RunLoop();
}

//////////////////////////////////////////////////////////////////////////////
void GamepadMultiDeviceDriver::_ThreadUpdateGamepad() {
  while (1) {
    Gamepad_detectDevices();
    Gamepad_processEvents();

    // delay is required
    std::this_thread::sleep_for(
        std::chrono::milliseconds(DEFAULT_UPDATE_RATE_TIMEOUT_MS));
  }
}

//////////////////////////////////////////////////////////////////////////////
