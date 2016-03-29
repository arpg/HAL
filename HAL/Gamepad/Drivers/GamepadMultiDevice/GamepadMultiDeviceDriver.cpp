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
  // Set _vebose=true for debug purposes
  _verbose = false;

  pbVecAxesMsg = pbGamepadMsg.mutable_axes();
  pbVecButtonsMsg = pbGamepadMsg.mutable_buttons();
}

///////////////////////////////////////////////////////////////////////////////
GamepadMultiDeviceDriver::~GamepadMultiDeviceDriver() {
  mShouldRun = false;
  mDeviceThread.join();
  mDeviceUpdateThread.join();
  delete (pbVecAxesMsg);
  delete (pbVecButtonsMsg);
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
                                             void* eventData,
                                             void* /*context*/) {
  pbGamepadMsg.set_system_time(hal::Tic());
  Gamepad_buttonEvent* event;

  event = (Gamepad_buttonEvent*)eventData;
  pbVecButtonsMsg->set_data(event->buttonID, event->down);
  if (mGamepadCallback) {
    mGamepadCallback(pbGamepadMsg);
  }

  if (_verbose) {
    printf("Button %u down (%d) on device %u at %f\n", event->buttonID,
           (int)event->down, event->device->deviceID, event->timestamp);
  }

  return true;
}

//////////////////////////////////////////////////////////////////////////////
bool GamepadMultiDeviceDriver::_OnButtonUp(void* /*sender*/,
                                           const char* /*eventID*/,
                                           void* eventData, void* /*context*/) {
  pbGamepadMsg.set_system_time(hal::Tic());
  Gamepad_buttonEvent* event;

  event = (Gamepad_buttonEvent*)eventData;
  pbVecButtonsMsg->set_data(event->buttonID, event->down);
  if (mGamepadCallback) {
    mGamepadCallback(pbGamepadMsg);
  }

  if (_verbose) {
    printf("Button %u up (%d) on device %u at %f\n", event->buttonID,
           (int)event->down, event->device->deviceID, event->timestamp);
  }

  return true;
}

//////////////////////////////////////////////////////////////////////////////
bool GamepadMultiDeviceDriver::_OnAxisMoved(void* /*sender*/,
                                            const char* /*eventID*/,
                                            void* eventData,
                                            void* /*context*/) {
  pbGamepadMsg.set_system_time(hal::Tic());
  Gamepad_axisEvent* event;

  event = (Gamepad_axisEvent*)eventData;
  pbVecAxesMsg->set_data(event->axisID, event->value);
  if (mGamepadCallback) {
    mGamepadCallback(pbGamepadMsg);
  }

  if (_verbose) {
    printf("Axis %u moved to %f on device %u at %f\n", event->axisID,
           event->value, event->device->deviceID, event->timestamp);
  }

  return true;
}

//////////////////////////////////////////////////////////////////////////////
bool GamepadMultiDeviceDriver::_OnDeviceAttached(void* /*sender*/,
                                                 const char* /*eventID*/,
                                                 void* eventData,
                                                 void* context) {
  // TODO: device and pHandler are not handled properly. device object should be
  // created in GamepadMultiDeviceDriver class and deleted properly
  Gamepad_device* device;
  GamepadMultiDeviceDriver* pHandler = (GamepadMultiDeviceDriver*)context;

  device = (Gamepad_device*)eventData;

  pbGamepadMsg.set_system_time(hal::Tic());
  pbGamepadMsg.set_device_id(device->deviceID);
  pbGamepadMsg.set_vendor_id(device->vendorID);
  pbGamepadMsg.set_product_id(device->productID);
  printf("Device ID %u attached (vendor = 0x%X; product = 0x%X)\n",
         device->deviceID, device->vendorID, device->productID);

  device->eventDispatcher->registerForEvent(device->eventDispatcher,
                                            GAMEPAD_EVENT_BUTTON_DOWN,
                                            _OnButtonDown, pHandler);
  device->eventDispatcher->registerForEvent(
      device->eventDispatcher, GAMEPAD_EVENT_BUTTON_UP, _OnButtonUp, pHandler);
  device->eventDispatcher->registerForEvent(device->eventDispatcher,
                                            GAMEPAD_EVENT_AXIS_MOVED,
                                            _OnAxisMoved, pHandler);

  std::cout << "Opened joystick " << device->deviceID << " which has "
            << device->numAxes << " axes "
            << " and " << device->numButtons << " buttons." << std::endl;
  pbGamepadMsg.set_num_buttons(device->numButtons);
  pbGamepadMsg.set_num_axes(device->numAxes);

  // reset all joystick axes and buttons
  for (size_t ii = 0; ii < device->numAxes; ii++) {
    pbVecAxesMsg->add_data(0);
  }
  for (size_t ii = 0; ii < device->numButtons; ii++) {
    pbVecButtonsMsg->add_data(0);
  }

  if (mGamepadCallback) {
    mGamepadCallback(pbGamepadMsg);
  }

  return true;
}

//////////////////////////////////////////////////////////////////////////////
bool GamepadMultiDeviceDriver::_OnDeviceRemoved(void* /*sender*/,
                                                const char* /*eventID*/,
                                                void* eventData,
                                                void* /*context*/) {
  struct Gamepad_device* device;

  device = (Gamepad_device*)eventData;
  std::cout << "Device ID " << device->deviceID << " removed." << std::endl;
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
