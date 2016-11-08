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
  should_run_mutex.lock();
  mShouldRun = false;
  should_run_mutex.unlock();
  if(_GamepadThreadHandle.joinable()) {
    _GamepadThreadHandle.join();
  } else {
    std::cerr << "Thread not joinable in " << __FILE__ << " Line: " << __LINE__ << std::endl;
  }
}

///////////////////////////////////////////////////////////////////////////////
bool GamepadMultiDeviceDriver::_Init() {
  // Device communication parameters

  mShouldRun = true;
  _GamepadThreadHandle =
      std::thread(std::bind(&GamepadMultiDeviceDriver::_GamepadThread, this));
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

////////////////////////////////////////////////////////////////////////////////
void GamepadMultiDeviceDriver::_OnButtonDown(struct Gamepad_device* device, unsigned int buttonID, double timestamp, void* /*context*/) {
  pbGamepadMsg.set_system_time(hal::Tic());
  pbVecButtonsMsg->set_data(buttonID, 1);
  if (mGamepadCallback) {
    mGamepadCallback(pbGamepadMsg);
  }

  if (_verbose) {
    printf("Button %u down (%d) on device %u at %f\n", buttonID,
           1, device->deviceID, timestamp);
  }
}

////////////////////////////////////////////////////////////////////////////////
void GamepadMultiDeviceDriver::_OnButtonUp(Gamepad_device *device, unsigned int buttonID, double timestamp, void */*context*/) {
  pbGamepadMsg.set_system_time(hal::Tic());
  pbVecButtonsMsg->set_data(buttonID, 0);
  if (mGamepadCallback) {
    mGamepadCallback(pbGamepadMsg);
  }

  if (_verbose) {
    printf("Button %u up (%d) on device %u at %f\n", buttonID,
           0, device->deviceID, timestamp);
  }
}

////////////////////////////////////////////////////////////////////////////////
void GamepadMultiDeviceDriver::_OnAxisMoved(struct Gamepad_device* device, unsigned int axisID, float value, float /*lastValue*/, double timestamp, void * /*context*/) {
  pbGamepadMsg.set_system_time(hal::Tic());
  pbVecAxesMsg->set_data(axisID,value);
  if (mGamepadCallback) {
    mGamepadCallback(pbGamepadMsg);
  }

  if (_verbose) {
    printf("Axis %u moved to %f on device %u at %f\n", axisID,
           value, device->deviceID, timestamp);
  }
}

//////////////////////////////////////////////////////////////////////////////
void GamepadMultiDeviceDriver::_OnDeviceAttached(struct Gamepad_device* device, void* /*context*/) {
  pbGamepadMsg.set_system_time(hal::Tic());
  pbGamepadMsg.set_device_id(device->deviceID);
  pbGamepadMsg.set_vendor_id(device->vendorID);
  pbGamepadMsg.set_product_id(device->productID);
  printf("Device ID %u attached (vendor = 0x%X; product = 0x%X)\n",
         device->deviceID, device->vendorID, device->productID);

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
}

////////////////////////////////////////////////////////////////////////////////
void GamepadMultiDeviceDriver::_OnDeviceRemoved(struct Gamepad_device* device, void* /*context*/) {
  std::cout << "Device ID " << device->deviceID << " removed." << std::endl;
}

//////////////////////////////////////////////////////////////////////////////
void GamepadMultiDeviceDriver::_GamepadThread() {
  Gamepad_deviceAttachFunc(_OnDeviceAttached,NULL);
  Gamepad_deviceRemoveFunc(_OnDeviceRemoved, NULL);
  Gamepad_axisMoveFunc(_OnAxisMoved, NULL);
  Gamepad_buttonUpFunc(_OnButtonUp, NULL);
  Gamepad_buttonDownFunc(_OnButtonDown, NULL);
  Gamepad_init();
  while (mShouldRun) {
    should_run_mutex.unlock();
    Gamepad_detectDevices();
    Gamepad_processEvents();
    std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_UPDATE_RATE_TIMEOUT_MS));
    should_run_mutex.lock();
  }
  should_run_mutex.unlock();
  Gamepad_shutdown();
  std::cout << "exiting gamepadthread" << std::endl;
}

//////////////////////////////////////////////////////////////////////////////
