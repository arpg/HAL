#include "JoystickHandler.h"

const bool VERBOSE = true;

JoystickHandler::JoystickHandler()
{
}


void JoystickHandler::UpdateJoystick()
{
    Gamepad_detectDevices();
    Gamepad_processEvents();
}

bool JoystickHandler::_OnButtonDown(void * sender, const char * eventID, void * eventData, void * context) {
    struct Gamepad_buttonEvent * event;
    JoystickHandler* pHandler = (JoystickHandler*)context;

    event = (Gamepad_buttonEvent*)eventData;
    pHandler->m_vButtonStates[event->buttonID] =event->down;
    if (VERBOSE) {
        printf("Button %u down (%d) on device %u at %f\n", event->buttonID, (int) event->down, event->device->deviceID, event->timestamp);
    }
    return true;
}

bool JoystickHandler::_OnButtonUp(void * sender, const char * eventID, void * eventData, void * context) {
    struct Gamepad_buttonEvent * event;
    JoystickHandler* pHandler = (JoystickHandler*)context;

    event = (Gamepad_buttonEvent*)eventData;
    pHandler->m_vButtonStates[event->buttonID] =event->down;
    if (VERBOSE) {
        printf("Button %u up (%d) on device %u at %f\n", event->buttonID, (int) event->down, event->device->deviceID, event->timestamp);
    }
    return true;
}

bool JoystickHandler::_OnAxisMoved(void * sender, const char * eventID, void * eventData, void * context) {
    struct Gamepad_axisEvent * event;
    JoystickHandler* pHandler = (JoystickHandler*)context;

    event = (Gamepad_axisEvent*)eventData;
    pHandler->m_vAxes[event->axisID] = event->value;
    if (VERBOSE) {
        printf("Axis %u moved to %f on device %u at %f\n", event->axisID, event->value, event->device->deviceID, event->timestamp);
    }
    return true;
}

bool JoystickHandler::_OnDeviceAttached(void * sender, const char * eventID, void * eventData, void * context) {
    struct Gamepad_device * device;
    JoystickHandler* pHandler = (JoystickHandler*)context;

    device = (Gamepad_device*)eventData;
    if (VERBOSE) {
        printf("Device ID %u attached (vendor = 0x%X; product = 0x%X)\n", device->deviceID, device->vendorID, device->productID);
    }
    device->eventDispatcher->registerForEvent(device->eventDispatcher, GAMEPAD_EVENT_BUTTON_DOWN, _OnButtonDown,pHandler);
    device->eventDispatcher->registerForEvent(device->eventDispatcher, GAMEPAD_EVENT_BUTTON_UP, _OnButtonUp,pHandler);
    device->eventDispatcher->registerForEvent(device->eventDispatcher, GAMEPAD_EVENT_AXIS_MOVED, _OnAxisMoved,pHandler);



    std::cout << "Opened joystick zero which has " << device->numAxes << " axes " << " and " << device->numButtons << " buttons." << std::endl;
    pHandler->m_vAxes.resize(device->numAxes ,0);
    pHandler->m_vButtonStates.resize(device->numButtons ,0);

    //reset all joystick axes and buttons
    for(size_t ii = 0 ; ii < pHandler->m_vAxes.size() ;ii++) {
        pHandler->m_vAxes[ii] = 0;
    }

    for(size_t ii = 0 ; ii < pHandler->m_vButtonStates.size() ;ii++) {
        pHandler->m_vButtonStates[ii] = 0;
    }
    return true;
}

bool JoystickHandler::_OnDeviceRemoved(void * sender, const char * eventID, void * eventData, void * context) {
    struct Gamepad_device * device;

    device = (Gamepad_device*)eventData;
    //if (verbose) {
        printf("Device ID %u removed\n", device->deviceID);
    //}
    return true;
}

bool JoystickHandler::InitializeJoystick()
{
    m_pJoystickThread = new std::thread(std::bind(&JoystickHandler::_ThreadFunc,this));

    return true;
}

void JoystickHandler::JoinThread() {
    m_pJoystickThread->join();
}

void JoystickHandler::_ThreadFunc()
{
    Gamepad_eventDispatcher()->registerForEvent(Gamepad_eventDispatcher(), GAMEPAD_EVENT_DEVICE_ATTACHED, _OnDeviceAttached, this);
    Gamepad_eventDispatcher()->registerForEvent(Gamepad_eventDispatcher(), GAMEPAD_EVENT_DEVICE_REMOVED, _OnDeviceRemoved, this);
    Gamepad_init();

    Gamepad_RunLoop();
}

