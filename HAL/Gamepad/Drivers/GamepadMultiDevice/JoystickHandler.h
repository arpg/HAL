#pragma once

#include <stdio.h>
#include <iostream>

#include <thread>
#include <vector>

#include "Gamepad/Gamepad.h"
#include "Gamepad/EventDispatcher.h"

#define JOYSTICK_AXIS_MAX 1.0
#define JOYSTICK_AXIS_MIN -1.0

class JoystickHandler
{
public:
    JoystickHandler();
    bool InitializeJoystick();
    void UpdateJoystick();

    double GetAxisValue(int id) { return id < (int)m_vAxes.size() ? m_vAxes[id] : 0; }
    bool IsButtonPressed(int id){ return id < (int)m_vButtonStates.size() ? m_vButtonStates[id] == 1 : false; }

    void JoinThread();


private:
    void _ThreadFunc();
    static bool _OnButtonDown(void *sender, const char *eventID, void *eventData, void *context);
    static bool _OnButtonUp(void *sender, const char *eventID, void *eventData, void *context);
    static bool _OnAxisMoved(void *sender, const char *eventID, void *eventData, void *context);
    static bool _OnDeviceAttached(void *sender, const char *eventID, void *eventData, void *context);
    static bool _OnDeviceRemoved(void *sender, const char *eventID, void *eventData, void *context);

    std::vector<double> m_vAxes;
    std::vector<int>    m_vButtonStates;
    std::thread*      m_pJoystickThread;

};
