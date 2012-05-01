/*
 * $Id: w_event.c,v 1.1 2005/02/19 00:37:41 slava Exp $
 *
 * Copyright (C) 2000-2005 by Slava Monich
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met: 
 *
 *   1.Redistributions of source code must retain the above copyright 
 *     notice, this list of conditions and the following disclaimer. 
 *   2.Redistributions in binary form must reproduce the above copyright 
 *     notice, this list of conditions and the following disclaimer 
 *     in the documentation and/or other materials provided with the 
 *     distribution. 
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) ARISING 
 * IN ANY WAY OUT OF THE USE OR INABILITY TO USE THIS SOFTWARE, EVEN 
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 *
 * The views and conclusions contained in the software and documentation 
 * are those of the authors and should not be interpreted as representing 
 * any official policies, either expressed or implied.
 */

#include "s_event.h"

/**
 * Event API implementation for Win32 environment
 */

#ifndef _WIN32
#  error "_WIN32 must be defined to compile Win32 code"
#endif

/**
 * Initialize a notification event. Returns False if initialization fails.
 */
Bool EVENT_Init(Event * e) 
{
    e->handle = CreateEvent(NULL, True, False, NULL);
    ASSERT(e->handle);
    if (!e->handle) {
        TRACE1("CreateEvent() failed, error %lu\n", GetLastError());
        return False;
    }
    return True;
}

/**
 * Destroy the notification event 
 */
void EVENT_Destroy(Event * e) 
{
    VERIFY(CloseHandle(e->handle));
}

/**
 * Notify all waiters and switch into signalled state.
 */
Bool EVENT_Set(Event * e) 
{
    if (!SetEvent(e->handle)) {
        ASSMSG1("SetEvent() failed, error %lu", GetLastError());
        return False;
    }
    return True;
}

/**
 * Notify all waiters and reset state to non-signaled
 */
Bool EVENT_Pulse(Event * e) 
{
    if (!PulseEvent(e->handle)) {
        ASSMSG1("PulseEvent() failed, error %lu", GetLastError());
        return False;
    }
    return True;
}

/**
 * Reset state to non-signaled.
 */
Bool EVENT_Reset(Event * e) 
{
    if (!ResetEvent(e->handle)) {
        ASSMSG1("ResetEvent() failed, error %lu", GetLastError());
        return False;
    }
    return True;
}

/**
 * Returns the current state, EVENT_SIGNALED or EVENT_NOT_SIGNALED, 
 * of a given event object.
 */
EventState EVENT_State(Event * e)
{
    if (EVENT_TimeWait(e,0) == WAIT_STATE_OK) {
        return EVENT_SIGNALED;
    } else {
        return EVENT_NOT_SIGNALED;
    }
}

/**
 * Wait for event to become signaled.
 */
WaitState EVENT_Wait(Event * e) 
{
    DWORD status = WaitForSingleObject(e->handle, INFINITE);
    if (status == WAIT_OBJECT_0) {
        return WAIT_STATE_OK;
    } else {
        TRACE1("WaitForSingleObject() status %08lX\n",status);
        ASSMSG1("WaitForSingleObject() failed, error %d",GetLastError());
        return WAIT_STATE_ERROR;
    }
}

/**
 * Wait for event to become signaled.
 */
WaitState EVENT_TimeWait(Event * e, long ms) 
{
    if (ms < 0) {
        return EVENT_Wait(e);
    } else {
        DWORD status = WaitForSingleObject(e->handle, ms);
        switch (status) {
        case WAIT_OBJECT_0:  return WAIT_STATE_OK;
        case WAIT_TIMEOUT:   return WAIT_STATE_TIMEOUT;
        }
        TRACE1("WaitForSingleObject() status %08lX\n",status);
        ASSMSG1("WaitForSingleObject() failed, error %d",GetLastError());
        return WAIT_STATE_ERROR;
    }
}

/*
 * HISTORY:
 *
 * $Log: w_event.c,v $
 * Revision 1.1  2005/02/19 00:37:41  slava
 * o separated platform specific code from platform independent code
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
