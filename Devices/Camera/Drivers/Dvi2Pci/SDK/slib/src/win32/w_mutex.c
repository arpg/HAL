/*
 * $Id: w_mutex.c,v 1.5 2009/11/17 00:14:25 slava Exp $
 *
 * Copyright (C) 2000-2009 by Slava Monich
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

#include "s_mutex.h"
#include "s_lockp.h"

/**
 * Mutex API implementation for Win32 environment
 */

#ifndef _WIN32
#  error "_WIN32 must be defined to compile Win32 code"
#endif

/**
 * Initialize a mutex. Returns False if initialization fails.
 */
Bool MUTEX_Init(Mutex * m)
{
    m->thread = 0;
    m->handle = CreateMutex(NULL, False, NULL);
    if (m->handle) {
        m->lock.type = &LockType_Mutex;
        return True;
    }
    ASSMSG1("CreateMutex() failed, error %lu", GetLastError());
    return False;
}

/**
 * Destroy the mutex 
 */
void MUTEX_Destroy(Mutex * m)
{
    ASSERT(!m->thread);
    VERIFY(CloseHandle(m->handle));
}

/**
 * Try to acquire the mutex. Returns True if mutex has been successfully
 * acquired, False otherwise.
 */
Bool MUTEX_TryLock(Mutex * m)
{
    ASSERT(!MUTEX_IsLocked(m));
    if (m->thread != GetCurrentThreadId()) {
        DWORD status = WaitForSingleObject(m->handle,0);
        switch (status) {
        case WAIT_ABANDONED_0:
            ASSMSG("Mutex was abandoned!");
            /* fall through */
        case WAIT_OBJECT_0:
            m->thread = GetCurrentThreadId();
            return True;
        case WAIT_TIMEOUT:
            return False;
        }
        TRACE1("WaitForSingleObject() status %08lX\n",status);
        ASSMSG1("WaitForSingleObject() failed, error %d",GetLastError());
    }
    return False;
}

/**
 * Locks the mutex. If mutex is already locked, waits until it becomes
 * available. Returns True if mutex has been successfully acquired, 
 * False otherwise.
 */
Bool MUTEX_Lock(Mutex * m)
{
    ASSERT(!MUTEX_IsLocked(m));
    if (m->thread != GetCurrentThreadId()) {
        DWORD status = WaitForSingleObject(m->handle,INFINITE);
        switch (status) {
        case WAIT_ABANDONED_0:
            ASSMSG("Mutex was abandoned!");
            /* fall through */
        case WAIT_OBJECT_0:
            m->thread = GetCurrentThreadId();
            return True;
        case WAIT_TIMEOUT:
            return False;
        }
        TRACE1("WaitForSingleObject() status %08lX\n",status);
        ASSMSG1("WaitForSingleObject() failed, error %d",GetLastError());
    }
    return False;
}

/**
 * Tests if mutex is locked by the current thread
 */
Bool MUTEX_IsLocked(const Mutex * m)
{
    return (m->thread == GetCurrentThreadId());
}

/**
 * Release the mutex.
 */
void MUTEX_Unlock(Mutex * m)
{
    ASSERT(MUTEX_IsLocked(m));
    m->thread = 0;
    if (!ReleaseMutex(m->handle)) {
        TRACE1("ERROR: thr %08lX can't release mutex\n",GetCurrentThreadId());
        ASSMSG1("ReleaseMutex() failed, error %lu", GetLastError());
    }
}

/*
 * HISTORY:
 *
 * $Log: w_mutex.c,v $
 * Revision 1.5  2009/11/17 00:14:25  slava
 * o introducing generic synchronization API. This is a pretty destructive
 *   checkin, it's not 100% backward compatible. The Lock type is now a
 *   generic lock. What used to be called Lock is now RWLock. Sorry.
 *
 * Revision 1.4  2009/10/05 14:53:15  slava
 * o added MUTEX_IsLocked() function
 *
 * Revision 1.3  2008/12/10 17:26:03  slava
 * o ASSERT that Win32 mutex isn't being used recursively, for consistency
 *   with other platforms.
 *
 * Revision 1.2  2008/12/05 08:53:41  slava
 * o fixed a bug in MUTEX_Init (CreateMutex failure wasn't handled)
 *
 * Revision 1.1  2005/02/19 00:37:41  slava
 * o separated platform specific code from platform independent code
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
