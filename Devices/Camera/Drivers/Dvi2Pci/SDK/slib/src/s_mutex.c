/*
 * $Id: s_mutex.c,v 1.27 2009/11/17 00:14:24 slava Exp $
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
#include "s_mem.h"

/**
 * Creates a mutex. This function is defined here (rather than in s_lock.c)
 * to avoid unnecessary linkage with the lock types that you are not using.
 */
Lock * LOCK_CreateMutex()
{
    Mutex * m = MUTEX_Create();
    return m ? &m->lock : NULL;
}

/**
 * Platform independent part of Mutex API.
 */

/**
 * Allocates a new mutex and initializes it
 */
Mutex * MUTEX_Create()
{
    Mutex * m = MEM_New(Mutex);
    if (m) {
        if (MUTEX_Init(m)) {
            return m;
        }
        MEM_Free(m);
    }
    return NULL;
}

/**
 * Deletes and deallocates the mutex previously allocated with 
 * MUTEX_Create()
 */
void MUTEX_Delete(Mutex * m)
{
    if (m) {
        MUTEX_Destroy(m);
        MEM_Free(m);
    }
}

/* 
 * Generic API for synchronization objects:
 *
 * LockLock     - locks the resource for exclusive use by the current thread
 * LockTryLock  - attempts to lock the resource without waiting
 * LockCount    - returns the nesting count for the current thread
 * LockUnlock   - releases the lock
 * LockFree     - deletes the lock
 */

#define _MutexCast(l) CAST(l,Mutex,lock)
#if DEBUG
STATIC Mutex * MutexCast(Lock * lock) {
    ASSERT(lock && lock->type == &LockType_Mutex);
    return _MutexCast(lock);
}
STATIC Mutex * MutexCastC(const Lock * lock) {
    ASSERT(lock && lock->type == &LockType_Mutex);
    return _MutexCast(lock);
}
#else
#  define MutexCast(l)  _MutexCast(l)
#  define MutexCastC(l) _MutexCast(l)
#endif

STATIC Bool _MUTEX_Lock(Lock * l)      { return MUTEX_Lock(MutexCast(l)); }
STATIC Bool _MUTEX_TryLock(Lock * l)   { return MUTEX_TryLock(MutexCast(l));}
STATIC int  _MUTEX_Count(const Lock* l){ return MUTEX_IsLocked(MutexCastC(l));}
STATIC void _MUTEX_Unlock(Lock * l)    { MUTEX_Unlock(MutexCast(l)); }
STATIC void _MUTEX_Free(Lock * l)      { MUTEX_Delete(MutexCast(l)); }

const LockType LockType_Mutex = {
    TEXT("Mutex")   /* name     */,
    _MUTEX_Lock     /* lock     */,
    _MUTEX_TryLock  /* trylock  */,
    _MUTEX_Count    /* count    */,
    _MUTEX_Unlock   /* unlock   */,
    _MUTEX_Free     /* free     */
};

/*
 * HISTORY:
 *
 * $Log: s_mutex.c,v $
 * Revision 1.27  2009/11/17 00:14:24  slava
 * o introducing generic synchronization API. This is a pretty destructive
 *   checkin, it's not 100% backward compatible. The Lock type is now a
 *   generic lock. What used to be called Lock is now RWLock. Sorry.
 *
 * Revision 1.26  2005/02/19 00:37:41  slava
 * o separated platform specific code from platform independent code
 *
 * Revision 1.25  2004/06/13 15:41:08  slava
 * o added couple ASSERTs in MUTEX_Lock for NT kernel
 *
 * Revision 1.24  2003/12/04 04:48:55  slava
 * o replaced _LINUX_KERNEL_ with _LINUX_KERNEL for consistency with other
 *   platform defines such as _UNIX, _WIN32, _NT_KERNEL etc.
 *
 * Revision 1.23  2003/12/01 03:00:01  slava
 * o fixed the use of Linux kernel spinlock functions
 *
 * Revision 1.22  2003/11/30 02:49:58  slava
 * o port to Linux kernel mode environment
 *
 * Revision 1.21  2003/11/02 17:45:02  slava
 * o added ASSERTs that these functions are called at appropriate IRQ level
 *
 * Revision 1.20  2003/11/02 17:30:18  slava
 * o keep track of the thread that has locked the mutex (NT kernel mode)
 * o made MUTEX_TryLock implementation mode useful (although still not
 *   perfect) in NT kernel mode environment
 *
 * Revision 1.19  2003/05/21 00:11:04  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.18  2002/08/27 11:52:03  slava
 * o some reformatting
 *
 * Revision 1.17  2002/05/29 08:27:22  slava
 * o removed MUTEX_InitModule
 *
 * Revision 1.16  2002/02/11 06:40:13  slava
 * o check _USE_PTHREADS rather than _UNIX to find out whether we use
 *   Posix threads
 *
 * Revision 1.15  2001/12/14 09:01:22  slava
 * o (Win32) handle WAIT_ABANDONED status from WaitForSingleObject
 *
 * Revision 1.14  2001/11/25 21:25:13  slava
 * o InitModule functions now invoke dependent InitModule functions. That is,
 *   LOCK_InitModule invokes MUTEX_InitModule and EVENT_InitModule, etc.
 *
 * Revision 1.13  2001/11/24 19:39:19  slava
 * o cleaned up thread support for Unix platforms
 *
 * Revision 1.12  2001/11/23 22:45:35  slava
 * o better diagnostics if WaitForSingleObject() fails
 *
 * Revision 1.11  2001/09/14 07:38:16  slava
 * o changed Mutex definition for Win32, making it type safe
 *
 * Revision 1.10  2001/08/12 18:11:10  slava
 * o added MUTEX_Create() and MUTEX_Delete() functions
 *
 * Revision 1.9  2001/08/12 03:03:11  slava
 * o cleaned up debug trace and ASSERT messages
 *
 * Revision 1.8  2001/07/06 07:17:12  slava
 * o print some Win32 errors in hex
 *
 * Revision 1.7  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.6  2001/05/27 11:45:51  slava
 * o port to NT kernel mode environment
 *
 * Revision 1.5  2001/05/18 22:36:57  slava
 * o THIS_FILE variable is no longer being used
 *
 * Revision 1.4  2001/01/31 02:00:41  slava
 * o portability fixes. Now compiles on Solaris
 *
 * Revision 1.3  2000/11/01 13:25:05  slava
 * o replaced TRUE/FALSE with True/False
 *
 * Revision 1.2  2000/09/16 03:49:16  slava
 * o fixed compilation warnings in release build on Linux
 *
 * Revision 1.1  2000/08/19 04:48:58  slava
 * o initial checkin
 *
 *
 * Local Variables:
 * c-basic-offset:4
 * indent-tabs-mode: nil
 * End:
 */
