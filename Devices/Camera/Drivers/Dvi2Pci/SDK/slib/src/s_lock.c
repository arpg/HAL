/*
 * $Id: s_lock.c,v 1.36 2009/11/17 00:43:52 slava Exp $
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

#include "s_lock.h"
#include "s_lockp.h"
#include "s_thread.h"
#include "s_mutex.h"
#include "s_rwlock.h"
#include "s_cs.h"

/*
 * LOCK_InitModule() increments this counter
 */
STATIC int LOCK_initcount = 0;

/**
 * Initialize the module
 */
void LOCK_InitModule() 
{
    if ((LOCK_initcount++) == 0) {
        THREAD_InitModule(); 
    }
}

/**
 * Checks that LOCK_InitModule has been called
 */
void LOCK_InitCheck()
{
    ASSERT(LOCK_initcount > 0);
    if (LOCK_initcount == 0) LOCK_InitModule();
}

/**
 * Deinitialize the module
 */
void LOCK_Shutdown() 
{
    if ((--LOCK_initcount) == 0) {
        THREAD_Shutdown(); 
    }
}

/*==========================================================================*
 *  Generic API for synchronization objects
 *==========================================================================*/

/**
 * Delete the synchronization object
 */
void LOCK_Delete(Lock * lock)
{
    ASSERT(!lock || lock->type);
    if (lock && lock->type) {
        LockFree f = lock->type->free;
        ASSERT(!LOCK_IsLocked(lock));
        f(lock);
    }
}

/**
 * Synchronization primitives
 */
Bool LOCK_Lock(Lock * lock)
{
    ASSERT(!lock || lock->type);
    return (lock && lock->type) ? lock->type->lock(lock) : False;
}

Bool LOCK_TryLock(Lock * lock)
{
    ASSERT(!lock || lock->type);
    return (lock && lock->type) ? lock->type->trylock(lock) : False;
}

void LOCK_Unlock(Lock * lock)
{
    ASSERT(!lock || lock->type);
    if (lock && lock->type) lock->type->unlock(lock);
}

int LOCK_Count(const Lock * lock)
{
    ASSERT(!lock || lock->type);
    return (lock && lock->type) ? lock->type->count(lock) : 0;
}

Str LOCK_Name(const Lock * lock)
{
    ASSERT(!lock || lock->type);
    return (lock && lock->type) ? lock->type->name : TEXT("???");
}

/*
 * HISTORY:
 *
 * $Log: s_lock.c,v $
 * Revision 1.36  2009/11/17 00:43:52  slava
 * o fixed Unicode build
 *
 * Revision 1.35  2009/11/17 00:23:08  slava
 * o fixed gcc compilation errors
 *
 * Revision 1.34  2009/11/17 00:14:24  slava
 * o introducing generic synchronization API. This is a pretty destructive
 *   checkin, it's not 100% backward compatible. The Lock type is now a
 *   generic lock. What used to be called Lock is now RWLock. Sorry.
 *
 * Revision 1.33  2008/05/10 11:01:54  slava
 * o removed LockEntry::handle field. It no longer makes any sense after the
 *   recent changes in Win32 thread API implementation
 *
 * Revision 1.32  2007/12/01 15:34:48  slava
 * o fixed stupid typo in internal flag name
 *
 * Revision 1.31  2007/05/01 00:31:27  slava
 * o added LOCK_DropWrite() function
 *
 * Revision 1.30  2005/02/20 20:31:29  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.29  2004/04/08 12:21:42  slava
 * o made it compilable with C++ compiler
 *
 * Revision 1.28  2003/05/21 00:11:04  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.27  2003/02/02 01:26:40  slava
 * o (hopefully) fixed the race condition which almost never occurs on
 *   Windows with its brain dead round robin scheduling, but was quite
 *   common on Unix. say, thread A owns the lock and thread B is waiting
 *   for it. thread A releases the lock and then almost immediately
 *   attempts to get it back. the old implementation did not guarantee
 *   that thread B would grab the lock before thread A gets it. not good.
 *   this patch should fix this problem, i.e. thread A in the described
 *   situation would put itself to the end of the waiters queue and wait
 *   for its turn to run.
 *
 * Revision 1.26  2003/02/01 05:37:54  slava
 * o the goal of this patch is to make the behavior of the lock more
 *   predictable and less dependent on the underlying OS scheduler.
 *   The waiters get the lock in the order they started to wait; some
 *   exceptions apply to the waiters for shared access.
 *
 * Revision 1.25  2003/01/24 02:38:19  slava
 * o cleaned up ASSERT messages (don't need newline)
 *
 * Revision 1.24  2003/01/20 19:02:46  slava
 * o changed XXX_InitModule and XXX_Shutdown (should be XXX_DeinitModule?)
 *   functions such that they can be called more than once. Obviously,
 *   only the first XXX_InitModule and the last XXX_Shutdown calls will
 *   actually do something; all other calls merely increment/decrement
 *   the "reference count". This addresses the problem of initializing/
 *   deinitializing slib in case if it's being used by multiple static
 *   libraries which may (but don't have to) be compiled into a single
 *   application. The caller of each XXX_InitModule function is now
 *   responsible for calling the respective XXX_Shutdown function when
 *   it no longer needs the services provided by the XXX module. The
 *   access to the internal "reference count" of each module is not
 *   synchronized, meaning that all the XXX_InitModule and XXX_Shutdown
 *   calls must be made under the circumstances that make race conditions
 *   impossible
 *
 * Revision 1.23  2002/07/15 17:49:11  slava
 * o made timeout calculation a little bit more accurate (mark the start
 *   of wait before rather than after MUTEX_Lock call)
 *
 * Revision 1.22  2002/06/24 05:14:28  slava
 * o added LOCK_Create and LOCK_Delete
 *
 * Revision 1.21  2002/06/18 01:29:42  slava
 * o LOCK_CanRead, LOCK_CanWrite, LOCK_GetLockCount and LOCK_GetAccess
 *   take const pointer as a parameter
 *
 * Revision 1.20  2002/05/29 08:29:49  slava
 * o removed EVENT_InitModule and MUTEX_InitModule functions
 *
 * Revision 1.19  2002/02/03 01:46:59  slava
 * o invoke THREAD_InitModule() from LOCK_InitModule()
 *
 * Revision 1.18  2001/12/21 01:49:53  slava
 * o fixed compilation warnings in release build
 *
 * Revision 1.17  2001/12/06 04:31:11  slava
 * o added LOCK_UnlockMany() function
 * o changed LockAccess enum type to avoid conflict with LOCK_WRITE constant
 *   defined in one of Windows header files
 *
 * Revision 1.16  2001/11/25 21:25:13  slava
 * o InitModule functions now invoke dependent InitModule functions. That is,
 *   LOCK_InitModule invokes MUTEX_InitModule and EVENT_InitModule, etc.
 *
 * Revision 1.15  2001/11/24 19:39:19  slava
 * o cleaned up thread support for Unix platforms
 *
 * Revision 1.14  2001/10/08 05:17:51  slava
 * o eliminated some strict gcc warnings (mostly shadow declarations)
 *
 * Revision 1.13  2001/09/16 17:00:18  slava
 * o fixed some comments
 *
 * Revision 1.12  2001/08/14 03:24:11  slava
 * o cleanup
 *
 * Revision 1.11  2001/08/12 03:04:37  slava
 * o wait with timeout was broken. fixed it (hopefully)
 *
 * Revision 1.10  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.9  2001/05/18 22:36:57  slava
 * o THIS_FILE variable is no longer being used
 *
 * Revision 1.8  2001/05/18 22:29:54  slava
 * o slib has been (partially) ported to Windows CE
 *
 * Revision 1.7  2001/05/03 20:11:50  slava
 * o fixed a bug (a rare race condition between findStaticLockEntry() and
 *   LOCK_Unlock() code, which was introduced in rev 1.6). See comments
 *   in LOCK_Unlock().
 *
 * Revision 1.6  2001/03/27 06:08:17  slava
 * o unsynchronized some operations to improve performance
 *
 * Revision 1.5  2001/01/31 02:00:41  slava
 * o portability fixes. Now compiles on Solaris
 *
 * Revision 1.4  2000/11/01 13:25:05  slava
 * o replaced TRUE/FALSE with True/False
 *
 * Revision 1.3  2000/09/22 01:25:00  slava
 * o fixed a problem with Microsoft Visual C++ 6.0 generating bad code
 *   when optimization is on (had to replace macro with a function) -
 *   looks like a bug in Microsoft's compiler
 *
 * Revision 1.2  2000/08/21 10:41:50  slava
 * o more diagnostics in debug build
 *
 * Revision 1.1  2000/08/19 04:48:58  slava
 * o initial checkin
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
