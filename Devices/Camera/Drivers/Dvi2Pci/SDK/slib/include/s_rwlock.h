/*
 * $Id: s_rwlock.h,v 1.14 2009/11/17 00:14:24 slava Exp $
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

#ifndef _SLAVA_RWLOCK_H_
#define _SLAVA_RWLOCK_H_

#include "s_def.h"
#include "s_queue.h"
#include "s_thread.h"
#include "s_event.h"
#include "s_mutex.h"

/*
 * PTHREAD implementation of read-write lock apparently assumes that 
 * locks cannot be acquired recursively (i.e. fail with EDEADLK if 
 * current thread already owns the read-write lock for writing or 
 * reading). This is why I have essentially re-implemented it. 
 * I believe that ability to acquire synchronization resources 
 * recursively significantly simplifies programming (although in 
 * same cases makes it easier to get into a deadlock situation).  
 * The other reason was that Win32 does not have anything like 
 * pthread's rwlock, i.e. I would pretty much have to re-implement 
 * it anyway if I want my code run under Windows (which I certainly do).
 */

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

typedef struct _RWEntry {
    ThrID  id;                  /* zero if entry is vacant */
    int write;                  /* write entry count */
    int read;                   /* read entry count */
} RWEntry;

typedef struct _RWLock {
    Lock lock;                  /* common for all synchronization objects */ 
    Mutex mutex;                /* protects this structure */

    Queue shareWaiters;         /* shared waiters */
    Queue exclusiveWaiters;     /* exclusive waiters */
    Queue waiterCache;          /* cached waiters */

    Event shareEvent;           /* event for all readers */
    Event exclusiveEvent;       /* event for the first writer */
    int   flags;                /* flags */
    int   locks;                /* total lock count */
    int   bypassCount;          /* n of times we let reader ahead of writer */
    long  contentions;          /* contention count, for debugging */

    int   numEntries;           /* total number of thread entries available */
    int   entriesInUse;         /* max entry index + 1 */
    int   entriesActive;        /* number of active entries */

    int   eventsInCache;        /* number of cached events */
    Event * eventCache[4];      /* cached writer events */

    RWEntry staticEntries[4];   /* statically allocated thread entries */
    RWEntry * moreEntries;      /* points to additional thread entries */
} RWLock;

/* type of access */
typedef enum _LockAccess {
    Lock_None,
    Lock_Read,
    Lock_Write
} LockAccess;

/* prototypes */
extern RWLock * RWLOCK_Create      P_((void));
extern Bool RWLOCK_Init          P_((RWLock * lock));
extern void RWLOCK_Delete        P_((RWLock * lock));
extern void RWLOCK_Destroy       P_((RWLock * lock));
extern Bool RWLOCK_ReadLock      P_((RWLock * lock));
extern Bool RWLOCK_TryReadLock   P_((RWLock * lock));
extern Bool RWLOCK_TimeReadLock  P_((RWLock * lock, long ms));
extern Bool RWLOCK_WriteLock     P_((RWLock * lock));
extern Bool RWLOCK_TryWriteLock  P_((RWLock * lock));
extern Bool RWLOCK_TimeWriteLock P_((RWLock * lock, long ms));
extern Bool RWLOCK_DropWrite     P_((RWLock * lock));
extern void RWLOCK_Unlock        P_((RWLock * lock));
extern void RWLOCK_UnlockMany    P_((RWLock * lock, int n));
extern Bool RWLOCK_CanRead       P_((const RWLock * lock));
extern Bool RWLOCK_CanWrite      P_((const RWLock * lock));
extern int  RWLOCK_GetLockCount  P_((const RWLock * lock));
extern LockAccess RWLOCK_GetAccess P_((const RWLock * lock));

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_LOCK_H_ */

/*
 * HISTORY:
 *
 * $Log: s_rwlock.h,v $
 * Revision 1.14  2009/11/17 00:14:24  slava
 * o introducing generic synchronization API. This is a pretty destructive
 *   checkin, it's not 100% backward compatible. The Lock type is now a
 *   generic lock. What used to be called Lock is now RWLock. Sorry.
 *
 * Revision 1.13  2008/05/10 11:01:54  slava
 * o removed LockEntry::handle field. It no longer makes any sense after the
 *   recent changes in Win32 thread API implementation
 *
 * Revision 1.12  2007/05/01 00:31:27  slava
 * o added LOCK_DropWrite() function
 *
 * Revision 1.11  2003/02/01 05:37:54  slava
 * o the goal of this patch is to make the behavior of the lock more
 *   predictable and less dependent on the underlying OS scheduler.
 *   The waiters get the lock in the order they started to wait; some
 *   exceptions apply to the waiters for shared access.
 *
 * Revision 1.10  2003/01/20 19:02:46  slava
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
 * Revision 1.9  2002/12/17 04:01:35  slava
 * o fixed a typo
 *
 * Revision 1.8  2002/08/12 05:13:27  slava
 * o use (void) rather than () to declare functions that have
 *   no arguments. The problem with () is that compilers interpret
 *   it as an *unspecified* argument list, but (void) clearly
 *   indicates that function takes no arguments. Improves compile
 *   time error checking
 *
 * Revision 1.7  2002/06/24 05:14:28  slava
 * o added LOCK_Create and LOCK_Delete
 *
 * Revision 1.6  2002/06/18 01:29:42  slava
 * o LOCK_CanRead, LOCK_CanWrite, LOCK_GetLockCount and LOCK_GetAccess
 *   take const pointer as a parameter
 *
 * Revision 1.5  2001/12/06 04:31:12  slava
 * o added LOCK_UnlockMany() function
 * o changed LockAccess enum type to avoid conflict with LOCK_WRITE constant
 *   defined in one of Windows header files
 *
 * Revision 1.4  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.3  2001/05/18 22:29:54  slava
 * o slib has been (partially) ported to Windows CE
 *
 * Revision 1.2  2000/12/17 16:09:47  slava
 * o added extern "C" {} so that slib could be used from a C++ program
 *
 * Revision 1.1  2000/08/19 04:48:58  slava
 * o initial checkin
 *
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
