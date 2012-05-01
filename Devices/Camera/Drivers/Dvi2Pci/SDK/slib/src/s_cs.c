/*
 * $Id: s_cs.c,v 1.2 2009/11/17 00:23:08 slava Exp $
 *
 * Copyright (C) 2009 by Slava Monich
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

#include "s_cs.h"
#include "s_lockp.h"
#include "s_mem.h"

/* 
 * Synchronization object descriptor
 */
STATIC const LockType LockType_CritSect;

/**
 * Creates a critical section. This function is defined here (rather than
 * in s_lock.c) to avoid unnecessary linkage with the lock types that you
 * are not using.
 */
Lock * LOCK_CreateCritSect()
{
    CritSect * cs = CS_Create();
    return cs ? &cs->lock : NULL;
}

/**
 * Creates new critical section
 */
CritSect * CS_Create()
{
    CritSect * cs = MEM_New(CritSect);
    if (cs) {
        if (CS_Init(cs)) {
            return cs;
        }
        MEM_Free(cs);
    }
    return NULL;
}

/**
 * Deletes the critical section
 */
void CS_Delete(CritSect * cs)
{
    if (cs) {
        CS_Destroy(cs);
        MEM_Free(cs);
    }
}

/**
 * Initializes the critical section
 */
Bool CS_Init(CritSect * cs)
{
    memset(cs, 0, sizeof(*cs));
    if (MUTEX_Init(&cs->mutex)) {
        cs->lock.type = &LockType_CritSect;
        return True;
    }
    return False;
}

/**
 * Destroys the critical section
 */
void CS_Destroy(CritSect * cs)
{
    ASSERT(!cs->count);
    MUTEX_Destroy(&cs->mutex);
}

/**
 * Attempts to acquire the critical section without waiting
 */
Bool CS_TryLock(CritSect * cs)
{
    if (MUTEX_IsLocked(&cs->mutex) || MUTEX_TryLock(&cs->mutex)) {
        cs->count++;
        return True;
    }
    return False;
}

/**
 * Acquires the critical section
 */
Bool CS_Lock(CritSect * cs)
{
    if (MUTEX_IsLocked(&cs->mutex) || MUTEX_Lock(&cs->mutex)) {
        cs->count++;
        return True;
    }
    return False;
}

/**
 * Checks how many time the current thread locked the critical section.
 * Returns zero if the current thread doesn't own the critical section.
 */
int CS_LockCount(const CritSect * cs)
{
    if (MUTEX_IsLocked(&cs->mutex)) {
        ASSERT(cs->count > 0);
        return cs->count;
    }
    return 0;
}

/**
 * Releases the critical section
 */
void CS_Unlock(CritSect * cs)
{
    ASSERT(MUTEX_IsLocked(&cs->mutex));
    if (MUTEX_IsLocked(&cs->mutex)) {
        ASSERT(cs->count > 0);
        if ((--cs->count) == 0) {
            MUTEX_Unlock(&cs->mutex);
        }
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

#define _CS_CastLock(s) CAST(s,CritSect,lock)
#if DEBUG
STATIC CritSect * CS_CastLock(Lock * lock) {
    ASSERT(lock && lock->type == &LockType_CritSect);
    return _CS_CastLock(lock);
}
STATIC const CritSect * CS_CastLockC(const Lock * lock) {
    ASSERT(lock && lock->type == &LockType_CritSect);
    return _CS_CastLock(lock);
}
#else
#  define CS_CastLock(l)  _CS_CastLock(l)
#  define CS_CastLockC(l) _CS_CastLock(l)
#endif

STATIC Bool _CS_Lock(Lock * l)       { return CS_Lock(CS_CastLock(l)); }
STATIC Bool _CS_TryLock(Lock * l)    { return CS_TryLock(CS_CastLock(l)); }
STATIC int  _CS_Count(const Lock* l) { return CS_LockCount(CS_CastLockC(l)); }
STATIC void _CS_Unlock(Lock * l)     { CS_Unlock(CS_CastLock(l)); }
STATIC void _CS_Free(Lock * l)       { CS_Delete(CS_CastLock(l)); }

STATIC const LockType LockType_CritSect = {
    TEXT("CritSect")    /* name     */,
    _CS_Lock            /* lock     */,
    _CS_TryLock         /* trylock  */,
    _CS_Count           /* count    */,
    _CS_Unlock          /* unlock   */,
    _CS_Free            /* free     */
};

/*
 * HISTORY:
 *
 * $Log: s_cs.c,v $
 * Revision 1.2  2009/11/17 00:23:08  slava
 * o fixed gcc compilation errors
 *
 * Revision 1.1  2009/11/17 00:14:24  slava
 * o introducing generic synchronization API. This is a pretty destructive
 *   checkin, it's not 100% backward compatible. The Lock type is now a
 *   generic lock. What used to be called Lock is now RWLock. Sorry.
 *
 * Local Variables:
 * c-basic-offset:4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
