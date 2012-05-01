/*
 * $Id: s_lockp.h,v 1.2 2009/11/17 00:23:08 slava Exp $
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

#ifndef _SLAVA_LOCK_PRIVATE_H_
#define _SLAVA_LOCK_PRIVATE_H_

#include "s_lock.h"

/* 
 * LockLock     - locks the resource for exclusive use by the current thread
 * LockTryLock  - attempts to lock the resource without waiting
 * LockCount    - returns the nesting count for the current thread
 * LockUnlock   - releases the lock
 * LockFree     - deletes the lock
 */
typedef Bool (*LockLock)     P_((Lock * lock));
typedef Bool (*LockTryLock)  P_((Lock * lock));
typedef int  (*LockCount)    P_((const Lock * lock));
typedef void (*LockUnlock)   P_((Lock * lock));
typedef void (*LockFree)     P_((Lock * lock));

/* data structures */
struct _LockType {          /* type of synchronization object */
    Str          name;      /* name of the object for debugging purposes */
    LockLock     lock;      /* locks the resource for exclusive use */
    LockTryLock  trylock;   /* attempts to lock without waiting */
    LockCount    count;     /* nesting count for the current thread */
    LockUnlock   unlock;    /* releases the lock */
    LockFree     free;      /* deletes the lock */
};

/* Checks that LOCK_InitModule has been called */
extern void LOCK_InitCheck P_((void));

/* Mutex descriptor (accessed from platform-specific code) */
extern const LockType LockType_Mutex; 

#endif /* _SLAVA_LOCK_PRIVATE_H_ */

/*
 * HISTORY:
 *
 * $Log: s_lockp.h,v $
 * Revision 1.2  2009/11/17 00:23:08  slava
 * o fixed gcc compilation errors
 *
 * Revision 1.1  2009/11/17 00:14:24  slava
 * o introducing generic synchronization API. This is a pretty destructive
 *   checkin, it's not 100% backward compatible. The Lock type is now a
 *   generic lock. What used to be called Lock is now RWLock. Sorry.
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
