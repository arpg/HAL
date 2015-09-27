/*
 * $Id: s_lock.h,v 1.14 2009/11/17 00:14:24 slava Exp $
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

#ifndef _SLAVA_LOCK_H_
#define _SLAVA_LOCK_H_

#include "s_def.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* Unified API for synchronization objects (mutex, read/write lock) */
typedef struct _LockType LockType;
typedef struct _Lock {
    const LockType * type;
} Lock;

extern void LOCK_InitModule P_((void));
extern void LOCK_Shutdown   P_((void));

extern Lock * LOCK_CreateMutex    P_((void));
extern Lock * LOCK_CreateRWLock   P_((void));
extern Lock * LOCK_CreateCritSect P_((void));

extern void LOCK_Delete     P_((Lock * lock));

extern Bool LOCK_Lock       P_((Lock * lock));
extern Bool LOCK_TryLock    P_((Lock * lock));
extern void LOCK_Unlock     P_((Lock * lock));
extern int  LOCK_Count      P_((const Lock * lock));
extern Str  LOCK_Name       P_((const Lock * lock));

#define LOCK_IsLocked(lock) (LOCK_Count(lock) > 0)

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_LOCK_H_ */

/*
 * HISTORY:
 *
 * $Log: s_lock.h,v $
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
