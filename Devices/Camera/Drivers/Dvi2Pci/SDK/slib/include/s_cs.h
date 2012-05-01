/*
 * $Id: s_cs.h,v 1.1 2009/11/17 00:14:24 slava Exp $
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

#ifndef _SLAVA_CS_H_
#define _SLAVA_CS_H_

#include "s_mutex.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* Critical section (recursive mutex) */
typedef struct _CritSect {
    Lock lock;    /* common part for all synchronization objects */
    Mutex mutex;  /* non-recursive mutex */
    int count;    /* how many times mutex is acquired by the current thread */
} CritSect;

extern CritSect * CS_Create P_((void));
extern void CS_Delete P_((CritSect * cs));
extern Bool CS_Init P_((CritSect * cs));
extern void CS_Destroy P_((CritSect * cs));
extern Bool CS_TryLock P_((CritSect * cs));
extern Bool CS_Lock P_((CritSect * cs));
extern int  CS_LockCount P_((const CritSect * cs));
extern void CS_Unlock P_((CritSect * cs));

#define CS_IsLocked(cs) (CS_LockCount(cs) > 0)

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_CS_H_ */

/*
 * HISTORY:
 *
 * $Log: s_cs.h,v $
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
