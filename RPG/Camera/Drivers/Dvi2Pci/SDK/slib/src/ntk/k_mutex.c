/*
 * $Id: k_mutex.c,v 1.3 2009/11/17 00:14:24 slava Exp $
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
 * Mutex API implementation for NT kernel mode environment
 */

#ifndef _NT_KERNEL
#  error "_NT_KERNEL must be defined to compile NT kernel code"
#endif

/**
 * Initialize a mutex. Returns False if initialization fails.
 */
Bool MUTEX_Init(Mutex * m)
{
    memset(m, 0, sizeof(*m));
    KeInitializeSpinLock(&m->spin);
    m->lock.type = &LockType_Mutex;
    return True;
}

/**
 * Destroy the mutex 
 */
void MUTEX_Destroy(Mutex * m)
{
    /* nothing to do */
    ASSERT(!m->acquired);
}

/**
 * Try to acquire the mutex. Returns True if mutex has been successfully
 * acquired, False otherwise.
 */
Bool MUTEX_TryLock(Mutex * m)
{
    /* TryToAcquireSpinLock functions are not exported from NT kernel */
    ASSERT(!m->acquired);
    if (!m->acquired) {
        return MUTEX_Lock(m);
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
    KIRQL irql = KeGetCurrentIrql();
    if (irql < DISPATCH_LEVEL) {
        KeAcquireSpinLock(&m->spin, &irql);
        m->dpc = False;
    } else {
        ASSERT(irql == DISPATCH_LEVEL);
        KeAcquireSpinLockAtDpcLevel(&m->spin);
        m->dpc = True;
    }
    ASSERT(!m->acquired);
    ASSERT(!m->thread);
    m->irql = irql;
    m->acquired = True;
    m->thread = KeGetCurrentThread();
    return True;
}

/**
 * Tests if mutex is locked by the current thread
 */
Bool MUTEX_IsLocked(const Mutex * m)
{
    return (m->thread == KeGetCurrentThread());
}

/**
 * Release the mutex.
 */
void MUTEX_Unlock(Mutex * m)
{
    ASSERT(m->acquired);
    ASSERT(m->thread == KeGetCurrentThread());
    ASSERT(KeGetCurrentIrql() == DISPATCH_LEVEL);
    if (m->acquired) {
        m->acquired = False;
        m->thread = NULL;
        if (m->dpc) {
            KeReleaseSpinLockFromDpcLevel(&m->spin);
        } else {
            KeReleaseSpinLock(&m->spin, m->irql);
        }
    }
}

/*
 * HISTORY:
 *
 * $Log: k_mutex.c,v $
 * Revision 1.3  2009/11/17 00:14:24  slava
 * o introducing generic synchronization API. This is a pretty destructive
 *   checkin, it's not 100% backward compatible. The Lock type is now a
 *   generic lock. What used to be called Lock is now RWLock. Sorry.
 *
 * Revision 1.2  2009/10/05 14:53:15  slava
 * o added MUTEX_IsLocked() function
 *
 * Revision 1.1  2005/02/19 00:37:41  slava
 * o separated platform specific code from platform independent code
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
