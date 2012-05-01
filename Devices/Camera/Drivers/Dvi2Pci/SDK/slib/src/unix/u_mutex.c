/*
 * $Id: u_mutex.c,v 1.3 2009/11/17 00:14:24 slava Exp $
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
 * Mutex API implementation for POSIX environment
 */

#ifndef _USE_PTHREADS
#  error "_USE_PTHREADS must be defined to compile this code"
#endif

/**
 * Initialize a mutex. Returns False if initialization fails.
 */
Bool MUTEX_Init(Mutex * m)
{
    int err;
    memset(m, 0, sizeof(*m));
    err = pthread_mutex_init(&m->mutex,NULL);
    if (!err) {
        m->lock.type = &LockType_Mutex;
        return True;
    }
    ASSMSG1("pthread_mutex_init() failed, error %d", err);
    return False;
}

/**
 * Destroy the mutex 
 */
void MUTEX_Destroy(Mutex * m)
{
#  if DEBUG
    int err = pthread_mutex_destroy(&m->mutex);
    if (err) ASSMSG1("pthread_mutex_destroy() failed, error %d", err);
#  else  /* ! DEBUG */
    pthread_mutex_destroy(&m->mutex);
#  endif /* ! DEBUG */
    ASSERT(!m->thread);
}

/**
 * Try to acquire the mutex. Returns True if mutex has been successfully
 * acquired, False otherwise.
 */
Bool MUTEX_TryLock(Mutex * m)
{
    int err = pthread_mutex_trylock(&m->mutex);
    switch (err) {
    case 0:
        ASSERT(!m->thread);
        m->thread = pthread_self();
        ASSERT(m->thread);
        return True;
    case EBUSY: return False;
    }
    ASSMSG1("pthread_mutex_trylock() failed, error %d", err);
    return False;
}

/**
 * Locks the mutex. If mutex is already locked, waits until it becomes
 * available. Returns True if mutex has been successfully acquired, 
 * False otherwise.
 */
Bool MUTEX_Lock(Mutex * m)
{
    int err = pthread_mutex_lock(&m->mutex);
    if (!err) {
        ASSERT(!m->thread);
        m->thread = pthread_self();
        ASSERT(m->thread);
        return True;
    }
    ASSMSG1("pthread_mutex_lock() failed, error %d", err);
    return False;
}

/**
 * Tests if mutex is locked by the current thread
 */
Bool MUTEX_IsLocked(const Mutex * m)
{
    return (m->thread == pthread_self());
}

/**
 * Release the mutex.
 */
void MUTEX_Unlock(Mutex * m)
{
#if DEBUG
    int err;
#endif /* DEBUG */
    if (m->thread == pthread_self()) m->thread = (pthread_t)0;
#if DEBUG
    err = pthread_mutex_unlock(&m->mutex);
    if (err) ASSMSG1("pthread_mutex_unlock() failed, error %d", err);
#else  /* ! DEBUG */
    pthread_mutex_unlock(&m->mutex);
#endif /* ! DEBUG */
}

/*
 * HISTORY:
 *
 * $Log: u_mutex.c,v $
 * Revision 1.3  2009/11/17 00:14:24  slava
 * o introducing generic synchronization API. This is a pretty destructive
 *   checkin, it's not 100% backward compatible. The Lock type is now a
 *   generic lock. What used to be called Lock is now RWLock. Sorry.
 *
 * Revision 1.2  2009/10/05 19:11:02  slava
 * o implemented MUTEX_IsLocked() for Unix
 *
 * Revision 1.1  2005/02/19 01:11:21  slava
 * o Unix specific code
 *
 * Local Variables:
 * c-basic-offset:4
 * indent-tabs-mode: nil
 * End:
 */
