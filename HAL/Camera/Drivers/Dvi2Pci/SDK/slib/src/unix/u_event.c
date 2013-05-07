/*
 * $Id: u_event.c,v 1.2 2008/07/19 11:34:35 slava Exp $
 *
 * Copyright (C) 2000-2008 by Slava Monich
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

#include "s_event.h"

/**
 * Event API implementation for POSIX environment
 */

#ifndef _USE_PTHREADS
#  error "_USE_PTHREADS must be defined to compile this code"
#endif

/**
 * Initialize a notification event. Returns False if initialization fails.
 */
Bool EVENT_Init(Event * e) 
{
    pthread_mutex_init(&e->posix.mut,NULL);
    pthread_cond_init(&e->posix.cond, NULL);
    e->state = EVENT_NOT_SIGNALED;
    return True;
}

/**
 * Destroy the notification event 
 */
void EVENT_Destroy(Event * e) 
{
    pthread_mutex_destroy(&e->posix.mut);
    pthread_cond_destroy(&e->posix.cond);
}

/**
 * Notify all waiters and switch into signalled state.
 */
Bool EVENT_Set(Event * e) 
{
    pthread_mutex_lock(&e->posix.mut);
    if (e->state != EVENT_SIGNALED) {
        pthread_cond_broadcast(&e->posix.cond);
    }
    e->state = EVENT_SIGNALED;
    pthread_mutex_unlock(&e->posix.mut);
    return True;
}

/**
 * Notify all waiters and reset state to non-signaled
 */
Bool EVENT_Pulse(Event * e) 
{
    pthread_mutex_lock(&e->posix.mut);
    if (e->state != EVENT_SIGNALED) {
        pthread_cond_broadcast(&e->posix.cond);
    }
    e->state = EVENT_NOT_SIGNALED;
    pthread_mutex_unlock(&e->posix.mut);
    return True;
}

/**
 * Reset state to non-signaled.
 */
Bool EVENT_Reset(Event * e) 
{
    pthread_mutex_lock(&e->posix.mut);
    e->state = EVENT_NOT_SIGNALED;
    pthread_mutex_unlock(&e->posix.mut);    
    return True;
}

/**
 * Returns the current state, EVENT_SIGNALED or EVENT_NOT_SIGNALED, 
 * of a given event object.
 */
EventState EVENT_State(Event * e)
{
    return e->state;
}

/**
 * Wait for event to become signaled.
 */
WaitState EVENT_Wait(Event * e) 
{
    pthread_mutex_lock(&e->posix.mut);
    if (e->state != EVENT_SIGNALED) {
        pthread_cond_wait(&e->posix.cond, &e->posix.mut);
    }
    pthread_mutex_unlock(&e->posix.mut);
    return WAIT_STATE_OK;
}

/**
 * Wait for event to become signaled.
 */
WaitState EVENT_TimeWait(Event * e, long ms) 
{
    if (ms < 0) {
        return EVENT_Wait(e);
    } else {
        WaitState state = WAIT_STATE_OK;
        pthread_mutex_lock(&e->posix.mut);
        if (e->state != EVENT_SIGNALED) {
            int err;
            struct timeval tv;
            struct timezone tz;
            struct timespec t;
            gettimeofday(&tv, &tz);
            tv.tv_usec += (ms%1000)*1000;               /* microseconds */
            t.tv_nsec = (tv.tv_usec % 1000000) * 1000;  /* nanoseconds */
            t.tv_sec = tv.tv_sec + ms/1000 + tv.tv_usec/1000000;
            err = pthread_cond_timedwait(&e->posix.cond, &e->posix.mut, &t);
            switch (err) {
            case 0: break;
            case ETIMEDOUT: 
                state = WAIT_STATE_TIMEOUT; 
                break;
            default: 
                TRACE1("pthread_cond_timedwait() failed, error %d\n",err);
                state = WAIT_STATE_ERROR;
                break;
            }
        }
        pthread_mutex_unlock(&e->posix.mut);
        return state;    
    }
}

/**
 * Similar to EVENT_TimeWait, only timeout is expressed in nanoseconds.
 */
WaitState EVENT_NanoWait(Event * e, I64s nsec) 
{
    if (nsec < 0) {
        return EVENT_Wait(e);
    } else {
        WaitState state = WAIT_STATE_OK;
        pthread_mutex_lock(&e->posix.mut);
        if (e->state != EVENT_SIGNALED) {
            int err;
            struct timeval tv;
            struct timezone tz;
            struct timespec t;
            gettimeofday(&tv, &tz);
            t.tv_sec = tv.tv_sec+tv.tv_usec/1000000+(long)(nsec/1000000000);
            t.tv_nsec = (tv.tv_usec% 1000000)*1000+(long)(nsec%1000000000);
            t.tv_sec += (t.tv_nsec/1000000000);
            t.tv_nsec %= 1000000000;
            err = pthread_cond_timedwait(&e->posix.cond, &e->posix.mut, &t);
            switch (err) {
            case 0: break;
            case ETIMEDOUT: 
                state = WAIT_STATE_TIMEOUT; 
                break;
            default: 
                TRACE1("pthread_cond_timedwait() failed, error %d\n",err);
                state = WAIT_STATE_ERROR;
                break;
            }
        }
        pthread_mutex_unlock(&e->posix.mut);
        return state;    
    }
}

/*
 * HISTORY:
 *
 * $Log: u_event.c,v $
 * Revision 1.2  2008/07/19 11:34:35  slava
 * o added Posix-specific EVENT_NanoWait function
 *
 * Revision 1.1  2005/02/19 01:11:21  slava
 * o Unix specific code
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
