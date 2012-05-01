/*
 * $Id: k_event.c,v 1.1 2005/02/19 00:37:41 slava Exp $
 *
 * Copyright (C) 2000-2005 by Slava Monich
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
 * Event API implementation for NT kernel mode environment
 */

#ifndef _NT_KERNEL
#  error "_NT_KERNEL must be defined to compile NT kernel code"
#endif

/**
 * Initialize a notification event. Returns False if initialization fails.
 */
Bool EVENT_Init(Event * e) 
{
    /* 
     * According to the Windows DDK documentation, the callers of 
     * KeInitializeEvent must be running at IRQL PASSIVE_LEVEL. 
     * I strongly believe it's a mistake in the documentation, because
     * KeInitializeEvent does not do anything that would require the
     * current thread to wait or to access pageable memory, unless the
     * event is allocated from paged pool or the KeInitializeEvent code
     * itself resides in a pageable code segment (which does not seem
     * to be the case). Anyway, that's why this ASSERT is here.
     */
    ASSERT(KeGetCurrentIrql() == PASSIVE_LEVEL);
    memset(e, 0, sizeof(*e));
    KeInitializeEvent(&e->kevent, NotificationEvent, FALSE);
    e->pinc = SEMAPHORE_INCREMENT;
    return True;
}

/**
 * Destroy the notification event 
 */
void EVENT_Destroy(Event * e) 
{
    /* nothing to do */
}

/**
 * Notify all waiters and switch into signalled state.
 */
Bool EVENT_Set(Event * e) 
{
    ASSERT(KeGetCurrentIrql() <= DISPATCH_LEVEL);
    KeSetEvent(&e->kevent, e->pinc, FALSE);
    return True;
}

/**
 * Notify all waiters and reset state to non-signaled
 */
Bool EVENT_Pulse(Event * e) 
{
    return EVENT_Set(e) && EVENT_Reset(e);
}

/**
 * Reset state to non-signaled.
 */
Bool EVENT_Reset(Event * e) 
{
    ASSERT(KeGetCurrentIrql() <= DISPATCH_LEVEL);
    KeClearEvent(&e->kevent);
    return True;
}

/**
 * Returns the current state, EVENT_SIGNALED or EVENT_NOT_SIGNALED, 
 * of a given event object.
 */
EventState EVENT_State(Event * e)
{
    ASSERT(KeGetCurrentIrql() <= DISPATCH_LEVEL);
    if (KeReadStateEvent(&e->kevent)) {
        return EVENT_SIGNALED;
    } else {
        return EVENT_NOT_SIGNALED;
    }
}

/**
 * Wait for event to become signaled.
 */
WaitState EVENT_Wait(Event * e) 
{
    ASSERT(KeGetCurrentIrql() == PASSIVE_LEVEL);
    if (KeGetCurrentIrql() == PASSIVE_LEVEL) {
        NTSTATUS err;
        err = KeWaitForSingleObject(&e->kevent,Executive,KernelMode,0,NULL);
        switch (err) {
        case STATUS_SUCCESS:  return WAIT_STATE_OK;
        case STATUS_TIMEOUT:  return WAIT_STATE_TIMEOUT;
        case STATUS_ALERTED:
        case STATUS_USER_APC:
            ASSMSG("STATUS_ALERTED and STATUS_USER_APC not handled properly");
        default:
            return WAIT_STATE_ERROR;
        }
    } else {
        return WAIT_STATE_ERROR;
    }
}

/**
 * Wait for event to become signaled.
 */
WaitState EVENT_TimeWait(Event * e, long ms) 
{
    if (ms < 0) {
        return EVENT_Wait(e);
    } else {
        /* only can wait at PASSIVE_LEVEL */
        KIRQL irql = KeGetCurrentIrql();
        if (irql == PASSIVE_LEVEL || (irql == DISPATCH_LEVEL && ms == 0)) {
            NTSTATUS err;
            LARGE_INTEGER t;
            t = RtlConvertLongToLargeInteger(ms);
            t = RtlExtendedIntegerMultiply(t, 10000);
            t = RtlLargeIntegerNegate(t);   /* relative time */
            err = KeWaitForSingleObject(&e->kevent,Executive,KernelMode,0,&t);
            switch (err) {
            case STATUS_SUCCESS:  return WAIT_STATE_OK;
            case STATUS_TIMEOUT:  return WAIT_STATE_TIMEOUT;
            case STATUS_ALERTED:
            case STATUS_USER_APC:
                ASSMSG("STATUS_ALERTED/STATUS_USER_APC not handled properly");
            default:
                return WAIT_STATE_ERROR;
            }
        } else {
            ASSMSG2("Wrong IRQL(%d) in EVENT_TimeWait(%ld)",(int)irql,ms);
            return WAIT_STATE_ERROR;
        }
    }
}

/*
 * HISTORY:
 *
 * $Log: k_event.c,v $
 * Revision 1.1  2005/02/19 00:37:41  slava
 * o separated platform specific code from platform independent code
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
