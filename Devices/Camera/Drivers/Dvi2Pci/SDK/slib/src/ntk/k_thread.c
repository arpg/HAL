/*
 * $Id: k_thread.c,v 1.2 2009/10/21 13:22:09 slava Exp $
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

#include "s_libp.h"
#include "s_thrp.h"
#include "s_queue.h"
#include "s_mutex.h"
#include "s_event.h"
#include "s_mem.h"

/**
 * Thread API implementation for NT kernel mode environment
 */

#ifndef _NT_KERNEL
#  error "_NT_KERNEL must be defined to compile NT kernel code"
#endif

typedef DWORD ThrKeyType;
struct _ThrID {
   QEntry entry;        /* entry in global thr_list (defined below) */
   Event start;         /* thread function waits for this event */
   PKTHREAD throbj;     /* pointer to kernel mode thread object */
   int flags;           /* flags, see below */

#  define THR_OK       0x01 /* thread has been initialized successfully */
#  define THR_FINISHED 0x02 /* thread has been removed from the list */
#  define THR_DETACHED 0x04 /* thread has been detached */
};

static Queue thr_list;
static Mutex thr_mutex;

/*
 * Thread start context
 */
typedef struct _ThrStart {
    ThrProc proc;
    void *  arg;
    ThrID thrid;
} ThrStart;

/**
 * Deallocates the _ThrID structure. The structure must have been removed 
 * from the global list
 */
STATIC void NT_DeleteThrID(ThrID thrid)
{
    if (thrid) {
        ObDereferenceObject(thrid->throbj);
        EVENT_Destroy(&thrid->start);
        MEM_Free(thrid);
    }
}

/**
 * NT-specific initialization
 */
void THREAD_PlatformInitialize()
{
    QUEUE_Init(&thr_list);
    MUTEX_Init(&thr_mutex);
}

/**
 * NT-specific deinitialization
 */
void THREAD_PlatformShutdown()
{
    ASSERT(QUEUE_IsEmpty(&thr_list));
    MUTEX_Destroy(&thr_mutex);
}

/**
 * NT-specific thread proc
 */
STATIC VOID NT_ThreadProc(PVOID arg) 
{
    ThrStart * context = (ThrStart*)arg;
    ThrID thrid = context->thrid;
    TRACE1("SLIB: kernel thread %08lX starting\n",KeGetCurrentThread());
    EVENT_Wait(&thrid->start);
    if (thrid->flags & THR_OK) {
        ThrProc proc;
        void * param;
        ASSERT(KeGetCurrentThread() == thrid->throbj);
        TRACE1("SLIB: kernel thread %08lX started\n",thrid->throbj);

#ifdef _USE_EXCEPTION_HANDLING
        __try { 
#endif /* _USE_EXCEPTION_HANDLING */

        /* copy the data and deallocate the context */
        proc = context->proc;
        param = context->arg;
        MEM_Free(context);
        (*proc)(param);

#ifdef _USE_EXCEPTION_HANDLING
        } __except(EXCEPTION_EXECUTE_HANDLER) {
            ASSMSG1("EXCEPTION %08lX in thread proc!",GetExceptionCode());
        }
#endif /* _USE_EXCEPTION_HANDLING */

        TRACE1("SLIB: kernel thread %08lX exiting\n",thrid->throbj);
        MUTEX_Lock(&thr_mutex);
        QUEUE_RemoveEntry(&thrid->entry);
        if (thrid->flags & THR_DETACHED) {
            NT_DeleteThrID(thrid);
        } else {
            thrid->flags |= THR_FINISHED;
        }
        MUTEX_Unlock(&thr_mutex);
    } else {
        /* exit immediately */
        TRACE1("SLIB: kernel thread %08lX is not OK\n",thrid->throbj);
    }
}

/**
 * Create a new thread, returning True on success
 */
Bool THREAD_Create(ThrID* id, ThrProc proc, void * arg)
{
    ThrStart* start = NULL;
    Bool success = False;
    ASSERT(THREAD_IsInited());

    if (id) *id = (ThrID)0;
    start = MEM_New(ThrStart);
    if (start) {
        start->proc = proc;
        start->arg = arg;
        start->thrid = MEM_New(struct _ThrID);
        if (start->thrid) {
            HANDLE handle;
            NTSTATUS status;
            OBJECT_ATTRIBUTES att;
            ThrID thrid = start->thrid;
            RtlZeroMemory(thrid, sizeof(*thrid));
            EVENT_Init(&thrid->start);
            InitializeObjectAttributes(&att,NULL,OBJ_KERNEL_HANDLE,NULL,NULL);
            status = PsCreateSystemThread(&handle, STANDARD_RIGHTS_REQUIRED,
                &att, NULL,  NULL, NT_ThreadProc, start);

            if (NT_SUCCESS(status)) {

                /* obtain pointer to the thread object */
                status = ObReferenceObjectByHandle(handle, 0, *PsThreadType,
                    KernelMode, &thrid->throbj, NULL);

                if (NT_SUCCESS(status)) {

                    /* add it to the list of known threads */
                    MUTEX_Lock(&thr_mutex);
                    QUEUE_InsertTail(&thr_list, &thrid->entry);
                    thrid->flags |= THR_OK;
                    MUTEX_Unlock(&thr_mutex);

                    /* resume the thread */
                    KeSetPriorityThread(thrid->throbj, LOW_REALTIME_PRIORITY);
                    EVENT_Set(&thrid->start);
                    if (id) *id = thrid;
                    success = True;

                } else {

                    /* resume the created thread and wait for it to exit */
                    TRACE1("SLIB: cannot get thread obj, err %08lX\n",status);
                    EVENT_Set(&thrid->start);
                    ZwWaitForSingleObject(handle, FALSE, NULL);

                    /* deallocate the thread id structure */
                    MEM_Free(thrid);
                }
                
                /* we don't need the handle because it's process-specific */
                ZwClose(handle);

            } else {
                TRACE1("SLIB: failed to create thread, err %08lX\n",status);
            }
        }

        /*
         * if we have failed to start the thread, deallocate the context
         */
        if (!success) MEM_Free(start);
    }

    return success;
}

/**
 * Returns the thread ID of the calling thread. 
 */
ThrID THREAD_Self()
{
    QEntry * e;
    ThrID self = NULL;
    PKTHREAD thisThread = KeGetCurrentThread();
    ASSERT(THREAD_IsInited());

    MUTEX_Lock(&thr_mutex);
    for (e = QUEUE_First(&thr_list); e; e = QUEUE_Next(e)) {
        ThrID id = QCAST(e, struct _ThrID, entry);
        if (id->throbj == thisThread) {
            self = id;
            break;
        }
    }
    MUTEX_Unlock(&thr_mutex);
    
    /*
     * Note that in NT kernel mode environment THREAD_Self() only works
     * in context of a thread created by THREAD_Create() call. There's no
     * obvious way to implement that in context of an arbitrary thread 
     * without leaking memory or kernel object references.
     */
    ASSERT(self);
    return self;
}

/**
 * Checks whether the specified ThrID represents the current thread
 */
Bool THREAD_IsSelf(ThrID tid)
{
    return tid && tid->throbj == KeGetCurrentThread();
}

/**
 * This call essentially invalidates ThrID variable. 
 * Detached thread cannot be "joined" (POSIX semantics)
 */
Bool THREAD_Detach(ThrID id)
{
    ASSERT(THREAD_IsInited());

    /* mark it as detached or deallocate it */
    if (id->flags & THR_FINISHED) {
        NT_DeleteThrID(id);
    } else {
        MUTEX_Lock(&thr_mutex);
        if (id->flags & THR_FINISHED) {
            NT_DeleteThrID(id);
        } else {
            /* else it must be in the list */
            ASSERT(QUEUE_Find(&thr_list, &id->entry));
            ASSERT(!(id->flags & THR_DETACHED));
            id->flags |= THR_DETACHED;
        }
        MUTEX_Unlock(&thr_mutex);
    }
    return True;
}

/**
 * Waits until the specified thread terminates
 */
Bool THREAD_Join(ThrID id)
{
    DWORD err;
    ASSERT(THREAD_IsInited());
    if (THREAD_IsSelf(id)) {
        ASSMSG("You've got a DEADLOCK, idiot!");
        return False;
    }

    ASSERT(!(id->flags & THR_DETACHED));
    ASSERT(KeGetCurrentIrql() == PASSIVE_LEVEL);
    err = KeWaitForSingleObject(id->throbj,Executive,KernelMode,FALSE,NULL);
    ASSERT(NT_SUCCESS(err));
    ASSERT(id->flags & THR_FINISHED);
    NT_DeleteThrID(id);
    return True;
}

/**
 * Terminates the current thread
 */
void THREAD_Exit()
{
    ThrID self = THREAD_Self();
    ASSERT(self);
    ASSERT(KeGetCurrentIrql() == PASSIVE_LEVEL);
    if (self) {
        MUTEX_Lock(&thr_mutex);
        QUEUE_RemoveEntry(&self->entry);
        if (self->flags & THR_DETACHED) {
            NT_DeleteThrID(self);
        } else {
            self->flags |= THR_FINISHED;
        }
        MUTEX_Unlock(&thr_mutex);
    }
    PsTerminateSystemThread(0);
}

/**
 * Relinguish the remainder of the time slice
 */
void THREAD_Yield()
{
    LARGE_INTEGER t = RtlConvertLongToLargeInteger(0);
    ASSERT(KeGetCurrentIrql() == PASSIVE_LEVEL);
    KeDelayExecutionThread(KernelMode, FALSE, &t);
}

/**
 * Sleep with millisecond precision
 */
void THREAD_Sleep(long ms)
{
    LARGE_INTEGER t = RtlLargeIntegerNegate( /* relative time */
                      RtlExtendedIntegerMultiply(
                      RtlConvertLongToLargeInteger(ms),10000));
    ASSERT(KeGetCurrentIrql() == PASSIVE_LEVEL);
    KeDelayExecutionThread(KernelMode, FALSE, &t);
}

/*
 * HISTORY:
 *
 * $Log: k_thread.c,v $
 * Revision 1.2  2009/10/21 13:22:09  slava
 * o added THREAD_IsSelf function
 *
 * Revision 1.1  2005/02/19 00:37:41  slava
 * o separated platform specific code from platform independent code
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
