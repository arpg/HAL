/*
 * $Id: s_wkq.c,v 1.20 2009/10/26 11:46:43 slava Exp $
 *
 * Copyright (C) 2001-2009 by Slava Monich
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

#include "s_wkq.h"
#include "s_queue.h"
#include "s_event.h"
#include "s_mutex.h"
#include "s_thread.h"
#include "s_util.h"
#include "s_libp.h"
#include "s_mem.h"

typedef struct _Waiter Waiter;
struct _Waiter {
    Event event;                /* event the waiter is waiting on */
    Waiter * next;              /* next waiter */
};

struct _WorkItem {
    QEntry itemsQ;              /* queue entry, WorkQueue->items list */
    QEntry submitQ;             /* queue entry, WorkQueue->submit queue */
    WorkProc proc;              /* function to call */
    void * param;               /* parameter to pass in */
    WorkProc2 proc2;            /* callback function taking 2 parameters */
    void * param2;              /* second parameter */
    Waiter * waiters;           /* the waiters */
    int flags;                  /* flags, see below */

#define WKI_CALL        0x0001  /* callback is in progress */
#define WKI_DONE        0x0002  /* callback was invoked */
#define WKI_CANCELED    0x0004  /* work item has been canceled */
#define WKI_DETACHED    0x0008  /* work item is owned by the work queue */
};

struct _WorkQueue {
    Queue items;                /* all work items associated with this queue */
    Queue submit;               /* submitted work items */
    Mutex mutex;                /* synchronization mutex */
    Event event;                /* notification event */
    Event stopEvent;            /* final notification event */
    IdleProc idleProc;          /* idle callback */
    void * idleParam;           /* parameter of the idle callback */
    long idleTimeout;           /* idle timeout */
    Time lastActivity;          /* when we last time did something */
    int flags;                  /* flags, see below */

#define WKQ_ACTIVE      0x0001  /* worker thread is active */
#define WKQ_DEAD        0x0002  /* work item thread has died */
#define WKQ_KILLME      0x0004  /* queue was destroyed on its own thread */

    int nthreads;               /* number of worker threads */
    ThrID threads[1];           /* ids of the worker threads */
};

#define DEFAULT_MAX_WAIT_POOL 16
#define DEFAULT_MAX_ITEM_POOL 16

typedef struct _WorkQueueModule {
    int initcount;              /* how many times we were initialized */
    Mutex mutex;                /* synchronization object */
    Waiter * waitpool;          /* pool of waiters */
    int maxwait;                /* max number of waiters in the pool */
    int nwait;                  /* current number of waiters in the pool */
    Queue itempool;             /* pool of preallocated work items */
    int maxitems;               /* maximum size of the work item pool */
} WorkQueueModule;

/*==========================================================================*
 *              G L O B A L    C O N T E X T
 *==========================================================================*/

STATIC WorkQueueModule WKQ = {0};

void WKQ_InitModule()
{
    if ((WKQ.initcount++) == 0) {
        THREAD_InitModule();
        QUEUE_Init(&WKQ.itempool);
        if (MUTEX_Init(&WKQ.mutex)) {
            WKQ.maxwait = DEFAULT_MAX_WAIT_POOL;
            WKQ.maxitems = DEFAULT_MAX_ITEM_POOL;
            return;
        }
        SLIB_Abort(TEXT("WKQ"));
    }
}

void WKQ_Shutdown()
{
    if ((--WKQ.initcount) == 0) {
        while (WKQ.waitpool) {
            Waiter * next = WKQ.waitpool->next;
            EVENT_Destroy(&WKQ.waitpool->event);
            MEM_Free(WKQ.waitpool);
            WKQ.waitpool = next;
            WKQ.nwait--;
        }
        ASSERT(WKQ.nwait == 0);
        while (!QUEUE_IsEmpty(&WKQ.itempool)) {
            QEntry * e = QUEUE_RemoveHead(&WKQ.itempool);
            WorkItem * w = QCAST(e,WorkItem,itemsQ);
            MEM_Free(w);
        }
        MUTEX_Destroy(&WKQ.mutex);
        THREAD_Shutdown();
    }
}

/**
 * Returns a wait context from the pool, or allocates a new one
 */
STATIC Waiter * WKQ_GetWaiter(WorkQueueModule * module)
{
    Waiter * waiter = NULL;
    ASSERT(module->initcount > 0);
    
    if (module->waitpool) {
        MUTEX_Lock(&module->mutex);
        if (module->waitpool) {
            waiter = module->waitpool;
            module->waitpool = waiter->next;
            waiter->next = NULL;
            module->nwait--;
            ASSERT(module->nwait >= 0);
            ASSERT(module->nwait || !module->waitpool);
        }
        MUTEX_Unlock(&module->mutex);
    }
    
    if (!waiter) {
        waiter = MEM_New(Waiter);
        if (waiter) {
            if (!EVENT_Init(&waiter->event)) {
                MEM_Free(waiter);
                waiter = NULL;
            }
        }
    }

    if (waiter) {
        EVENT_Reset(&waiter->event);
    }

    return waiter;
}

/**
 * Internal routine. Puts waiter to the pool, assumes that caller provides
 * synchronization and has checked everything (that waiter isn't NULL, 
 * that pool size did not exceed the maximum, etc.)
 */
STATIC void WKQ_WaiterToPool(WorkQueueModule * module, Waiter * waiter)
{
    waiter->next = module->waitpool;
    module->waitpool = waiter;
    module->nwait++;
}

/**
 * Puts event back to the pool or deallocates it.
 */
STATIC void WKQ_ReleaseWaiter(WorkQueueModule * module, Waiter * waiter)
{
    ASSERT(module->initcount > 0);
    if (module->nwait < module->maxwait) {
        MUTEX_Lock(&module->mutex);
        if (module->nwait < module->maxwait) {
            WKQ_WaiterToPool(module, waiter);
            waiter = NULL;
        }
        MUTEX_Unlock(&module->mutex);
    }
    if (waiter) {
        EVENT_Destroy(&waiter->event);
        MEM_Free(waiter);
    }
}

/**
 * Returns work item from the pool, allocates a new one if needed.
 */
STATIC WorkItem *
WKQ_GetWorkItem(WorkQueueModule * mod, WorkQueue * q, 
    WorkProc cb, WorkProc2 cb2, void * p1, void * p2)
{
    WorkItem * w = NULL;

    ASSERT(mod->initcount > 0);
    /* can't use QUEUE_IsEmpty without synchronization */
    if (!mod->itempool.size) {
        MUTEX_Lock(&mod->mutex);
        if (!QUEUE_IsEmpty(&mod->itempool)) {
            w = QCAST(QUEUE_RemoveHead(&mod->itempool),WorkItem,itemsQ);
            w->flags = 0;
        }
        MUTEX_Unlock(&mod->mutex);
    }

    if (!w) {
        w = MEM_New(WorkItem);
        if (w) {
            memset(w, 0, sizeof(*w));
        }
    }

    if (w) {
        if (MUTEX_Lock(&q->mutex)) {
            w->proc = cb;
            w->proc2 = cb2;
            w->param = p1;
            w->param2 = p2;
            QUEUE_InsertTail(&q->items,&w->itemsQ);
            MUTEX_Unlock(&q->mutex);
            return w;
        }
        MEM_Free(w);
    }

    return NULL;
}

/**
 * Puts work item to the global pool or deallocates it. Also deallocates 
 * the events associated with the work item. NOTE: this code is designed 
 * to be efficient, not compact
 */
STATIC void WKQ_ReleaseWorkItem(WorkQueueModule * module, WorkItem * w)
{
    Bool locked = False;

    ASSERT(module->initcount > 0);
    ASSERT(!w->submitQ.queue);
    ASSERT(!w->itemsQ.queue);

    /* deallocate waiters */
    while (w->waiters) {
        Waiter * waiter = w->waiters;
        Waiter * next = waiter->next;

        if (module->nwait < module->maxwait) {
            if (locked) {
                WKQ_WaiterToPool(module, waiter);
                waiter = NULL;
            } else {
                locked = MUTEX_Lock(&module->mutex);
                if (module->nwait < module->maxwait) {                    
                    WKQ_WaiterToPool(module, waiter);
                    waiter = NULL;
                }
            }
        }

        if (waiter) {
            EVENT_Destroy(&waiter->event);
            MEM_Free(waiter);
        }

        w->waiters = next;
    }

    if (QUEUE_Size(&module->itempool) < module->maxitems) {
        if (locked) {
            w->flags = WKI_DETACHED;
            QUEUE_InsertTail(&module->itempool, &w->itemsQ);
        } else {
            locked = MUTEX_Lock(&module->mutex);
            if (QUEUE_Size(&module->itempool) < module->maxitems) {
                w->flags = WKI_DETACHED;
                QUEUE_InsertTail(&module->itempool, &w->itemsQ);
            } else {
                MEM_Free(w);
            }
        }
    } else {
        MEM_Free(w);
    }

    if (locked) {
        MUTEX_Unlock(&module->mutex);
    }
}

/*==========================================================================*
 *              W O R K    I T E M
 *==========================================================================*/

/**
 * Callback for the work item that has two parameters
 */
STATIC void WKI_WorkProc2(WorkItem * w, void* param)
{
    ASSERT(w->param == param);
    if (w->proc2) {
        w->proc2(w, w->param, w->param2);
    }
}

/**
 * Return the work queue for the specified work item
 */
STATIC WorkQueue * WKI_GetQueue(WorkItem * w)
{
    if (w) {
        ASSERT(w->itemsQ.queue);
        if (w->itemsQ.queue) {
            return CAST_QUEUE(w->itemsQ.queue,WorkQueue,items);
        }
    }
    return NULL;
}

/**
 * Creates a work item. The returned work item is not associated with any
 * work queue.
 */
WorkItem * WKI_Create(WorkQueue * q, WorkProc cb, void * par)
{
    ASSERT(WKQ.initcount > 0);
    if (WKQ.initcount == 0) WKQ_InitModule();
    return WKQ_GetWorkItem(&WKQ, q, cb, NULL, par, NULL);
}

/**
 * Creates a work item that has 2 parameters. The returned work item is not
 * associated with any work queue.
 */
WorkItem * WKI_Create2 (WorkQueue * q, WorkProc2 cb, void * p1, void * p2)
{
    ASSERT(WKQ.initcount > 0);
    if (WKQ.initcount == 0) WKQ_InitModule();
    return WKQ_GetWorkItem(&WKQ, q, WKI_WorkProc2, cb, p1, p2);
}

/**
 * Submits a work item to the specified work queue. Re-submitting the same
 * work before it has been executed just moves it to the tail of the work
 * queue. It does NOT schedule it to run twice.
 */
Bool WKI_Submit(WorkItem * w)
{
    WorkQueue * q = WKI_GetQueue(w);
    ASSERT(q);
    ASSERT(!(w->flags & WKI_DETACHED));
    if (q) {
        if (MUTEX_Lock(&q->mutex)) {
            if (q->flags & WKQ_ACTIVE) {
                w->flags &= ~(WKI_DONE | WKI_CANCELED);
                QUEUE_RemoveEntry(&w->submitQ);
                QUEUE_InsertTail(&q->submit, &w->submitQ);
                EVENT_Set(&q->event);
                MUTEX_Unlock(&q->mutex);
                return True;
            }
            MUTEX_Unlock(&q->mutex);
            /* fall through and return False */
        }
    }
    return False;
}

/**
 * These simple functions check various flags
 */
Bool WKI_IsCanceled(WorkItem * w)
{
    ASSERT(!(w->flags & WKI_DETACHED));
    return BoolValue(w->flags & WKI_CANCELED);
}

Bool WKI_IsDone(WorkItem * w)
{
    ASSERT(!(w->flags & WKI_DETACHED));
    return BoolValue(w->flags & WKI_DONE);
}

/**
 * Switches the work item into the "detached" state. A detached work
 * item is not waitable, the caller does not own the work item anymore.
 */
void WKI_Detach(WorkItem * w)
{
    WorkQueue * q = WKI_GetQueue(w);
    ASSERT(!(w->flags & WKI_DETACHED));
    MUTEX_Lock(&q->mutex);
    w->flags |= WKI_DETACHED;
    if (!w->submitQ.queue && !(w->flags & WKI_CALL)) {
        QUEUE_RemoveEntry(&w->itemsQ);
        WKQ_ReleaseWorkItem(&WKQ, w);
    }
    MUTEX_Unlock(&q->mutex);
}

/**
 * Attaches a waiter to the work item if the work item is waitable.
 * The caller is responsible for deallocating this waiter
 */
STATIC Waiter * WKI_AttachWaiter(WorkItem * w)
{
    Waiter * waiter = NULL;
    WorkQueue * q = WKI_GetQueue(w);
    ASSERT(!(w->flags & WKI_DETACHED));

    /* quick check without synchronization */
    if (w->submitQ.queue || (w->flags & WKI_CALL)) {

        /* We avoid calling WKQ_GetWaiter under mutex because in NT kernel
         * mode environment this results in KeInitializeEvent being called
         * at IRQL DISPATCH_LEVEL. According to the Windows DDK documentation,
         * the callers of KeInitializeEvent must be running at IRQL 
         * PASSIVE_LEVEL. I personally think it's a mistake in the
         * documentation, because KeInitializeEvent does not do anything
         * that would require the current thread to wait or to access
         * pageable memory, unless the event is allocated from paged pool
         * or the KeInitializeEvent code itself resides in a pageable code
         * segment (which does not seem to be the case). Anyway, I decided
         * to play it safe and follow the documentation.
         */
        waiter = WKQ_GetWaiter(&WKQ);
        if (waiter) {
            Bool waitable = False;
            MUTEX_Lock(&q->mutex);
            /* the same check, this time under synchronization */
            if (w->submitQ.queue || (w->flags & WKI_CALL)) {
                waiter->next = w->waiters;
                w->waiters = waiter;
                waitable = True;
            }
            MUTEX_Unlock(&q->mutex);
            if (!waitable) {

                /* Something must have changed while we were allocating the 
                 * waiter. Return it to the pool. In real life, this almost 
                 * never happens.
                 */
                WKQ_ReleaseWaiter(&WKQ, waiter);
                waiter = NULL;
            }
        }
    }
    return waiter;
}

/**
 * Waits for the work item to complete
 */
void WKI_Wait(WorkItem * w)
{
    Waiter * waiter = WKI_AttachWaiter(w);
    if (waiter) {
        EVENT_Wait(&waiter->event);
        WKQ_ReleaseWaiter(&WKQ, waiter);
    }
}

/**
 * Waits for the work item to complete
 */
WaitState WKI_TimeWait(WorkItem * w, long ms)
{
    WaitState result = WAIT_STATE_OK;
    Waiter * waiter = WKI_AttachWaiter(w);
    if (waiter) {
        result = EVENT_TimeWait(&waiter->event,ms);
        WKQ_ReleaseWaiter(&WKQ, waiter);
    }
    return result;
}

/**
 * Signals the events associated with the work item. The caller must 
 * hold the queue mutex. Note that the event associated with the work
 * item is deallocated by the thread that was waiting for this work 
 * item to complete.
 */
STATIC void WKI_Signal(WorkItem * w)
{
    if (w->waiters) {
        Waiter * waiter;
        for (waiter = w->waiters; waiter; waiter = waiter->next) {
            EVENT_Set(&waiter->event);
        }
        /* the waiters will release their context */
        w->waiters = NULL;
    }
}

/**
 * Cancels the work item. Returns True if work item has been removed from 
 * the queue before being called, False in any other case. Unblocks the
 * waiters.
 */
Bool WKI_Cancel(WorkItem * w)
{
    Bool canceled = False;
    WorkQueue * q = WKI_GetQueue(w);
    ASSERT(!(w->flags & WKI_DETACHED));
    if (MUTEX_Lock(&q->mutex)) {
        if (QUEUE_RemoveEntry(&w->submitQ)) {
            canceled = True;
            w->flags |= WKI_CANCELED;
            WKI_Signal(w);
        }
        MUTEX_Unlock(&q->mutex);
    }
    return canceled;
}

/*==========================================================================*
 *              W O R K    Q U E U E
 *==========================================================================*/

/**
 * Deallocates the work queue
 */
STATIC void WKQ_Free(WorkQueue * q)
{
    ASSERT(QUEUE_IsEmpty(&q->submit));
    ASSERT(QUEUE_IsEmpty(&q->items));
    ASSERT(q->flags & WKQ_DEAD);

    EVENT_Destroy(&q->stopEvent);
    EVENT_Destroy(&q->event);
    MUTEX_Destroy(&q->mutex);

    MEM_Free(q);
}

/**
 * The worker thread
 */
STATIC void WKQ_Thread(void * par) 
{
    WorkQueue * q = (WorkQueue *)par;
    TRACE("WKQ: starting\n");
    
    /* start the loop */
    MUTEX_Lock(&q->mutex);
    q->lastActivity = TIME_Now();
    while ((q->flags & WKQ_ACTIVE) || !QUEUE_IsEmpty(&q->submit)) {
        QEntry * e;
        while ((e = QUEUE_RemoveHead(&q->submit)) != NULL) {
            WorkItem * w = QCAST(e,WorkItem,submitQ);
            ASSERT(!(w->flags & (WKI_DONE|WKI_CANCELED)));

            /*
             * NULL callback may be used by dummy work items whose purpose
             * is to wait until all pending work items have been processed
             */
            if (w->proc) {

                /* update flags */
                w->flags |= WKI_CALL;

                /* invoke the handler */
                MUTEX_Unlock(&q->mutex);
                w->proc(w, w->param);
                MUTEX_Lock(&q->mutex);

                q->lastActivity = TIME_Now();
                if (w->flags & WKI_DETACHED) {
                
                    /* put the work item to the pool or deallocate it */
                    ASSERT(!w->waiters);
                    QUEUE_RemoveEntry(&w->itemsQ);
                    WKQ_ReleaseWorkItem(&WKQ, w);

                } else {

                    /* 
                     * update flags. Note that we released the mutex when 
                     * were invoking the callback. Therefore, this work 
                     * item could be re-submitted to the queue. Or it could
                     * be re-submitted and then canceled. In such cases we
                     * don't need to set the WKI_DONE flag.
                     */
                    w->flags &= ~WKI_CALL;
                    if (!(w->flags & WKI_CANCELED) && !w->submitQ.queue) {
                        w->flags |= WKI_DONE;
                    }

                    /* signal the events associated with the work item */
                    WKI_Signal(w);
                }
            } else {
                
                /* it's a dummy work item. Just release the waiters */
                WKI_Signal(w);
            }
        }

        /* wait for a signal */
        if (q->flags & WKQ_ACTIVE) {
            EVENT_Reset(&q->event);
            if (q->idleProc) {

                /* we have an idle timeout */
                IdleProc idle = q->idleProc;
                void * param = q->idleParam;
                Time now = TIME_Now();
                Time deadline = q->lastActivity + q->idleTimeout;
                if (deadline > now) {
                    MUTEX_Unlock(&q->mutex);
                    switch (EVENT_TimeWait(&q->event,(long)(deadline-now))) {

                    case WAIT_STATE_OK:
                        /* don't invoke idle callback */
                        MUTEX_Lock(&q->mutex);
                        break;

                    case WAIT_STATE_TIMEOUT:
                        /* invoke idle callback */
                        MUTEX_Lock(&q->mutex);
                        now = TIME_Now();
                        deadline = q->lastActivity + q->idleTimeout;
                        if (deadline <= now) {
                            MUTEX_Unlock(&q->mutex);
                            q->lastActivity = now;
                            idle(q, param);
                            MUTEX_Lock(&q->mutex);
                        }
                        break;

                    default:
                    case WAIT_STATE_ERROR:
                        /* terminate the thread on error */
                        MUTEX_Lock(&q->mutex);
                        q->flags &= ~WKQ_ACTIVE;
                        break;
                    }
                } else {
                    q->lastActivity = now;
                    MUTEX_Unlock(&q->mutex);
                    idle(q, param);
                    MUTEX_Lock(&q->mutex);
                }

            } else {

                /* wait forever */
                MUTEX_Unlock(&q->mutex);
                EVENT_Wait(&q->event);
                MUTEX_Lock(&q->mutex);
            }
        }
    }

    /* cleanup */
    MUTEX_Unlock(&q->mutex);
    TRACE("WKQ: done\n");
    if (q->flags & WKQ_KILLME) {
        TRACE1("WKQ: killing WorkQueue %p\n",q);
        WKQ_Free(q);
    }
}

/**
 * Creates and starts a single worker thread
 */
WorkQueue * WKQ_Create()
{
    return WKQ_CreatePool(1);
}

/**
 * Creates and starts n worker thread
 */
WorkQueue * WKQ_CreatePool(int n)
{
    size_t size = sizeof(WorkQueue) + sizeof(ThrID)*(MAX(n,1)-1);
    WorkQueue * q = MEM_Alloc(size);
    if (q) {
        ASSERT(WKQ.initcount > 0);
        if (WKQ.initcount == 0) WKQ_InitModule();
        memset(q, 0, size);
        q->nthreads = n;
        if (MUTEX_Init(&q->mutex)) {
            if (EVENT_Init(&q->event)) {
                if (EVENT_Init(&q->stopEvent)) {
                    if (EVENT_Reset(&q->stopEvent)) {
                        int i;
                        q->flags = WKQ_ACTIVE;
                        QUEUE_Init(&q->items);
                        QUEUE_Init(&q->submit);
                        for (i=0; i<n; i++) {
                            if (!THREAD_Create(q->threads+i, WKQ_Thread, q)) {
                                WKQ_Delete(q);
                                return NULL;
                            }
                        }
                        return q;
                    }
                    EVENT_Destroy(&q->stopEvent);
                }
                EVENT_Destroy(&q->event);
            }
            MUTEX_Destroy(&q->mutex);
        }
        MEM_Free(q);
    }
    return NULL;
}

/**
 * Stop the worker thread
 */
void WKQ_Stop(WorkQueue * q, Bool canWait)
{
    int nthreads = 0;

    if (MUTEX_Lock(&q->mutex)) {
        if (q->flags & WKQ_ACTIVE) {
            q->flags &= ~WKQ_ACTIVE;
            EVENT_Set(&q->event);
        }
        if (canWait && q->nthreads) {
            ASSERT(!(q->flags & WKQ_DEAD));
            nthreads = q->nthreads;
            q->nthreads = 0;
        }
        MUTEX_Unlock(&q->mutex);
    }

    if (canWait) {
        if (nthreads > 0) {
            int i, flags = WKQ_DEAD;
            for (i=0; i<nthreads; i++) {
                if (THREAD_IsSelf(q->threads[i])) {

                    /* We can handle this, but it's a bit awkward and
                     * usually unnecessary. Let's issue a warning in
                     * the debug build, just in case */
                    TRACE("WARNING! WKQ_Stop called on its own thread\n");

                    /* Settings WKQ_KILLME flag will cause WKQ_Thread to
                     * kill the associated WorkQueue. Obviously, we have
                     * to set this flag after all other threads have
                     * terminated ot they all will try to do the same */
                    flags |= WKQ_KILLME;

                    /* it's a bit weird that a thread is detaching itself
                     * but it seems to work */
                    THREAD_Detach(q->threads[i]);
                } else {
                    THREAD_Join(q->threads[i]);
                }
            }
            ASSERT(!(q->flags & WKQ_DEAD));
            MUTEX_Lock(&q->mutex);
            q->flags |= flags;
            EVENT_Set(&q->stopEvent);
            MUTEX_Unlock(&q->mutex);
        } else {
            WKQ_Wait(q);
        }
    }
}

/**
 * Waits until all work items associated with this work queue are done ane 
 * deallocates the work queue.
 */
void WKQ_Delete(WorkQueue * q)
{
    if (q) {
        WKQ_Stop(q, True);

        if (q->flags & WKQ_KILLME) {
            ASSERT(QUEUE_Size(&q->items) == 1);
        } else {
            ASSERT(QUEUE_IsEmpty(&q->items));
            WKQ_Free(q);
        }
    }
}

/**
 * Submits internal workitem that will invoke the specified callback function
 * with the specified parameter
 */
Bool WKQ_InvokeLater(WorkQueue * q, WorkProc cb, void * par)
{
    Bool ok = False;
    WorkItem * w = WKI_Create(q, cb, par);
    if (w) {
        ok = WKI_Submit(w);
        WKI_Detach(w);
    }
    return ok;
}

/**
 * Submits internal workitem that will invoke the specified callback function
 * with the specified parameter, and waits for completion
 */
Bool WKQ_InvokeAndWait(WorkQueue * q, WorkProc cb, void * par)
{
    Bool ok = False;
    WorkItem * w = WKI_Create(q, cb, par);
    if (w) {
        ok = WKI_Submit(w);
        if (ok) {
            WKI_Wait(w);
        }
        WKI_Detach(w);
    }
    return ok;
}

/**
 * Submits internal workitem that will invoke the specified callback function
 * with the specified parameters
 */
Bool WKQ_InvokeLater2 (WorkQueue * q, WorkProc2 cb, void * p1, void * p2)
{
    Bool ok = False;
    WorkItem * w = WKI_Create2(q, cb, p1, p2);
    if (w) {
        ok = WKI_Submit(w);
        WKI_Detach(w);
    }
    return ok;
}

/**
 * Submits internal workitem that will invoke the specified callback function
 * with the specified parameters, and waits for completion
 */
Bool WKQ_InvokeAndWait2 (WorkQueue * q, WorkProc2 cb, void * p1, void * p2)
{
    Bool ok = False;
    WorkItem * w = WKI_Create2(q, cb, p1, p2);
    if (w) {
        ok = WKI_Submit(w);
        if (ok) {
            WKI_Wait(w);
        }
        WKI_Detach(w);
    }
    return ok;
}

/**
 * Sets the idle timeout function for the work queue. If the cb parameter 
 * is NULL, idle timeouts are disabled and the other parameters (ms, param)
 * are ignored. The timeout must be positive.
 *
 * NOTE: currently, the old callback may still be invoked (btu no more than
 * once) after this function has returned.
 */
void WKQ_SetIdle(WorkQueue * q, long ms, IdleProc cb, void * param)
{
    if (MUTEX_Lock(&q->mutex)) {
        if (cb) {
            ASSERT(ms > 0);
            q->idleTimeout = MAX(ms,1);
            q->idleProc = cb;
            q->idleParam = param;
        } else {
            q->idleTimeout = 0;
            q->idleProc = NULL;
            q->idleParam = NULL;
        }
        /* notify the worker thread */
        EVENT_Set(&q->event); 
        MUTEX_Unlock(&q->mutex);
    }
}

/**
 * Cancels all pending work items in the work queue.
 */
void WKQ_Cancel(WorkQueue * q)
{
    if (MUTEX_Lock(&q->mutex)) {
        QEntry * e;
        while ((e = QUEUE_RemoveHead(&q->submit)) != NULL) {
            WorkItem * w = QCAST(e,WorkItem,submitQ);
            w->flags |= WKI_CANCELED;
            if (w->flags & WKI_DETACHED) {
                ASSERT(!w->waiters);
                QUEUE_RemoveEntry(&w->itemsQ);
                WKQ_ReleaseWorkItem(&WKQ, w);
            } else {
                WKI_Signal(w);
            }
        }
        MUTEX_Unlock(&q->mutex);
    }
}

/**
 * Waits for this work queue to stop
 */
void WKQ_Wait(WorkQueue * q)
{
    EVENT_Wait(&q->stopEvent);
    ASSERT(q->flags & WKQ_DEAD);
}

/**
 * Waits for this work queue to stop
 */
WaitState WKQ_TimeWait(WorkQueue * q, long ms)
{
    return EVENT_TimeWait(&q->stopEvent, ms);
}

/*
 * HISTORY:
 *
 * $Log: s_wkq.c,v $
 * Revision 1.20  2009/10/26 11:46:43  slava
 * o allow destruction of WorkQueue on its own work thread
 *
 * Revision 1.19  2009/10/02 20:25:56  slava
 * o added WKI_Create2, WKQ_InvokeLater2 and WKQ_InvokeAndWait2 functions
 *   that create work items with two context parameters which it sometimes
 *   very convenient. Also, fixed a bug in WKQ_GetWorkItem (QUEUE_IsEmpty
 *   can't be called without synchronization)
 *
 * Revision 1.18  2009/06/30 11:16:40  slava
 * o extended WorkQueue functionality so that it can use multuiple threads
 *   to run the work items.
 *
 * Revision 1.17  2005/02/19 03:36:13  slava
 * o fixed compilation warnings produced by C++ compiler
 *
 * Revision 1.16  2004/11/28 18:19:51  slava
 * o allow NULL callback function - such dummy work items may be used in order
 *   to wait until the work queue it empty.
 *
 * Revision 1.15  2004/11/27 17:41:24  slava
 * o added WKQ_SetIdle function (sets idle callback)
 *
 * Revision 1.14  2003/06/21 18:26:06  slava
 * o fixed compilation warning under HP-UX
 *
 * Revision 1.13  2003/05/21 00:11:04  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.12  2003/01/20 19:02:47  slava
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
 * Revision 1.11  2002/08/29 04:50:09  slava
 * o added WKQ_Cancel (cancels all pending work items)
 *
 * Revision 1.10  2002/08/29 03:15:36  slava
 * o cleanup, renamed WKQ_Destroy into WKQ_Delete
 *
 * Revision 1.9  2002/08/27 11:52:03  slava
 * o some reformatting
 *
 * Revision 1.8  2002/05/29 08:29:49  slava
 * o removed EVENT_InitModule and MUTEX_InitModule functions
 *
 * Revision 1.7  2002/05/15 01:34:50  slava
 * o fixed a bug in WKQ_Stop - it could block forever for no good reason
 *
 * Revision 1.6  2001/12/18 05:49:50  slava
 * o WorkProc takes WorkItem * as another parameter. This allows a WorkProc
 *   function to determine that its work item has been canceled
 *
 * Revision 1.5  2001/11/28 02:31:05  slava
 * o redesigned WKI_Wait routines so that they can be safely invoked by
 *   multiple threads
 *
 * Revision 1.4  2001/11/27 14:32:34  slava
 * o avoid grabbing global lock unless it's absolutely necessary
 *
 * Revision 1.3  2001/11/27 08:22:30  slava
 * o do not detach the work queue thread
 *
 * Revision 1.2  2001/11/27 06:49:57  slava
 * o fixed compilation warning
 *
 * Revision 1.1  2001/11/27 06:06:58  slava
 * o added "work queue" module
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
