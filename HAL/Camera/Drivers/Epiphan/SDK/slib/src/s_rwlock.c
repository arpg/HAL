/*
 * $Id: s_rwlock.c,v 1.35 2011/03/03 15:28:46 slava Exp $
 *
 * Copyright (C) 2000-2011 by Slava Monich
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

#include "s_rwlock.h"
#include "s_lockp.h"
#include "s_util.h"
#include "s_mem.h"

/*
 * Flags
 */
#define RWLOCK_FLAG_EXCLUSIVE_LOCK         0x0001 /* writer owns the lock */
#define RWLOCK_FLAG_EXCLUSIVE_EVENT_IN_USE 0x0002 /* exclusiveEvent in use */

/*
 * Waiter context. In the waiter is shared, then its event field points
 * to Lock->shareEvent. The event field of the first exclusive waiter
 * points to Lock->exclusiveEvent. For subsequent exclusive waiters,
 * it points to a dynamically allocated event. The index is zero for
 * the first waiter. If waiter was not the first, the index if the 
 * highest active index + 1. The highest active index is determined
 * by checking the last entries in both exclusive and shared lists.
 */
typedef struct _RWLockWaiter {
    QEntry entry;               /* entry in the linked list */
    Event * event;              /* event this waiter is waiting on */
    int index;                  /* to determine the order of waiters */
} RWLockWaiter;

/*
 * Here is the problem. The resource is being used in readonly mode.
 * Then comes an exclusive waiter. It gets put into the list of waiters
 * and starts waiting. Then comes another readonly waiter. What do we do?
 * We can do one of the two things:
 * 
 * o immediately grant access to the reader because writer has to wait anyway
 * o put reader into the queue after the writer because writer got here first
 *
 * The first case should normally be more efficient unless reader are coming
 * in too fast, in which case the writer may never get a chance to run. The
 * second variant guarantees that writer will run sooner or later but forces
 * reader threads wait longer than necessary. What we do is a combination
 * of these two solutions. We allow the reader to get ahead of the writer
 * as long as this happens no more than RWLOCK_MAX_BYPASS_COUNT times. After 
 * that, everyone, including the readers has to wait for its turn. The
 * counter is reset every time a writer gets the lock.
 */
#define RWLOCK_MAX_BYPASS_COUNT 10

/*
 * Number of entries allocated statically as a part of Lock structure
 */
#define STATIC_ENTRIES ((int)(FIELDSIZE(RWLock,staticEntries)/sizeof(RWEntry)))

/* 
 * The maximum number of waiters to cache.
 */
#define MAX_WAITER_CACHE_SIZE (STATIC_ENTRIES + 1)

/*
 * NOTE: Even if we allocate additional entries we keep using 
 * the static entries.
 *
 * EXCERSISE. Below is the original version of GET_ENTRY macro:
 *
 *   #define GET_ENTRY(_lock, _i) \
 *        (((_i) < STATIC_ENTRIES) ? \
 *        ((_lock)->staticEntries + (_i)) : \
 *        ((_lock)->moreEntries + ((_i) - STATIC_ENTRIES)))
 *
 * For some reason (I think it's a bug in Microsoft's compiler)
 * this macro results in a code that causes access violation
 * when compiled by Visual C++ 6.0 with optimization on (i.e. 
 * release build). Moving this code into a function solved 
 * the problem. Am I missing something? Do you see any problem
 * with this macro?
 */
       
#define GET_ENTRY(_lock, _i) RWLOCK_GetEntryAt(_lock, _i)

/* 
 * Synchronization object descriptor
 */
STATIC const LockType LockType_RWLock;

/**
 * Creates a read/write lock. This function is defined here (rather than
 * in s_lock.c) to avoid unnecessary linkage with the lock types that you
 * are not using.
 */
Lock * LOCK_CreateRWLock()
{
    RWLock * l = RWLOCK_Create();
    return l ? &l->lock : NULL;
}

/**
 * Allocates a new lock and initializes it
 */
RWLock * RWLOCK_Create() 
{
    RWLock * lock = MEM_New(RWLock);
    if (lock) {
        if (RWLOCK_Init(lock)) {
            return lock;
        }
        MEM_Free(lock);
    }
    return NULL;
}

/**
 * Deletes and deallocates the mutex previously allocated with 
 * RWLOCK_Create()
 */
void RWLOCK_Delete(RWLock * lock) 
{
    if (lock) {
        RWLOCK_Destroy(lock);
        MEM_Free(lock);
    }
}

/**
 * Initialize the lock.
 */
Bool RWLOCK_Init(RWLock * lock) 
{
    LOCK_InitCheck();
    memset(lock, 0, sizeof(*lock));
    if (MUTEX_Init(&lock->mutex)) {
        if (EVENT_Init(&lock->shareEvent)) {
            if (EVENT_Init(&lock->exclusiveEvent)) {
                QUEUE_Init(&lock->shareWaiters);
                QUEUE_Init(&lock->exclusiveWaiters);
                QUEUE_Init(&lock->waiterCache);
                lock->numEntries = COUNT(lock->staticEntries);
                lock->lock.type = &LockType_RWLock;
                return True;
            }
            EVENT_Destroy(&lock->shareEvent);
        }
        MUTEX_Destroy(&lock->mutex);
    }
    return False;
}

/**
 * Deallocates the system resources used by the lock.
 */
void RWLOCK_Destroy(RWLock * lock) 
{
    QEntry * e;
    ASSERT(!lock->locks);
    ASSERT(!lock->entriesInUse);
    ASSERT(!lock->entriesActive);
    ASSERT(!lock->shareWaiters.size);
    ASSERT(!lock->exclusiveWaiters.size);
    MUTEX_Destroy(&lock->mutex);
    EVENT_Destroy(&lock->shareEvent);
    EVENT_Destroy(&lock->exclusiveEvent);

    /* free dynamically allocated entries */
    if (lock->moreEntries) {
        MEM_Free(lock->moreEntries);
        lock->moreEntries = NULL;
    }

    /* free waiter cache */
    while ((e = QUEUE_RemoveHead(&lock->waiterCache)) != NULL) {
        RWLockWaiter * w = QCAST(e,RWLockWaiter,entry);
        MEM_Free(w);
    }

    /* free dynamically allocated events */
    while (lock->eventsInCache > 0) {
        lock->eventsInCache--;
        ASSERT(lock->eventCache[lock->eventsInCache]);
        EVENT_Delete(lock->eventCache[lock->eventsInCache]);
        lock->eventCache[lock->eventsInCache] = NULL;
    }

    lock->numEntries = 0;
    lock->locks = -1;  /* to cause ASSERT if destroyed twice */
}

/**
 * Gets event from cache or allocates a new one. Must be called 
 * under synchronization.
 */
STATIC Event * RWLOCK_GetExclusiveEvent(RWLock * lock)
{
    if (lock->eventsInCache > 0) {
        Event * e = lock->eventCache[lock->eventsInCache-1];
        lock->eventCache[lock->eventsInCache-1] = NULL;
        lock->eventsInCache--;
        ASSERT(e);
        return e;
    } else {
        return EVENT_Create();
    }
}

/**
 * Returns event to the cache or deallocates it if the cache is full.
 * Must be called under synchronization
 */
STATIC void RWLOCK_ReleaseExclusiveEvent(RWLock * lock, Event * e)
{
    ASSERT(e);
    ASSERT(e != &lock->exclusiveEvent);
    if (lock->eventsInCache < COUNT(lock->eventCache)) {
        lock->eventCache[lock->eventsInCache++] = e;
    } else {
        EVENT_Delete(e);
    }
}

/**
 * Gets the next waiter index. The index is zero for the first waiter.
 * If waiter was not the first, the index if the highest active index + 1.
 * The highest active index is determined by checking the last entries
 * in both exclusive and shared lists. Must be called under synchronization
 */
STATIC int RWLOCK_GetNextWaiterIndex(RWLock * lock)
{
    int i = 0;
    RWLockWaiter * lastWaiter;
    QEntry * e = QUEUE_Last(&lock->shareWaiters);
    if (e) {
        lastWaiter = QCAST(e,RWLockWaiter,entry);
        i = lastWaiter->index + 1;
    }
    e = QUEUE_Last(&lock->exclusiveWaiters);
    if (e) {
        lastWaiter = QCAST(e,RWLockWaiter,entry);
        i = MAX(i,lastWaiter->index + 1);
    }
    return i;
}

/**
 * Puts RWLockWaiter structure into the cache or deallocates it. This is
 * enough for a shared waiter but not for exclusive waiter (see function
 * below). Must be used under synchronization.
 */
STATIC void RWLOCK_ReleaseWaiter(RWLock * lock, RWLockWaiter * w)
{
    QUEUE_RemoveEntry(&w->entry);
    if (QUEUE_Size(&lock->waiterCache) < MAX_WAITER_CACHE_SIZE) {
        w->index = 0;
        w->event = NULL;
        QUEUE_InsertTail(&lock->waiterCache, &w->entry);
    } else {
        MEM_Free(w);
    }
}

/**
 * Release a RWLockWaiter structure for an exclusive waiter.
 * Must be used under synchronization
 */
STATIC void RWLOCK_ReleaseExclusiveWaiter(RWLock * lock, RWLockWaiter * w)
{
    /* release the event */
    if (w->event) {
        if (w->event == &lock->exclusiveEvent) {
            ASSERT(lock->flags & RWLOCK_FLAG_EXCLUSIVE_EVENT_IN_USE);
            lock->flags &= ~RWLOCK_FLAG_EXCLUSIVE_EVENT_IN_USE;
        } else {
            RWLOCK_ReleaseExclusiveEvent(lock, w->event);
        }
    }

    /* put waiter into the cache or deallocate it */
    RWLOCK_ReleaseWaiter(lock, w);
}

/**
 * Gets a RWLockWaiter structure from the cache or allocates a new one.
 * It's the responsibility of the caller to initialize the event 
 * pointer and insert the waiter into the right queue. It's done
 * by RWLOCK_GetExclusiveWaiter and RWLOCK_GetSharedWaiter functions.
 * Must be called under synchronization
 */
STATIC RWLockWaiter * RWLOCK_GetWaiter(RWLock * lock)
{
    RWLockWaiter * w = NULL;
    QEntry * e = QUEUE_RemoveTail(&lock->waiterCache);
    if (e) {
        w = QCAST(e,RWLockWaiter,entry);
        w->index = RWLOCK_GetNextWaiterIndex(lock);
        return w;
    } else {
        w = MEM_New(RWLockWaiter);
        if (w) {
            memset(w,0,sizeof(*w));
            w->index = RWLOCK_GetNextWaiterIndex(lock);
            return w;
        }
    }
    return NULL;
}

/**
 * Gets a RWLockWaiter structure for an exclusive waiter from the cache or 
 * allocates a new one. Adds the waiter to the tail of the queue. 
 * Must be called under synchronization
 */
STATIC RWLockWaiter * RWLOCK_GetExclusiveWaiter(RWLock * lock)
{
    RWLockWaiter * w = RWLOCK_GetWaiter(lock);
    if (w) {
        if (lock->flags & RWLOCK_FLAG_EXCLUSIVE_EVENT_IN_USE) {
            ASSERT(w->index);
            w->event = RWLOCK_GetExclusiveEvent(lock);
            if (!w->event) {
                RWLOCK_ReleaseWaiter(lock, w);
                return NULL;
            }
        } else {
            lock->flags |= RWLOCK_FLAG_EXCLUSIVE_EVENT_IN_USE;
            w->event = &lock->exclusiveEvent;
        }
        QUEUE_InsertTail(&lock->exclusiveWaiters, &w->entry);
        return w;
    }
    return NULL;
}

/**
 * Gets a RWLockWaiter structure for a shared waiter from the cache or 
 * allocates a new one. Must be called under synchronization
 */
STATIC RWLockWaiter * RWLOCK_GetShareWaiter(RWLock * lock)
{
    RWLockWaiter * w = RWLOCK_GetWaiter(lock);
    if (w) {
        w->event = &lock->shareEvent;
        QUEUE_InsertTail(&lock->shareWaiters, &w->entry);
    }
    return w;
}

/**
 * Acquires the lock mutex. This function is being used instead of simply
 * calling MUTEX_Lock in case if Lock is a const pointer to avoid casts 
 * all over the place.
 */
STATIC void RWLOCK_GrabMutex(const RWLock * lock)
{
    MUTEX_Lock((Mutex*)&lock->mutex);
}

/**
 * Releases the ownership of the lock mutex. This function is being used 
 * instead of simply calling MUTEX_Lock in case if Lock is a const pointer 
 * to avoid casts all over the place.
 */
STATIC void RWLOCK_ReleaseMutex(const RWLock * lock) {
    MUTEX_Unlock((Mutex*)&lock->mutex);
}

/**
 * Returns lock entry at specified index. If index is less that 
 * STATIC_ENTRIES, looks up the "statis entry" (i.e. a part of 
 * the Lock structure), otherwise looks up an "extended" entry.
 * Initially this was a macro but Microsoft Visual C++ couldn't 
 * compile it right, so it became a function. Must be invoked 
 * under synchronization because it touches the extended entries
 * which may be reallocated by the other thread.
 */
STATIC RWEntry * RWLOCK_GetEntryAt(RWLock * lock, int i) 
{
    if (i < STATIC_ENTRIES) {
        return (lock->staticEntries + i);
    } else {
        int pos = i - STATIC_ENTRIES;
        ASSERT(lock->moreEntries);
        return (lock->moreEntries + pos);
    }
}

/**
 * Finds existing static RWEntry for the calling thread (an entry that 
 * is a part of Lock structure). It does not have to be called under 
 * synchronization because:
 *
 * o other thread cannot modify the entry occupied by this thread; and
 * o these entries cannot be deallocated by the other thread (unlike 
 *   the "extended" entries)
 *
 * Returns NULL if entry for calling thread does not exist.
 */
STATIC const RWEntry * RWLOCK_FindStaticEntry(const RWLock * lock) 
{
    int i;
    ThrID self = THREAD_Self();
    for (i=0; i<lock->entriesInUse && i<STATIC_ENTRIES; i++) {
        const RWEntry * entry = lock->staticEntries + i;
        if (entry->id == self) {
            return entry;
        }
    }    
    return NULL;
}

/**
 * Finds existing "extended" RWEntry for the calling thread. This 
 * function must be called under synchronization. Returns NULL if 
 * entry for calling thread does not exist.
 */
STATIC const RWEntry * RWLOCK_FindExtEntry(const RWLock * lock) 
{
    int i;
    ThrID self = THREAD_Self();
    for (i=STATIC_ENTRIES; i<lock->entriesInUse; i++) {
        const RWEntry * entry = lock->moreEntries + i-STATIC_ENTRIES;
        if (entry->id == self) {
            return entry;
        }
    }    
    return NULL;
}

/**
 * Finds a RWEntry for the calling thread. This function must be called 
 * under synchronization. Returns NULL if entry for calling thread does not 
 * exist.
 */
STATIC RWEntry * RWLOCK_FindEntry(RWLock * lock) 
{
    int i;
    ThrID self = THREAD_Self();
    for (i=0; i<lock->entriesInUse; i++) {
        RWEntry * entry = GET_ENTRY(lock,i);
        if (entry->id == self) {
            return entry;
        }
    }    
    return NULL;
}

/**
 * Returns available RWEntry for the calling thread. Reallocates array
 * of thread entries if necessary. Obviously, this function must be called 
 * under synchronization. Returns NULL only if we run out of thread entries 
 * and memory allocation fails.
 */
STATIC RWEntry * RWLOCK_GetEntry(RWLock * lock) 
{
    int i;
    ThrID self = THREAD_Self();
    RWEntry * empty = NULL;  /* vacant empty entry */

    /* 
     * try to find existing entry for this thread. At the same time
     * pick up the first available empty slot.
     */
    for (i=0; i<lock->entriesInUse; i++) {
        RWEntry * entry = GET_ENTRY(lock,i);
        if (entry->id == self) {
            return entry;
        } else if (!empty && !entry->id) {
            empty = entry;
        }
    }

    /* 
     * we didn't find existing slot for the calling thread. 
     * We have to find an empty slot.
     */
    if (!empty) {
        for (; i<lock->numEntries; i++) {
            RWEntry * entry = GET_ENTRY(lock,i);
            if (!entry->id) {
                empty = entry;
                break;
            }
        }

        /*
         * looks like we are out of luck. We need to reallocate the array
         * of thread entries.
         */
        if (!empty) {
            int count = lock->numEntries + STATIC_ENTRIES;
            size_t size = count * sizeof(RWEntry);
            RWEntry * newEntries = (RWEntry*)
                MEM_Realloc(lock->moreEntries, size);
            if (!newEntries) {
                return NULL;
            }
            
            lock->moreEntries = newEntries;
            lock->numEntries = count;
            empty = lock->moreEntries + i - STATIC_ENTRIES;
            memset(empty, 0, sizeof(RWEntry) * STATIC_ENTRIES);
        }

        lock->entriesInUse = i + 1;
    }

    lock->entriesActive++;

    /*
     * initialize the empty entry.
     */
    empty->id = self;
    empty->read = empty->write = 0L;

    return empty;
}

/**
 * Locks resource for exclusive use, waits if necessary. Returns True if lock 
 * has been successfully acquired, otherwise False.
 */
Bool RWLOCK_TimeWriteLock(RWLock * lock, long ms) 
{
    Bool ok = True;
    Bool success = False;
    RWLockWaiter * waiter = NULL;
    Time deadline = 0;

    /*
     * this flag is False if we have found that current thread is NOT 
     * an owner of the resource, so that we don't scan the lock entries
     * more than once.
     */
    Bool maybeOwner = True;
    RWEntry * entry = NULL;

    /* calculate the deadline if it's a wait with timeout */
    if (ms > 0) {
        deadline = TIME_Now() + ms;
    }

    /*
     * we can acquire the resource immediately if
     * 1. resource is unowned and no one is waiting; or
     * 2. this thread is the only one that is using the resource, either 
     *    shared or exclusively 
     */
    MUTEX_Lock(&lock->mutex);
    while (ok) {

        Time now = 0;
        
        /*
         * if this thread already owns this resource exclusively, 
         * we are all set. All we need is to increment entry count.
         */
        if (lock->entriesActive == 1 && maybeOwner) {
            if (!entry) {
                entry = RWLOCK_FindEntry(lock);
            }
            if (entry) {
                success = True;
                lock->flags |= RWLOCK_FLAG_EXCLUSIVE_LOCK;
                entry->write++; /* convert shared to exclusive */
                break;
            } else {
                maybeOwner = False;
            }
        }

        /* if resource is not owned and no one is waiting, we can have it */
        if (lock->locks <= 0) {
            Bool gotIt = False;
            if (waiter) {
                gotIt = BoolValue(lock->exclusiveWaiters.head.next == 
                    &waiter->entry);
            } else {
                gotIt = BoolValue(QUEUE_Size(&lock->shareWaiters) == 0 &&
                                  QUEUE_Size(&lock->exclusiveWaiters) == 0);
            }

            /*
             * note that it's quite possible that resource is not owned
             * but the wait queue is not empty. this can happen for example
             * if this thread just released the resource and waiters didn't
             * yet have the chance to run. in such case, this thread should
             * be place into the queue to avoid starving the waiters
             */
            if (gotIt) {
                if (!entry) {
                    entry = RWLOCK_GetEntry(lock);
                }
                if (entry) {
                    success = True;
                    lock->flags |= RWLOCK_FLAG_EXCLUSIVE_LOCK;
                    entry->write++;
                }
                break;
            }
        }

        /*
         * resource cannot be acquired immediately for exclusive access. 
         * If we cannot wait (any longer), break the loop. 
         */
        if (ms == 0) {
            break;
        } else if (ms > 0) {
            /* check for timeout */
            now = TIME_Now();
            if (now >= deadline) {
                break;
            }
        }

        /* 
         * release the mutex and wait for event to be signaled, then 
         * start it all over again. 
         */
        lock->contentions++;
        if (!waiter) {
            waiter = RWLOCK_GetExclusiveWaiter(lock);
            if (!waiter) break;
        }

        EVENT_Reset(waiter->event);
        MUTEX_Unlock(&lock->mutex);

        /* wait */
        if (ms > 0) {
            long tmo = (long)(deadline - now);
            if (EVENT_TimeWait(waiter->event,tmo) == WAIT_STATE_ERROR) {
                ok = False;
            }
        } else {
            ok = BoolValue(EVENT_Wait(waiter->event) == WAIT_STATE_OK);
        }
        MUTEX_Lock(&lock->mutex);
    }
    if (success) lock->locks++;
    if (waiter) RWLOCK_ReleaseExclusiveWaiter(lock, waiter);
    lock->bypassCount = 0;
    MUTEX_Unlock(&lock->mutex);
    return success;
}

/**
 * Locks resource for non-exclusive use, waits if necessary. Returns True 
 * if lock has been successfully acquired, otherwise False.
 */
Bool RWLOCK_TimeReadLock(RWLock * lock, long ms) 
{
    Bool ok = True;
    Bool success = False;
    RWLockWaiter * waiter = NULL;
    Time deadline = 0;

    /*
     * this flag is False if we have found that current thread is NOT 
     * an owner of the resource, so that we don't scan the lock entries
     * more than once.
     */
    Bool maybeOwner = True;
    RWEntry * entry = NULL;

    /* calculate the deadline if it's a wait with timeout */
    if (ms > 0) {
        deadline = TIME_Now() + ms;
    }

    MUTEX_Lock(&lock->mutex);
    while (ok) {
        
        Time now = 0;

        /*
         * if this thread already owns this resource either exclusively 
         * or shared, we are all set. All we need is to increment entry
         * count. NOTE that we don't touch the "exclusive" flag, meaning
         * that if resource has been acquired exclusively, it remains
         * this way.   
         */
        if (maybeOwner) {
            entry = RWLOCK_FindEntry(lock);
            if (entry) {
                success = True;
                if (lock->flags & RWLOCK_FLAG_EXCLUSIVE_LOCK) {
                    ASSERT(entry->write > 0);
                    entry->write++;
                } else { 
                    ASSERT(entry->write == 0);
                    entry->read++;
                }
                break;
            } else {
                maybeOwner = False; /* don't scan entry table again */
            }
        }

        /* if resource is not owned and no one is waiting, we can have it */
        if (lock->locks <= 0 && 
            QUEUE_Size(&lock->shareWaiters) == 0 &&
            QUEUE_Size(&lock->exclusiveWaiters) == 0) {

            /*
             * note that it's quite possible that resource is not owned
             * but the wait queue is not empty. this can happen for example
             * if this thread just released the resource and waiters didn't
             * yet have the chance to run. in such case, this thread should
             * be place into the queue to avoid starving the waiters
             */
            entry = RWLOCK_GetEntry(lock);
            if (entry) {
                success = True;
                lock->flags &= ~RWLOCK_FLAG_EXCLUSIVE_LOCK;
                entry->read++;
            }
            break;
        }

        /* 
         * if resource is owned in shared mode, there's a good chance that
         * we can have it immediately. Some restrictions apply (see below)
         */
        if (!(lock->flags & RWLOCK_FLAG_EXCLUSIVE_LOCK)) {

            /*
             * normally we allow this thread to access the resource
             * in readonly mode even if there's an exclusive waiter.
             * However, if we always did that, the exclusive waiter
             * might end up waiting forever if new readonly waters
             * keep coming. To prevent this from happening, we count
             * the number of times an exclusive waiter has been bypassed
             * by a lucky late-coming reader. If this number exceeds
             * the limit, everyone has to stay in the line.
             */
            if (QUEUE_Size(&lock->exclusiveWaiters) == 0  ||
                lock->bypassCount < RWLOCK_MAX_BYPASS_COUNT) {
                entry = RWLOCK_GetEntry(lock);
                if (entry) {
                    if (QUEUE_Size(&lock->exclusiveWaiters) > 0) {
                        lock->bypassCount++;
                    }
                    ASSERT(entry->write == 0);
                    success = True;
                    entry->read++;
                }
                break; 
            }
        }

        /*
         * resource cannot be acquired immediately for exclusive access. 
         * If we cannot wait (any longer), break the loop. 
         */
        if (ms == 0) {
            break;
        } else if (ms > 0) {
            /* check for timeout */
            now = TIME_Now();
            if (now >= deadline) {
                break;
            }
        }

        /* 
         * release the mutex and wait for event to be signalled, then 
         * start it all over again. 
         */
        lock->contentions++;
        if (!waiter) {
            waiter = RWLOCK_GetShareWaiter(lock);
            if (!waiter) break;
        }

        EVENT_Reset(&lock->shareEvent);
        MUTEX_Unlock(&lock->mutex);

        /* wait */
        if (ms > 0) {
            long tmo = (long)(deadline - now);
            if (EVENT_TimeWait(waiter->event, tmo) == WAIT_STATE_ERROR) {
                ok = False;
            }
        } else {
            ok = BoolValue(EVENT_Wait(waiter->event) == WAIT_STATE_OK);
        }

        MUTEX_Lock(&lock->mutex);
    }

    if (success) lock->locks++;
    if (waiter) RWLOCK_ReleaseWaiter(lock, waiter);
    MUTEX_Unlock(&lock->mutex);
    return success;
}

/**
 * Attempts to acquire read lock immediately, without waiting. 
 * Returns True if the lock has been acquired, False otherwise
 * (which includes error conditions)
 */
Bool RWLOCK_TryReadLock(RWLock * lock) 
{
    return RWLOCK_TimeReadLock(lock,0);
}

/**
 * Attempts to acquire exclusive lock immediately, without waiting. 
 * Returns True if the lock has been acquired, False otherwise
 * (which includes error conditions)
 */
Bool RWLOCK_TryWriteLock(RWLock * lock)
{
    return RWLOCK_TimeWriteLock(lock,0);
}

/**
 * Locks resource for non-exclusive use. If the resource cannot be acquired 
 * immediately, waits until it becomes available. Returns True if lock has 
 * been successfully acquired, False if an error occured.
 */
Bool RWLOCK_ReadLock(RWLock * lock)
{
    return RWLOCK_TimeReadLock(lock,-1);
}

/**
 * Locks resource for exclusive use. If the resource cannot be acquired 
 * immediately, waits until it becomes available. Returns True if lock has 
 * been successfully acquired, False if an error occured.
 */
Bool RWLOCK_WriteLock(RWLock * lock)
{
    return RWLOCK_TimeWriteLock(lock,-1);
}

/**
 * Converts exclusive lock to non-exclusive without releasing it. The caller
 * must own the lock exclusively.
 */
Bool RWLOCK_DropWrite(RWLock * lock)
{
    Bool success = False;
    RWEntry * entry;

    MUTEX_Lock(&lock->mutex);
    entry = RWLOCK_FindEntry(lock);

    /* current thread must have the lock */
    ASSERT(entry);
    if (entry) {

        /* and it must be the write lock */
        ASSERT(entry->write > 0);
        if (entry->write > 0) {
            QEntry * e;
            RWLockWaiter * shareWaiter = NULL;
            RWLockWaiter * exclusiveWaiter = NULL;

            /* convert write lock to read lock */
            entry->read += entry->write;
            entry->write = 0;

            /* lock is no longer owned exclusively */
            ASSERT(lock->flags & RWLOCK_FLAG_EXCLUSIVE_LOCK);
            lock->flags &= ~RWLOCK_FLAG_EXCLUSIVE_LOCK;

            /*
             * wake up shared waiters only unless the exclusive
             * waiter is first in the line
             */
            e = QUEUE_First(&lock->shareWaiters);
            if (e) shareWaiter = QCAST(e,RWLockWaiter,entry);
            e = QUEUE_First(&lock->exclusiveWaiters);
            if (e) exclusiveWaiter = QCAST(e,RWLockWaiter,entry);

            if (shareWaiter && (!exclusiveWaiter ||
                shareWaiter->index < exclusiveWaiter->index)) {
                EVENT_Set(shareWaiter->event);
            }

            /* success */
            success = True;
        }
    }

    MUTEX_Unlock(&lock->mutex);
    return success;
}

/**
 * Release n recursively acquired locks.
 */
void RWLOCK_UnlockMany(RWLock * lock, int n) 
{
    if (n > 0) {
        RWEntry * entry;
        MUTEX_Lock(&lock->mutex);
        entry = RWLOCK_FindEntry(lock);

        /*
         * if we cannot find the entry, it means that current thread 
         * does not own the lock. It's a programming error. 
         */
        ASSERT(entry);
        if (entry) {
            lock->locks--;
            
            /* first release write locks */
            if (entry->write > 0) {
                if (entry->write >= n) {
                    entry->write -= n;
                    n = 0;
                } else {
                    n -= entry->write;
                    entry->write = 0;
                }
            }

            /* then read locks */
            if (n > 0) {
                entry->read -= n;
            }

            /* 
             * ASSERT that current thread does not release more locks than 
             * it has acquired
             */
            ASSERT(lock->locks >= 0);
            ASSERT(entry->read >= 0);
            ASSERT(entry->write >= 0);

            /*
             * no more work to do unless calling thread has released the
             * resource (i.e. usage count came down to zero)
             */
            if ((entry->read + entry->write) <= 0) {
                int i;
                int inUse;
                QEntry * e;
                RWLockWaiter * shareWaiter = NULL;
                RWLockWaiter * exclusiveWaiter = NULL;

                entry->id = 0;

                lock->entriesActive--;
                ASSERT(lock->entriesActive >= 0);

                /* 
                 * update lock->entriesInUse
                 * NOTE that RWLOCK_FindStaticEntry() may access it without 
                 * synchronization.
                 */
                i = lock->entriesInUse - 1;
                inUse = 0;
                while (i >= 0) {
                    RWEntry * lockEntry = GET_ENTRY(lock,i);
                    if (lockEntry->id) {
                        inUse = i + 1;
                        break;
                    }
                    i--;
                }

                lock->entriesInUse = inUse;

                /*
                 * if resource was acquired exclusively, it must be free now
                 */
                if (lock->flags & RWLOCK_FLAG_EXCLUSIVE_LOCK) {
                    ASSERT(!lock->locks);
                    lock->flags &= ~RWLOCK_FLAG_EXCLUSIVE_LOCK;
                }

                /* 
                 * release the waiters in the order they have arrived
                 */
                e = QUEUE_First(&lock->shareWaiters);
                if (e) shareWaiter = QCAST(e,RWLockWaiter,entry);
                e = QUEUE_First(&lock->exclusiveWaiters);
                if (e) exclusiveWaiter = QCAST(e,RWLockWaiter,entry);

                if (exclusiveWaiter && (!shareWaiter ||
                    exclusiveWaiter->index < shareWaiter->index)) {
                    EVENT_Set(exclusiveWaiter->event);
                } else if (shareWaiter) {
                    /* this should unblock all shared waiters */
                    EVENT_Set(shareWaiter->event);
                }

            } else if (lock->flags & RWLOCK_FLAG_EXCLUSIVE_LOCK) {

                /*
                 * if the owner of the lock has released all its WRITE locks 
                 * but still have some READ locks, switch to shared mode.
                 */
                if (!entry->write) {            
                    QEntry * e;
                    RWLockWaiter * shareWaiter = NULL;
                    RWLockWaiter * exclusiveWaiter = NULL;

                    ASSERT(entry->read > 0);
                    lock->flags &= ~RWLOCK_FLAG_EXCLUSIVE_LOCK;

                    /*
                     * wake up shared waiters only unless the exclusive
                     * waiter is first in the line
                     */
                    e = QUEUE_First(&lock->shareWaiters);
                    if (e) shareWaiter = QCAST(e,RWLockWaiter,entry);
                    e = QUEUE_First(&lock->exclusiveWaiters);
                    if (e) exclusiveWaiter = QCAST(e,RWLockWaiter,entry);

                    if (shareWaiter && (!exclusiveWaiter ||
                        shareWaiter->index < exclusiveWaiter->index)) {
                        EVENT_Set(shareWaiter->event);
                    }
                }
            }
        }
        MUTEX_Unlock(&lock->mutex);
    }
}

/**
 * Release previously acquired lock.
 */
void RWLOCK_Unlock(RWLock * lock) 
{
    RWLOCK_UnlockMany(lock, 1);
}
 
/**
 * Determines if the calling thread has READ access to the resource 
 * protected by the lock.
 */
Bool RWLOCK_CanRead(const RWLock * lock) 
{
    if (lock->locks > 0) {
        const RWEntry * entry = RWLOCK_FindStaticEntry(lock);
        if (!entry && lock->entriesInUse > STATIC_ENTRIES) {
            /* must synchronize access to the "extended" entries */
            RWLOCK_GrabMutex(lock);
            entry = RWLOCK_FindExtEntry(lock);
            RWLOCK_ReleaseMutex(lock);
        }
        return BoolValue(entry != NULL);
    }
    return False;
}

/**
 * Determines if the calling thread has WRITE access to the resource 
 * protected by the lock.
 */
Bool RWLOCK_CanWrite(const RWLock * lock) 
{
    if (lock->locks > 0 && (lock->flags & RWLOCK_FLAG_EXCLUSIVE_LOCK)) {
        const RWEntry * entry = RWLOCK_FindStaticEntry(lock);
        if (!entry && lock->entriesInUse > STATIC_ENTRIES) {
            /* must synchronize access to the "extended" entries */
            RWLOCK_GrabMutex(lock);
            entry = RWLOCK_FindExtEntry(lock);
            RWLOCK_ReleaseMutex(lock);
        }
        return BoolValue(entry != NULL);
    }
    return False;
}

/**
 * Determines what kind of access the calling thread has.
 */
LockAccess RWLOCK_GetAccess(const RWLock * lock) 
{
    if (lock->locks > 0) {
        const RWEntry * entry = RWLOCK_FindStaticEntry(lock);
        if (!entry && lock->entriesInUse > STATIC_ENTRIES) {
            /* must synchronize access to the "extended" entries */
            RWLOCK_GrabMutex(lock);
            entry = RWLOCK_FindExtEntry(lock);
            RWLOCK_ReleaseMutex(lock);
        }
        if (entry) {
            if (lock->flags & RWLOCK_FLAG_EXCLUSIVE_LOCK) {
                return Lock_Write;
            } else {
                return Lock_Read;
            }
        }
    }
    return Lock_None;
}

/**
 * Determines how many times this lock has been locked by the current thread.
 * This includes both read and write locks. If current thread does not own 
 * the lock, the return value is zero as you might expect.
 */
int RWLOCK_GetLockCount(const RWLock * lock) 
{
    if (lock->locks > 0) {
        const RWEntry * entry = RWLOCK_FindStaticEntry(lock);
        if (!entry && lock->entriesInUse > STATIC_ENTRIES) {
            /* must synchronize access to the "extended" entries */
            RWLOCK_GrabMutex(lock);
            entry = RWLOCK_FindExtEntry(lock);
            RWLOCK_ReleaseMutex(lock);
        }

        /* no need to synchronize access to our entry. 
         * Other threads won't touch it. */
        if (entry) {
            return (entry->read + entry->write);
        }
    }
    return 0;
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

#define _RWLockCast(s) CAST(s,RWLock,lock)
#if DEBUG
STATIC RWLock * RWLockCast(Lock * lock) {
    ASSERT(lock && lock->type == &LockType_RWLock);
    return _RWLockCast(lock);
}
STATIC const RWLock * RWLockCastC(const Lock * lock) {
    ASSERT(lock && lock->type == &LockType_RWLock);
    return _RWLockCast(lock);
}
#else
#  define RWLockCast(s)  _RWLockCast(s)
#  define RWLockCastC(s) _RWLockCast(s)
#endif

STATIC Bool _RWLOCK_Lock(Lock* l)  {return RWLOCK_WriteLock(RWLockCast(l));}
STATIC Bool _RWLOCK_TryLock(Lock*l){return RWLOCK_TryWriteLock(RWLockCast(l));}
STATIC int  _RWLOCK_Count(const Lock*l){return RWLOCK_GetLockCount(RWLockCastC(l));}
STATIC void _RWLOCK_Unlock(Lock* l){RWLOCK_Unlock(RWLockCast(l));}
STATIC void _RWLOCK_Free(Lock* l)  {RWLOCK_Delete(RWLockCast(l));}

STATIC const LockType LockType_RWLock = {
    TEXT("RWLock")      /* name     */,
    _RWLOCK_Lock        /* lock     */,
    _RWLOCK_TryLock     /* trylock  */,
    _RWLOCK_Count       /* count    */,
    _RWLOCK_Unlock      /* unlock   */,
    _RWLOCK_Free        /* free     */
};

/*
 * HISTORY:
 *
 * $Log: s_rwlock.c,v $
 * Revision 1.35  2011/03/03 15:28:46  slava
 * o a few minor fixes suggested by Apple's code analyzer
 *
 * Revision 1.34  2009/11/17 00:14:24  slava
 * o introducing generic synchronization API. This is a pretty destructive
 *   checkin, it's not 100% backward compatible. The Lock type is now a
 *   generic lock. What used to be called Lock is now RWLock. Sorry.
 *
 * Revision 1.33  2008/05/10 11:01:54  slava
 * o removed LockEntry::handle field. It no longer makes any sense after the
 *   recent changes in Win32 thread API implementation
 *
 * Revision 1.32  2007/12/01 15:34:48  slava
 * o fixed stupid typo in internal flag name
 *
 * Revision 1.31  2007/05/01 00:31:27  slava
 * o added LOCK_DropWrite() function
 *
 * Revision 1.30  2005/02/20 20:31:29  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.29  2004/04/08 12:21:42  slava
 * o made it compilable with C++ compiler
 *
 * Revision 1.28  2003/05/21 00:11:04  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.27  2003/02/02 01:26:40  slava
 * o (hopefully) fixed the race condition which almost never occurs on
 *   Windows with its brain dead round robin scheduling, but was quite
 *   common on Unix. say, thread A owns the lock and thread B is waiting
 *   for it. thread A releases the lock and then almost immediately
 *   attempts to get it back. the old implementation did not guarantee
 *   that thread B would grab the lock before thread A gets it. not good.
 *   this patch should fix this problem, i.e. thread A in the described
 *   situation would put itself to the end of the waiters queue and wait
 *   for its turn to run.
 *
 * Revision 1.26  2003/02/01 05:37:54  slava
 * o the goal of this patch is to make the behavior of the lock more
 *   predictable and less dependent on the underlying OS scheduler.
 *   The waiters get the lock in the order they started to wait; some
 *   exceptions apply to the waiters for shared access.
 *
 * Revision 1.25  2003/01/24 02:38:19  slava
 * o cleaned up ASSERT messages (don't need newline)
 *
 * Revision 1.24  2003/01/20 19:02:46  slava
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
 * Revision 1.23  2002/07/15 17:49:11  slava
 * o made timeout calculation a little bit more accurate (mark the start
 *   of wait before rather than after MUTEX_Lock call)
 *
 * Revision 1.22  2002/06/24 05:14:28  slava
 * o added LOCK_Create and LOCK_Delete
 *
 * Revision 1.21  2002/06/18 01:29:42  slava
 * o LOCK_CanRead, LOCK_CanWrite, LOCK_GetLockCount and LOCK_GetAccess
 *   take const pointer as a parameter
 *
 * Revision 1.20  2002/05/29 08:29:49  slava
 * o removed EVENT_InitModule and MUTEX_InitModule functions
 *
 * Revision 1.19  2002/02/03 01:46:59  slava
 * o invoke THREAD_InitModule() from LOCK_InitModule()
 *
 * Revision 1.18  2001/12/21 01:49:53  slava
 * o fixed compilation warnings in release build
 *
 * Revision 1.17  2001/12/06 04:31:11  slava
 * o added LOCK_UnlockMany() function
 * o changed LockAccess enum type to avoid conflict with LOCK_WRITE constant
 *   defined in one of Windows header files
 *
 * Revision 1.16  2001/11/25 21:25:13  slava
 * o InitModule functions now invoke dependent InitModule functions. That is,
 *   LOCK_InitModule invokes MUTEX_InitModule and EVENT_InitModule, etc.
 *
 * Revision 1.15  2001/11/24 19:39:19  slava
 * o cleaned up thread support for Unix platforms
 *
 * Revision 1.14  2001/10/08 05:17:51  slava
 * o eliminated some strict gcc warnings (mostly shadow declarations)
 *
 * Revision 1.13  2001/09/16 17:00:18  slava
 * o fixed some comments
 *
 * Revision 1.12  2001/08/14 03:24:11  slava
 * o cleanup
 *
 * Revision 1.11  2001/08/12 03:04:37  slava
 * o wait with timeout was broken. fixed it (hopefully)
 *
 * Revision 1.10  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.9  2001/05/18 22:36:57  slava
 * o THIS_FILE variable is no longer being used
 *
 * Revision 1.8  2001/05/18 22:29:54  slava
 * o slib has been (partially) ported to Windows CE
 *
 * Revision 1.7  2001/05/03 20:11:50  slava
 * o fixed a bug (a rare race condition between findStaticLockEntry() and
 *   LOCK_Unlock() code, which was introduced in rev 1.6). See comments
 *   in LOCK_Unlock().
 *
 * Revision 1.6  2001/03/27 06:08:17  slava
 * o unsynchronized some operations to improve performance
 *
 * Revision 1.5  2001/01/31 02:00:41  slava
 * o portability fixes. Now compiles on Solaris
 *
 * Revision 1.4  2000/11/01 13:25:05  slava
 * o replaced TRUE/FALSE with True/False
 *
 * Revision 1.3  2000/09/22 01:25:00  slava
 * o fixed a problem with Microsoft Visual C++ 6.0 generating bad code
 *   when optimization is on (had to replace macro with a function) -
 *   looks like a bug in Microsoft's compiler
 *
 * Revision 1.2  2000/08/21 10:41:50  slava
 * o more diagnostics in debug build
 *
 * Revision 1.1  2000/08/19 04:48:58  slava
 * o initial checkin
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
