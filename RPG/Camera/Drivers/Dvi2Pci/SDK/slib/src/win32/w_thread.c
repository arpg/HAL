/*
 * $Id: w_thread.c,v 1.10 2010/06/26 09:22:48 slava Exp $
 *
 * Copyright (C) 2000-2010 by Slava Monich
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
#include "s_mem.h"
#include "s_event.h"
#include "s_vector.h"

/**
 * Thread API implementation for Win32 environment
 */

#ifndef _WIN32
#  error "_WIN32 must be defined to compile Win32 code"
#endif

#define MAX_DESTRUCTOR_ITERATIONS 10

/*
 * Thread descriptor
 */
typedef struct _ThrData {
    LONG * signature;       /* points to WIN32_ThreadCount */
    LONG refCount;          /* reference count */
    ThrProc proc;           /* thread function */
    void * arg;             /* thread argument */
    DWORD  thrid;           /* Windows thread id */
    Vector cleanupList;     /* contains sorted ThrKey pointers */
    HANDLE handle;          /* thread handle returned by CreateThread */
    Event  exitEvent;       /* set by WIN32_ThreadProc on exit */
    LONG   detached;        /* incremented by THREAD_Detach */
} ThrData;

/*
 * Descriptor of a thread local value.
 */
struct _ThrKey {
    DWORD index;            /* TLS index */
    ThrClean clean;         /* the destructor */
    long ref;               /* ref count */
};

/**
 * Pointer to this variable is used as a signature
 */
STATIC LONG WIN32_ThreadCount = 0;
#define ThrData_Signature (&WIN32_ThreadCount)

/**
 * Used by THREAD_Self
 */
STATIC LONG WIN32_SelfHandleIndex = 0;
#define MIN_SELF_HANDLE_VALUE ((PtrWord)(&THREAD_Self))

/* 
 * Win32 thread local data.
 * 
 * WIN32_TLS_Data    points to ThrData structure for the thread
 *                   created by THREAD_Create. NULL for the threads
 *                   that were not created by SLIB
 * 
 * WIN32_TLS_ThrID   contains the thread id returned by THREAD_Self
 *                   functions.
 *
 * NOTE: it's the responsibility of the caller to close the handle
 * returned from THREAD_Create or THREAD_Self. It's done by calling
 * THREAD_Detach or THREAD_Join. 
 */
STATIC DWORD WIN32_TLS_Data = TLS_OUT_OF_INDEXES;
STATIC DWORD WIN32_TLS_ThrID = TLS_OUT_OF_INDEXES;

#define WIN32_FAILURE(what) \
    TRACE2("%s() failed, error %lu\n",TEXT(#what),GetLastError());

/**
 * This function checks if thread handle is valid
 */
STATIC Bool WIN32_IsThread(HANDLE handle) 
{
    if (handle) {
        DWORD status;
        if (GetExitCodeThread(handle,&status)) {
            return True;
        }
        WIN32_FAILURE(GetExitCodeThread);
        TRACE1("ERROR: not a thread handle: 0x%08lX\n",handle);
    }
    return False;
}

/**
 * Decrements the reference count and deallocates the ThrData structure
 * when it reaches zero.
 */
STATIC void THREAD_Deref(ThrData* thr) 
{
    ASSERT(thr->refCount > 0);
    ASSERT(thr->signature == ThrData_Signature);
    if (InterlockedDecrement(&thr->refCount) == 0) {
        ASSERT(!thr->cleanupList.data);
        if (!CloseHandle(thr->handle)) WIN32_FAILURE(CloseHandle);
        EVENT_Destroy(&thr->exitEvent);
        thr->signature = 0;
        MEM_Free(thr);
    }
}

/**
 * Destroys thread local variables, sets exit event and dereferences
 * the thread handle.
 */
STATIC void THREAD_Cleanup(ThrData* thr)
{
    /* invoke destructors for thread local variables */
    int i, n = 1;
    for (i=0; i<MAX_DESTRUCTOR_ITERATIONS && n > 0; i++) {
        int k;
        n = 0;
        for (k=0; k<VECTOR_Size(&thr->cleanupList); k++) {
            ThrKey key = VECTOR_Get(&thr->cleanupList, k);
            void * value = TlsGetValue(key->index);
            if (value) {
                TlsSetValue(key->index, NULL);
#ifdef _USE_EXCEPTION_HANDLING
                __try { 
#endif /* _USE_EXCEPTION_HANDLING */

                key->clean(value);
                ASSERT(!TlsGetValue(key->index));

#ifdef _USE_EXCEPTION_HANDLING
                } __except(EXCEPTION_EXECUTE_HANDLER) {
                    TlsSetValue(key->index,NULL);
                    ASSMSG1("EXCEPTION %08lX in cleanup proc!",
                            GetExceptionCode());
                }
#endif /* _USE_EXCEPTION_HANDLING */
                n++;
            }
        }
    }

    ASSERT(i<MAX_DESTRUCTOR_ITERATIONS);

    /* Dereference ThrKey structures */
    while ((n = VECTOR_Size(&thr->cleanupList)) > 0) {
        ThrKey key = VECTOR_Remove(&thr->cleanupList, n-1);
        if (InterlockedDecrement(&key->ref) == 0) {
            ASSERT(key->index == TLS_OUT_OF_INDEXES);
            MEM_Free(key);
        }
    }

    InterlockedDecrement(&WIN32_ThreadCount);
    VECTOR_Destroy(&thr->cleanupList);
    EVENT_Set(&thr->exitEvent);
    THREAD_Deref(thr);
}

/**
 * Win32-specific thread proc
 */
STATIC DWORD WINAPI WIN32_ThreadProc(LPVOID arg) 
{
    ThrData * thr = arg;

    /* This must succeed since we are not allocating any memory */
    VERIFY(VECTOR_Init(&thr->cleanupList,0,NULL,NULL));

#ifdef _USE_EXCEPTION_HANDLING
    __try { 
#endif /* _USE_EXCEPTION_HANDLING */

    ASSERT(TlsGetValue(WIN32_TLS_ThrID) == NULL);
    VERIFY(TlsSetValue(WIN32_TLS_ThrID, thr));
    if (WIN32_TLS_ThrID != TLS_OUT_OF_INDEXES) {
        ASSERT(TlsGetValue(WIN32_TLS_Data) == NULL);
        VERIFY(TlsSetValue(WIN32_TLS_Data, thr));
    }
    
    (*thr->proc)(thr->arg);

#ifdef _USE_EXCEPTION_HANDLING
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        ASSMSG1("EXCEPTION %08lX in thread proc!",GetExceptionCode());
    }
#endif /* _USE_EXCEPTION_HANDLING */

    THREAD_Cleanup(thr);
    return 0;
}

/**
 * Win32 specific initialization
 */
void THREAD_PlatformInitialize()
{
    WIN32_TLS_ThrID = TlsAlloc();
    ASSERT(WIN32_TLS_ThrID != TLS_OUT_OF_INDEXES);
    if (WIN32_TLS_ThrID != TLS_OUT_OF_INDEXES) {
        /* Allow this to fail */
        WIN32_TLS_Data = TlsAlloc();
        ASSERT(WIN32_TLS_Data != TLS_OUT_OF_INDEXES);
        return;
    }

    /* unrecoverable error */
    SLIB_Abort(TEXT("THREAD"));
}

/**
 * Win32 specific deinitialization
 */
void THREAD_PlatformShutdown()
{
    ASSERT(!WIN32_ThreadCount);
    TlsFree(WIN32_TLS_ThrID);
    WIN32_TLS_ThrID = TLS_OUT_OF_INDEXES;
    if (WIN32_TLS_Data != TLS_OUT_OF_INDEXES) {
        TlsFree(WIN32_TLS_Data);
        WIN32_TLS_Data = TLS_OUT_OF_INDEXES;
    }
}

/**
 * Create a new thread, returning True on success
 */
Bool THREAD_Create(ThrID* id, ThrProc proc, void * arg)
{
    ThrData * thr = NULL;
    ASSERT(THREAD_IsInited());

    if (id) *id = NULL;
    if (!THREAD_IsInited()) return False;

    /* allocate thread data */
    thr = MEM_New(ThrData);
    if (thr) {
        memset(thr, 0, sizeof(*thr));
        if (EVENT_Init(&thr->exitEvent)) {
            InterlockedIncrement(&WIN32_ThreadCount);
            thr->signature = ThrData_Signature;
            thr->refCount = 2;
            thr->proc = proc;
            thr->arg = arg;

            /* create Win32 thread */
            thr->handle = CreateThread(NULL, 0, WIN32_ThreadProc, thr,
                CREATE_SUSPENDED, &thr->thrid);

            ASSERT(thr->handle);
            if (thr->handle) {
                if (ResumeThread(thr->handle) != (DWORD)-1) {
                    if (id) *id = thr;
                    return True;
                } else {
                    WIN32_FAILURE(ResumeThread);
                    VERIFY(CloseHandle(thr->handle));
                }

            } else {
                WIN32_FAILURE(CreateThread);
            }

            EVENT_Destroy(&thr->exitEvent);
            InterlockedDecrement(&WIN32_ThreadCount);
        }
        MEM_Free(thr);
    }

    return False;
}

/**
 * Returns the thread ID of the calling thread. 
 */
ThrID THREAD_Self()
{
    ASSERT(THREAD_IsInited());
    if (WIN32_TLS_ThrID != TLS_OUT_OF_INDEXES) {
        HANDLE handle = (HANDLE)TlsGetValue(WIN32_TLS_ThrID);
        if (!handle) {
            /*
             * NOTE: we only get here if this functions is invoked by 
             * a thread  that was not created by THREAD_Create(). We
             * need to generate a reasonably unique number, that would
             * never equal any value that can be possibly returned by
             * THREAD_Create. We are taking the address of THREAD_Self
             * function (somewhere in the code segment) and every time
             * add a new value to it. That should do it.
             */
            handle = (HANDLE)(MIN_SELF_HANDLE_VALUE +
                InterlockedIncrement(&WIN32_SelfHandleIndex));
            VERIFY(TlsSetValue(WIN32_TLS_ThrID, handle));
        }
        return (ThrID)handle;
    } else {
        return NULL;
    }
}

/**
 * Checks whether the specified thread id is a fully functional thread id
 * created by THREAD_Create or a fake produced by THREAD_Self
 */
STATIC Bool THREAD_IsFakeID(ThrID thr)
{
    PtrWord handleValue = (PtrWord)thr;
    return (handleValue >= MIN_SELF_HANDLE_VALUE &&
            handleValue <= (MIN_SELF_HANDLE_VALUE + WIN32_SelfHandleIndex));
}

/**
 * Checks whether the specified ThrID represents the current thread
 */
Bool THREAD_IsSelf(ThrID tid)
{
    return tid && tid->thrid == GetCurrentThreadId();
}

/**
 * This call essentially invalidates ThrID variable. 
 * Detached thread cannot be "joined" (POSIX semantics)
 */
Bool THREAD_Detach(ThrID thr)
{
    Bool success = False;
    if (!THREAD_IsFakeID(thr)) {
        ASSERT(THREAD_IsInited());
        ASSERT(!IsBadWritePtr(thr, sizeof(thr)));
        if (!IsBadWritePtr(thr, sizeof(thr))) {
            ASSERT(thr->refCount > 0);
            ASSERT(thr->signature == ThrData_Signature);
            if (thr->refCount > 0 && thr->signature == ThrData_Signature) {
                ASSERT(WIN32_IsThread(thr->handle));
                if (InterlockedIncrement(&thr->detached) == 1) {
                    THREAD_Deref(thr);
                    success = True;
                } else {
                    /* This is a programming mistake */
                    InterlockedDecrement(&thr->detached);
                    TRACE("SLIB: thread detached more than once\n");
                }
            }
        }
    }

    return success;
}

/**
 * Waits until the specified thread terminates
 */
Bool THREAD_Join(ThrID thr)
{
    Bool success = False;

    ASSERT(THREAD_IsInited());

    /* make sure that this isn't a pseudo-handle created by THREAD_Self */
    if (!THREAD_IsFakeID(thr)) {
        ASSERT(!IsBadWritePtr(thr, sizeof(thr)));
        if (!IsBadWritePtr(thr, sizeof(thr))) {
            ASSERT(thr->signature == ThrData_Signature);
            ASSERT(thr->refCount > 0);
            ASSERT(!thr->detached);
            if (thr->signature == ThrData_Signature &&
                thr->refCount > 0 && !thr->detached) {

                ASSERT(WIN32_IsThread(thr->handle));

                /* thread can't join itself */
                if (!THREAD_IsSelf(thr)) {

                    /* WaitForSingleObject on a thread handle hangs forever
                     * if invoked from a DLL entry point. That's why we are
                     * using our own exit event here
                     */
                    HANDLE h[2];
                    h[0] = thr->exitEvent.handle;
                    h[1] = thr->handle;
                    WaitForMultipleObjects(COUNT(h), h, FALSE, INFINITE);
                    if (EVENT_State(&thr->exitEvent) == EVENT_NOT_SIGNALED) {
                        TRACE1("SLIB: thread %d was killed\n",thr->thrid);
                        THREAD_Deref(thr);
                    }
                    THREAD_Deref(thr);
                    success = True;
                } else {
                    ASSMSG("You've got a DEADLOCK, idiot!");
                }
            }
        }
    } else {
        TRACE("SLIB: can't join a thread not created by THREAD_Create\n");
    }

    return success;
}

/**
 * Terminates the current thread
 */
void THREAD_Exit()
{
    ASSERT(THREAD_IsInited());
    if (WIN32_TLS_ThrID != TLS_OUT_OF_INDEXES) {
        HANDLE handle = (HANDLE)TlsGetValue(WIN32_TLS_ThrID);
        if (handle) {
            ThrID thr = (ThrID)handle;
            ASSERT(thr->refCount > 0);
            ASSERT(thr->signature == ThrData_Signature);
            THREAD_Cleanup(thr);
        }
    }
    ExitThread(0);
}

/**
 * Relinguish the remainder of the time slice
 */
void THREAD_Yield()
{
    Sleep(0);
}

/**
 * Sleep with millisecond precision
 */
void THREAD_Sleep(long ms)
{
    Sleep(ms);
}

/*
 * Creates a thread-local key
 */
ThrKey THREAD_CreateKey(ThrClean proc)
{
    ASSERT(THREAD_IsInited());
    if (THREAD_IsInited()) {
        DWORD index = TlsAlloc();
        ASSERT(index != TLS_OUT_OF_INDEXES);
        if (index != TLS_OUT_OF_INDEXES) {
            ThrKey key = MEM_New(struct _ThrKey);
            if (key) {
                key->index = index;
                key->clean = proc;
                key->ref = 1;
                return key;
            }
            TlsFree(index);
        }
    }
    return NULL;
}

/**
 * Deletes the thread local key created by THREAD_CreateKey function.
 * Does not run the destructors.
 */
void THREAD_DeleteKey(ThrKey key)
{
    ASSERT(key);
    ASSERT(THREAD_IsInited());
    if (THREAD_IsInited() && key) {
        VERIFY(TlsFree(key->index));
        ASSERT(key->ref > 0);
        key->index = TLS_OUT_OF_INDEXES;
        if (InterlockedDecrement(&key->ref) == 0) {
            MEM_Free(key);
        }
    }
}

/* 
 * Checks if thread local value can be set for this key.
 *
 * NOTE: under Win32 we cannot run destructors for the thread that were 
 * not created by THREAD_Create(). Therefore, we refuse to set thread 
 * local value for such threads if key has a destructor.
 */
Bool THREAD_CanSetValue(ThrKey key)
{
    return (key && ((!key->clean) || (WIN32_TLS_Data != TLS_OUT_OF_INDEXES &&
        TlsGetValue(WIN32_TLS_Data))));
}

/**
 * Sets a thread-local value
 */
Bool THREAD_SetValue(ThrKey key, void * value)
{
    ASSERT(key);
    if (key) {

        /* 
         * NOTE: under Win32 we cannot run destructors for the thread that 
         * were not created by THREAD_Create(). Therefore, we refuse to set 
         * thread local value for such threads if key has a destructor.
         */
        if (THREAD_CanSetValue(key)) {
            if (key->clean) {
                int pos;
                ThrData* thr = TlsGetValue(WIN32_TLS_Data);
                void * prev = TlsGetValue(key->index);
                if (value && !prev) {
                    pos = VECTOR_Search(&thr->cleanupList,key,NULL);
                    if (pos < 0) {
                        InterlockedIncrement(&key->ref);
                        ASSERT(pos < 0);
                        if (VECTOR_Insert(&thr->cleanupList, -(pos+1), key)) {
                            if (TlsSetValue(key->index, value)) {
                                return True;
                            }
                            ASSMSG1("TlsSetValue err %u",GetLastError());
                            VECTOR_Remove(&thr->cleanupList,-(pos+1));
                        }
                        return False;
                    }
                }
            }

            /* set the thread local value */
            if (TlsSetValue(key->index, value)) {
                return True;
            }
            ASSMSG1("TlsSetValue err %u",GetLastError());
        } else {
            TRACE("SLIB: attempt to set TLS value for non-SLIB thread\n");
        }
    }
    return False;
}

/**
 * Returns a thread-local value
 */
void * THREAD_GetValue(ThrKey key)
{
    ASSERT(key);
    if (key) {
        ASSERT(key->index != TLS_OUT_OF_INDEXES);
        ASSERT(key->index != 0xfdfdfdfd);
        ASSERT(key->index != 0xcdcdcdcd);
        return TlsGetValue(key->index);
    }
    return NULL;
}

/*
 * HISTORY:
 *
 * $Log: w_thread.c,v $
 * Revision 1.10  2010/06/26 09:22:48  slava
 * o decrement WIN32_ThreadCount in THREAD_Cleanup. That helps to avoid
 *   occasional ASSERTs in THREAD_PlatformShutdown caused by a race condition
 *   between WIN32_ThreadProc and THREAD_Join which now gets unblocked by the
 *   exit event (as opposed to waiting on the thread handle)
 *
 * Revision 1.9  2010/02/11 15:33:40  slava
 * o THREAD_Join shouldn't block forever if thread got killed and haven't
 *   had a chance to set the exit event
 *
 * Revision 1.8  2010/02/09 08:53:43  slava
 * o it turned out that there is a big problem with WaitForSingleObject
 *   called on a thread handle - it hangs forever if invoked from a DLL
 *   entry point. Now THREAD_Join waits on an event handle rather than
 *   a thread handle. Which means that THREAD_Join doesn't REALLY waits
 *   for the thread to terminate. Instead, it just qurantees that the
 *   thread function has returned, which is essentially the same from
 *   any practical point of view.
 * o updated THREAD_Exit to destroy thread local variables and dereference
 *   thread data before calling ExitThread
 *
 * Revision 1.7  2009/10/21 13:22:09  slava
 * o added THREAD_IsSelf function
 *
 * Revision 1.6  2008/09/01 14:57:41  slava
 * o defined ThrID on Win32 as a pointer to struct _ThrData for better type
 *   safety
 *
 * Revision 1.5  2008/08/29 09:49:04  slava
 * o allow a thread to detach itself, there may be perfectly valid reasons
 *   for doing that. Actually, it was already allowed, but triggered an
 *   unnecessary ASSERT.
 *
 * Revision 1.4  2008/05/10 10:58:52  slava
 * o quite a significant update. Resolved the issues with handle reuse (thread
 *   handle could be reused by Windows after it's closed) and handle leaks (if
 *   THREAD_Self was called by a thread that was not created by SLIB).
 *
 * Revision 1.3  2005/02/25 02:53:40  slava
 * o TLS code moved to platform specific area
 *
 * Revision 1.2  2005/02/19 01:13:00  slava
 * o cleanup
 *
 * Revision 1.1  2005/02/19 00:37:41  slava
 * o separated platform specific code from platform independent code
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
