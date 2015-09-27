/*
 * $Id: s_mem.c,v 1.41 2010/10/26 05:47:11 slava Exp $
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

#include "s_def.h"
#include "s_util.h"
#include "s_libp.h"
#include "s_mem.h"

/*
 * This counter is incremented by MEM_Alloc and MEM_Realloc after each 
 * successful memory (re)allocation. NOTE: this counter may not be 100%
 * accurate in release build because its increment is not synchronized.
 * Which is OK since we don't need the count to be accurate in the
 * release build. It's being used by MEM_Hook which only cares whether
 * it's zero or not. In debug build the count is perfectly synchronized.
 */
STATIC I64s totalAllocations = 0;

/* Maximum value of size_t */
#define MAX_SIZE_T ((size_t)(-1))

/*==========================================================================*
 *              D E B U G
 *==========================================================================*/

/**
 * This part is only for debugging
 */

#ifndef DEBUG_ALLOC
#  if DEBUG && !(defined(_UNIX) && defined(__KERNEL__))
#    define DEBUG_ALLOC 1
#  else
#    define DEBUG_ALLOC 0
#  endif
#endif /* DEBUG_ALLOC */

#if DEBUG_ALLOC
#include "s_mutex.h"
#include "s_queue.h"

Bool memAssertOnAllocFailure = True;

/**
 * Memory statistics.
 */
STATIC I64s bytesAllocated = 0;
STATIC I64s blocksAllocated = 0;

STATIC I64s maxBytesAllocated = 0;
STATIC I64s maxBlocksAllocated = 0;

STATIC Mutex memMutex;          /* protects data from being corrupted */
STATIC int  memInitCount = 0;   /* initialization count */

#ifdef DEBUG_MEM
STATIC Queue memQueue;                  /* all allocated memory blocks */ 
STATIC size_t memSizeBreak = (size_t)-1;  /* break on allocation size */
#endif /* DEBUG_MEM */

#define MEM_SIGNATURE  _SIGNATURE32('M','E','M',' ')
#define MEM_DEAD       _SIGNATURE32('D','E','A','D')
#define OVERHEAD       OFFSET(OSMemBlock,data)

typedef struct _OSMemHeader {
    I32u   sign;                /* signature */
    size_t size;                /* usable size */
#ifdef DEBUG_MEM
    I64s   num;                 /* unique block number */
    QEntry e;                   /* queue entry */ 
#endif /* DEBUG_MEM */
} OSMemHeader;

typedef struct _OSMemBlock {
    OSMemHeader head;           /* header */
    char data[1];               /* user data */
} OSMemBlock;

/**
 * Check if address is a valid pointer into a memory block allocated 
 * with one of our memory management routines. If so, returns pointer
 * to OSMemBlock, otherwise returns NULL.
 */
STATIC OSMemBlock * validateMemBlock(void * ptr) 
{
    if (ptr) {
        OSMemBlock * mb = CAST(ptr,OSMemBlock,data);
        if (mb->head.sign == MEM_SIGNATURE) {
            return mb;
        } else if (mb->head.sign == MEM_DEAD) {
            ASSMSG1("ERROR: dead memory block at 0x%08lX",(unsigned long)ptr); 
        } else {
            ASSMSG1("ERROR: bad memory block at 0x%08lX",(unsigned long)ptr); 
        }        
    }
    return NULL;
}

/**
 * Allocates requested amount of memory.
 */
STATIC void * MEM_DebugAlloc(size_t size) 
{
    size_t mbsize = size + OVERHEAD;
    OSMemBlock * mb = (OSMemBlock *)OS_MemAlloc(mbsize);
    if (memInitCount == 0) MEM_InitModule();
    if (mb) {
        Bool haveMutex;
        void * ptr = mb->data;
        mb->head.sign = MEM_SIGNATURE;
        mb->head.size = size;

        /* lock global data */
        haveMutex = MUTEX_Lock(&memMutex);
        ASSERT(haveMutex);

        /* update statistics */
        blocksAllocated++;
        bytesAllocated += size;
        if (maxBytesAllocated < bytesAllocated) {
            maxBytesAllocated = bytesAllocated;
        }
        if (maxBlocksAllocated < blocksAllocated) {
            maxBlocksAllocated = blocksAllocated;
        }

#ifdef DEBUG_MEM
        ASSERT(size != memSizeBreak);
        QUEUE_InsertTail(&memQueue, &mb->head.e);
        ASSERT(blocksAllocated == memQueue.size);
        mb->head.num = ++totalAllocations;
#endif /* DEBUG_MEM */

        /* unlock global data */
        if (haveMutex) MUTEX_Unlock(&memMutex);

        return ptr;
    }
    return NULL;
}

/**
 * Deallocate memory block
 */
STATIC void MEM_DebugFree(void * ptr) 
{
    ASSERT(memInitCount > 0);
    if (ptr) {
        OSMemBlock * mb = validateMemBlock(ptr);
        if (mb) {

            /* lock global data */
            Bool haveMutex = MUTEX_Lock(&memMutex);
            ASSERT(haveMutex);

            /* update statistics */
            blocksAllocated--;
            bytesAllocated -= mb->head.size;            
            ASSERT(blocksAllocated >= 0);
            ASSERT(bytesAllocated >= 0);

#ifdef DEBUG_MEM
            QUEUE_RemoveEntry(&mb->head.e);
            ASSERT(blocksAllocated == memQueue.size);
#endif /* DEBUG_MEM */

            /* unlock global data */
            if (haveMutex) MUTEX_Unlock(&memMutex);

            /* invalidate the signature and free the memory */
            mb->head.sign = MEM_DEAD;

            OS_MemFree(mb);
        }
    }
}

/**
 * Changes the size of the memory block pointed to by ptr to size bytes. 
 * The contents will be  unchanged to the minimum of the old and new sizes;
 * newly allocated memory will be uninitialized.  If ptr is NULL, the call
 * is equivalent  to MEM_Alloc(size); if size is equal to zero, the call 
 * is equivalent to MEM_Free(ptr). Unless ptr is NULL, it must have been 
 * returned by an earlier call to MEM_Alloc() or MEM_Realloc().
 */
STATIC void * MEM_DebugRealloc(void * ptr, size_t size) 
{
    if (memInitCount == 0) MEM_InitModule();
    if (!size) {
        MEM_DebugFree(ptr);
        return NULL;
    } else if (ptr) {
        OSMemBlock * mb = validateMemBlock(ptr);
        if (mb) {
            Bool haveMutex;
            OSMemBlock * newmb;
            size_t prevsize = mb->head.size;
            size_t mbsize = size + OVERHEAD;

            /* invalidate signature before reallocating the memory block */
            mb->head.sign = MEM_DEAD;

            /* lock global data */
            haveMutex = MUTEX_Lock(&memMutex);
            ASSERT(haveMutex);

#ifdef DEBUG_MEM
            ASSERT(size != memSizeBreak);
            ASSERT(blocksAllocated == memQueue.size);
            QUEUE_RemoveEntry(&mb->head.e);
            
            /* 
             * Temporary decrement blocksAllocated counter because we
             * are to release the mutex. Otherwise, another memory 
             * allocation may occur in between and we will get an
             * assertion failure because blocksAllocatedisn't equal 
             * memQueue.size
             */
            blocksAllocated--;
#endif /* DEBUG_MEM */

            /* unlock global data */
            if (haveMutex) MUTEX_Unlock(&memMutex);

#ifdef NO_REALLOC
            newmb = (OSMemBlock*)OS_MemAlloc(mbsize);
            if (newmb) {
                memcpy(newmb, mb, mb->head.size + OVERHEAD);
                OS_MemFree(mb);
            }
#else  /* NO_REALLOC */
            newmb = (OSMemBlock*)OS_MemRealloc(mb,mbsize);
#endif /* NO_REALLOC */
            if (newmb) {
                newmb->head.sign = MEM_SIGNATURE;
                newmb->head.size = size;

                /* lock global data */
                haveMutex = MUTEX_Lock(&memMutex);
                ASSERT(haveMutex);

                /* update statistics */
                if (prevsize > size) {
                    bytesAllocated -= (prevsize-size);
                } else {
                    bytesAllocated += (size-prevsize);
                    if (maxBytesAllocated < bytesAllocated) {
                        maxBytesAllocated = bytesAllocated;
                    }
                }

#ifdef DEBUG_MEM
                /* restore the blocksAllocated counter */
                blocksAllocated++;
                QUEUE_InsertTail(&memQueue, &newmb->head.e);
                ASSERT(blocksAllocated == memQueue.size);
                newmb->head.num = ++totalAllocations;
#endif /* DEBUG_MEM */

                /* unlock global data */
                if (haveMutex) MUTEX_Unlock(&memMutex);

                /* return new address to the caller */
                return newmb->data;
            } else {

                /* reallocation failed, restore signature */
                mb->head.sign = MEM_SIGNATURE;                

#ifdef DEBUG_MEM
                /* restore the blocksAllocated counter */
                haveMutex = MUTEX_Lock(&memMutex);
                blocksAllocated++;
                QUEUE_InsertTail(&memQueue, &newmb->head.e);
                ASSERT(blocksAllocated == memQueue.size);
                if (haveMutex) MUTEX_Unlock(&memMutex);
#endif /* DEBUG_MEM */

            }
        }
        return NULL;
    } else {
        return MEM_DebugAlloc(size);
    }
}

/**
 * Returns True if pointer appears to point to the beginning of a valid 
 * memory block.
 */
STATIC Bool _ValidPointer(void * ptr) 
{
    return BoolValue(validateMemBlock(ptr) != NULL);
}

/**
 * Dumps some memory statistics to standard output
 */
void MEM_DebugStat(PrintProc print) 
{
    Bool haveMutex = (memInitCount ? MUTEX_Lock(&memMutex) : False);
#ifdef DEBUG_MEM
    ASSERT(blocksAllocated == memQueue.size);
#endif /* DEBUG_MEM */

    print(T_("Total allocations:   %ld\n"),(long)totalAllocations);
    print(T_("Peak allocations:    %ld blocks\n"),(long)maxBlocksAllocated);
    print(T_("                     %ld bytes\n"),(long)maxBytesAllocated);
    print(T_("Currently allocated: %ld blocks\n"),(long)blocksAllocated);
    print(T_("                     %ld bytes\n"),(long)bytesAllocated);

    if (haveMutex) MUTEX_Unlock(&memMutex);
}

#ifdef DEBUG_MEM
typedef struct _MemDump {
    PrintProc print;
    int count;
} MemDump;

/**
 * Callback for _DebugMemDump().
 * Returns True to continue, False to stop examining the queue.
 */
STATIC Bool debugMemDumpCB(QEntry* e, void * ctx) 
{
    MemDump* dump = (MemDump*)ctx;
    OSMemBlock * block = QCAST(e, OSMemBlock, head.e);
    dump->count++;
    dump->print(T_("%d. Memory block #%lu at 0x%08lX, %lu bytes:\n"),
        dump->count, (unsigned long)block->head.num,
        (unsigned long)block->data, (unsigned long)block->head.size);
    PRINT_Dump2(dump->print, block->data, block->head.size, 0, 4096);
    return True;
}
#endif /* DEBUG_MEM */

/**
 * Dumps all allocated memory to standard output. 
 */
void MEM_DebugDump(PrintProc print) 
{
#ifdef DEBUG_MEM
    if (memInitCount > 0) {
        MemDump dump;
        Bool haveMutex = MUTEX_Lock(&memMutex);
        ASSERT(haveMutex);
        dump.count=0;
        dump.print = print;
        QUEUE_Examine(&memQueue, debugMemDumpCB, &dump);
        if (haveMutex) MUTEX_Unlock(&memMutex);
    }
#else  /* !DEBUG_MEM */
    print(T_("Memory dump not available (try compiling with DEBUG_MEM)\n"));
#endif /* !DEBUG_MEM */
}

#endif /* DEBUG_ALLOC */

/*==========================================================================*
 *              M E M O R Y    H O O K
 *==========================================================================*/

struct _MemHook {
    MemProc proc;
    MemContext * ctx;
    const MemHook * next;
};

/**
 * Returns the next hook.
 */
const MemHook * MEM_NextHook(const MemHook * hook)
{
    return (hook ? hook->next : NULL);
}

/**
 * The following three functions call the appropriate hook functions
 */
void * MEM_Alloc2(const MemHook * hook, size_t size)
{
    while (!hook->proc.memAlloc) hook = hook->next;
    return (*(hook->proc.memAlloc))(hook->ctx, size);
}

void * MEM_Realloc2(const MemHook * hook, void * ptr, size_t size)
{
    while (!hook->proc.memRealloc) hook = hook->next;
    return (*(hook->proc.memRealloc))(hook->ctx, ptr, size);
}

void MEM_Free2(const MemHook * hook, void * ptr)
{
    while (!hook->proc.memFree) hook = hook->next;
    (*(hook->proc.memFree))(hook->ctx, ptr);
}

/*==========================================================================*
 *              D E F A U L T    H O O K
 *==========================================================================*/

STATIC void * MEM_DefaultAlloc(MemContext * ctx, size_t size) 
{
    UNREF(ctx);
#if DEBUG_ALLOC
    return MEM_DebugAlloc(size);
#else  /* DEBUG_ALLOC */
    return OS_MemAlloc(size);
#endif /* DEBUG_ALLOC */
}

STATIC void * MEM_DefaultRealloc(MemContext * ctx, void * ptr, size_t size) 
{
    UNREF(ctx);
#ifndef NO_REALLOC
#  if DEBUG_ALLOC
    return MEM_DebugRealloc(ptr, size);
#  else  /* DEBUG_ALLOC */
    return OS_MemRealloc(ptr, size);
#  endif /* DEBUG_ALLOC */
#else  /* NO_REALLOC */
    ASSMSG("MEM_DefaultRealloc");
    return OS_MemAlloc(size);
#endif /* NO_REALLOC */
}

STATIC void   MEM_DefaultMemFree(MemContext * ctx, void * ptr) 
{
    UNREF(ctx);
#if DEBUG_ALLOC
    MEM_DebugFree(ptr);
#else  /* DEBUG_ALLOC */
    OS_MemFree(ptr);
#endif /* DEBUG_ALLOC */
}

/*==========================================================================*
 *              I N T E R F A C E
 *==========================================================================*/

STATIC const MemHook memDefaultHook = {
    {                       /* proc */
        NULL,               /* memInit */
        MEM_DefaultAlloc,   /* memAlloc */
        MEM_DefaultRealloc, /* memRealloc */
        MEM_DefaultMemFree  /* memFree */
    },
    NULL,                   /* ctx */
    NULL                    /* next */
};

STATIC const MemHook * memHook = &memDefaultHook;

/**
 * Initialize the module. Has no effect in release build
 */
void MEM_InitModule() 
{
#if DEBUG_ALLOC
    if ((memInitCount++) == 0) {
#ifdef DEBUG_MEM
        /* 
         * Protection against MEM_InitModule() called after
         * MEM_Shutdown() with some memory still allocated
         */
        if (!memQueue.head.queue) {
            QUEUE_Init(&memQueue);
        }
#endif /* DEBUG_MEM */
        if (!MUTEX_Init(&memMutex)) {
            /* unrecoverable error */
            SLIB_Abort(T_("MEM"));
        }
    }
#endif /* DEBUG_ALLOC */
}

/**
 * Shutdown the module. Has no effect in release build
 */
void MEM_Shutdown() 
{
#if DEBUG_ALLOC
    if (memInitCount == 1) {
        MEM_DumpStat();
        MEM_Dump();
        memInitCount = 0;
        MUTEX_Destroy(&memMutex);
    } else {
        memInitCount--;
    }
#endif /* DEBUG_ALLOC */
}

/**
 * Allocates size bytes and returns a pointer to the allocated memory.  
 * The memory is not cleared.
 */
void * MEM_Alloc(size_t size)
{
    void * ptr = MEM_Alloc2(memHook, size);
    if (ptr) {

        /*
         * In debug build totalAllocations is being incremented by 
         * MEM_DebugAlloc and MEM_DebugRealloc under synchronization.
         * In release build, we have to do it here. Note that since
         * this operation is not synchronized in release build, this
         * counter may not be accurate. This is OK because it's not
         * being used in release build in any way, except by MEM_Hook
         * to check whether something has been allocated or not;
         * it does not need accurate count.
         */
#if !DEBUG_ALLOC
        totalAllocations++;
#endif /* !DEBUG_ALLOC */
        return ptr;
    }
    DTRACE1(T_("ERROR! failed to malloc ") T_(I64U_FORMAT) T_(" bytes!\n"),
        (I64u)size);
#if DEBUG_ALLOC
    if (memAssertOnAllocFailure) {
        ASSMSG("ERROR! memory allocation failure");
    }
#endif /* !DEBUG_ALLOC */
    return NULL;
}

/**
 * Allocates memory for the specified number of elements of the specified
 * size. Allocated memory is not initialized.
 */
void * MEM_AllocN(size_t count, size_t size)
{
    /* check for overflow */
    if (!size || (MAX_SIZE_T/size) >= count) {
        return MEM_Alloc(count*size);
    } else {
        DTRACE2(T_("ERROR! can't malloc ") T_(I64U_FORMAT) T_("*")
            T_(I64U_FORMAT) T_(" bytes!\n"), (I64u)count, (I64u)size);
#if DEBUG_ALLOC
        if (memAssertOnAllocFailure) {
            ASSMSG("ERROR! memory allocation failure");
        }
#endif /* !DEBUG_ALLOC */
        return NULL;
    }
}

/**
 * Allocates memory for the specified number of elements of the specified
 * size. Allocated memory is initialized to zero.
 */
void * MEM_Calloc(size_t count, size_t size)
{
    /* check for overflow */
    if (!size || (MAX_SIZE_T/size) >= count) {
        size_t total = count*size;
        void * ptr = MEM_Alloc(total);
        if (ptr) {
            memset(ptr, 0, total);
            return ptr;
        }
    } else {
        DTRACE2(T_("ERROR! can't calloc ") T_(I64U_FORMAT) T_("*")
            T_(I64U_FORMAT) T_(" bytes!\n"), (I64u)count, (I64u)size);
#if DEBUG_ALLOC
        if (memAssertOnAllocFailure) {
            ASSMSG("ERROR! memory allocation failure");
        }
#endif /* !DEBUG_ALLOC */
    }
    return NULL;
}

/**
 * Changes the size of the memory block pointed to by ptr to size bytes. 
 * The contents will be  unchanged to the minimum of the old and new sizes;
 * newly allocated memory will be uninitialized.  If ptr is NULL, the call
 * is equivalent  to MEM_Alloc(size); if size is equal to zero, the call 
 * is equivalent to MEM_Free(ptr). Unless ptr is NULL, it must have been 
 * returned by an earlier call to MEM_Alloc() or MEM_Realloc().
 */
#ifndef NO_REALLOC
void * MEM_Realloc(void * ptr, size_t size) 
{
    void * newptr = MEM_Realloc2(memHook, ptr, size);
    if (newptr) {

        /*
         * In debug build totalAllocations is being incremented by 
         * MEM_DebugAlloc and MEM_DebugRealloc under synchronization.
         * In release build, we have to do it here. Note that since
         * this operation is not synchronized in release build, this
         * counter may not be accurate. This is OK because it's not
         * being used in release build in any way, except by MEM_Hook
         * to check whether something has been allocated or not;
         * it does not need accurate count.
         */
#if !DEBUG_ALLOC
        totalAllocations++;
#endif /* !DEBUG_ALLOC */

        return newptr;
    } 
#if DEBUG_ALLOC
    if (size) {
        DTRACE1(T_("ERROR! failed to realloc ")T_(I64U_FORMAT)T_(" bytes!\n"),
            (I64u)size);
        if (memAssertOnAllocFailure) {
            ASSMSG("Memory reallocation failure");
        }
    }
#endif /* !DEBUG_ALLOC */
    return NULL;
}

/**
 * Changes the size of the memory block pointed to by ptr. Basically,
 * does the same thing as MEM_Realloc, but deals with the possibility
 * of overflow.
 */
void * MEM_ReallocN(void * ptr, size_t count, size_t size)
{
    /* check for overflow */
    if (!size || (MAX_SIZE_T/size) >= count) {
        return MEM_Realloc(ptr, count*size);
    } else {
#if DEBUG_ALLOC
        DTRACE2(T_("ERROR! can't realloc ")T_(I64U_FORMAT)T_("*")
            T_(I64U_FORMAT)T_(" bytes!\n"), (I64u)count, (I64u)size);
        if (memAssertOnAllocFailure) {
            ASSMSG("Memory reallocation failure");
        }
#endif /* DEBUG_ALLOC */
        return NULL;
    }
}
#endif /* NO_REALLOC */

/**
 * Frees  the  memory  space pointed to by ptr, which must have been 
 * returned by a previous call to MEM_Alloc() or MEM_Realloc(). Otherwise, 
 * or if MEM_Free(ptr) has already been called before, undefined behaviour
 * occurs. If ptr is NULL, no operation is performed.
 */
void MEM_Free(void * ptr)
{
    MEM_Free2(memHook, ptr);
}

/**
 * Compares two blocks of memory
 */
Bool MEM_Equal(const void * p1, const void * p2, size_t size)
{
    if ((p1 == p2) || !size) {
        return True;
    } else {
#ifdef _NT_KERNEL
        /* some DDKs come with ntoskrnl.lib that doesn't export memcmp */
        return (RtlCompareMemory(p1, p2, size) == size);
#else
        return !memcmp(p1, p2, size);
#endif
    }
}

/**
 * Registers a hook. Memory hooks stack on top of each other. Once registered, 
 * the hook remains there forever because there's no simple and efficient way 
 * to keep track of which memory block allocated by which hook, nor there's a 
 * need for it. There's a default "hook", always the last one, that actually 
 * calls OS specific memory allocation functions. 
 *
 * Memory hooks must be registered at startup, before one piece of memory
 * is allocated by MEM_Alloc
*/
const MemHook * MEM_Hook(const MemProc * proc, MemContext * ctx) 
{
    if (!totalAllocations) {
        ASSERT(proc);
        if (proc) {
            MemHook * hook = (MemHook*)OS_MemAlloc(sizeof(MemHook));
            ASSERT(proc->memAlloc || proc->memRealloc || proc->memFree);
            if (hook) {
                memset(hook, 0, sizeof(MemHook));
                hook->proc = *proc;
                hook->ctx = ctx;
                hook->next = memHook;
                if (!proc->memInit) {
                    memHook = hook;
                    return hook;
                } else if ((*(proc->memInit))(ctx, hook->next)) {
                    memHook = hook;
                    return hook;
                }
                OS_MemFree(hook);
            }
        }
    } else {
        ASSMSG("It's too late to call MEM_Hook!");
    }
    return NULL;
}

/*
 * HISTORY:
 *
 * $Log: s_mem.c,v $
 * Revision 1.41  2010/10/26 05:47:11  slava
 * o fixed Unicode build
 *
 * Revision 1.40  2010/09/29 07:10:15  slava
 * o fixed Unicode build
 *
 * Revision 1.39  2010/09/28 19:47:12  slava
 * o fixing format/argument mismatch issues
 *
 * Revision 1.38  2010/09/25 10:13:49  slava
 * o fixed 64-bit format warning
 *
 * Revision 1.37  2010/09/25 09:55:06  slava
 * o added PRINTF_ATTR macro which assigns printf-like characteristics to the
 *   declared function and enables gcc to check the format string against the
 *   parameters.
 *
 * Revision 1.36  2010/09/25 09:31:32  slava
 * o made it possible to compile some slib modules in Mac OS X kernel build
 *   environment
 *
 * Revision 1.35  2010/06/26 09:26:42  slava
 * o use MEM_Equal instead of memcmp to resolve linking problem with some
 *   versions of Windows DDK
 *
 * Revision 1.34  2010/06/21 16:25:34  slava
 * o only include s_mutex.h and s_queue.h in debug build
 *
 * Revision 1.33  2007/04/29 18:49:19  slava
 * o removed erroneous #ifdef _WIN32
 *
 * Revision 1.32  2007/03/16 20:53:09  slava
 * o added runtime flag memAssertOnAllocFailure in debug build; useful for
 *   debugging aplications which may legitimately run out of memory.
 *
 * Revision 1.31  2006/11/03 17:27:20  slava
 * o changed MEM_DebugDump to not dump more than first 4K of each leaked
 *   memory block
 *
 * Revision 1.30  2006/10/29 22:49:06  slava
 * o MEM_NewArray, MEM_ReallocArray and MEM_Calloc should deal with
 *   possible block size overflow. Without checking for overflow, they
 *   may end up allocating less than requested amount of memory.
 *
 * Revision 1.29  2005/02/20 20:31:30  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.28  2005/01/07 14:16:15  slava
 * o fixed an ASSERT that was cause by race condition during reallocation
 *   of a memory block in debug build. This problem didn't affect the build.
 *
 * Revision 1.27  2004/12/30 23:20:55  slava
 * o updated comments
 *
 * Revision 1.26  2004/12/30 23:16:53  slava
 * o added a few more ASSERTs
 * o handle the case if MEM_Shutdown is followed by MEM_InitModule with
 *   some memory still allocated. We didn't quite handle it before. Such
 *   a sequence would cause ASSERT or even crash, although only on the
 *   debug build. Release build was working fine.
 *
 * Revision 1.25  2004/12/26 18:22:19  slava
 * o support for CodeWarrior x86 compiler
 *
 * Revision 1.24  2004/04/08 00:59:10  slava
 * o QCAST macro was used instead of CAST
 *
 * Revision 1.23  2003/11/29 19:52:10  slava
 * o need to include s_mutex.h rather than s_lock.h
 *
 * Revision 1.22  2003/11/03 21:48:30  slava
 * o added MEM_Calloc function
 *
 * Revision 1.21  2003/05/21 00:11:04  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.20  2003/01/24 02:38:19  slava
 * o cleaned up ASSERT messages (don't need newline)
 *
 * Revision 1.19  2003/01/20 19:02:46  slava
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
 * Revision 1.18  2002/08/27 11:52:03  slava
 * o some reformatting
 *
 * Revision 1.17  2002/05/29 08:29:49  slava
 * o removed EVENT_InitModule and MUTEX_InitModule functions
 *
 * Revision 1.16  2001/11/25 21:25:13  slava
 * o InitModule functions now invoke dependent InitModule functions. That is,
 *   LOCK_InitModule invokes MUTEX_InitModule and EVENT_InitModule, etc.
 *
 * Revision 1.15  2001/10/29 03:34:14  slava
 * o synchronize increment of totalAllocations counter in debug build.
 *   In release build, leave it unsynchronized. This is OK because it's
 *   not being used in release build in any way, except by MEM_Hook
 *   to check whether something has been allocated or not; it does
 *   not need accurate count.
 *
 * Revision 1.14  2001/10/22 00:10:36  slava
 * o recrafted memory management.
 *
 * Revision 1.13  2001/06/30 16:33:04  slava
 * o added MEM_InitModule() and MEM_Shutdown() which in release build do
 *   nothing. In debug build, MEM_Shutdown() makes sure that memory mutex
 *   gets destroyed. While under Windows NT/98 or on Unix it's not a problem
 *   because all handles get closed by the system when the process dies, in
 *   some scenarios (like, NT kernel mode or perhaps Windows CE) this may
 *   cause a resource leak.
 *
 * Revision 1.12  2001/06/30 15:47:45  slava
 * o replaces some error TRACEs with ASSERTs
 * o added option to break on allocation of memory block of certain size
 *
 * Revision 1.11  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.10  2001/05/27 11:45:51  slava
 * o port to NT kernel mode environment
 *
 * Revision 1.9  2001/05/20 03:13:54  slava
 * o fixed a Windows CE specific problem
 *
 * Revision 1.8  2001/05/18 22:36:57  slava
 * o THIS_FILE variable is no longer being used
 *
 * Revision 1.7  2001/05/18 22:29:54  slava
 * o slib has been (partially) ported to Windows CE
 *
 * Revision 1.6  2001/03/31 07:17:43  slava
 * o use LONG_LONG_FORMAT for formatting 64 bit numbers
 *
 * Revision 1.5  2001/03/17 07:41:37  slava
 * o changed MEM_DebugDump() so that it does not ASSERT in case if the
 *   module has never been initialized (because there were no memory
 *   allocation requests)
 *
 * Revision 1.4  2000/11/05 07:07:49  slava
 * o reorganization of output functions (Verbose(), Output(), etc.)
 *
 * Revision 1.3  2000/11/01 13:25:05  slava
 * o replaced TRUE/FALSE with True/False
 *
 * Revision 1.2  2000/08/25 11:21:51  slava
 * o fixed potention data loss (int64 -> long conversion)
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
