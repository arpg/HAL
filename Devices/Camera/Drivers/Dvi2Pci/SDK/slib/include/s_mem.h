/*
 * $Id: s_mem.h,v 1.13 2010/06/26 09:26:41 slava Exp $
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

#ifndef _SLAVA_MEM_H_
#define _SLAVA_MEM_H_

#include "s_def.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* memory allocation */

extern void   MEM_InitModule P_((void));
extern void   MEM_Shutdown P_((void));

extern void * MEM_Alloc P_((size_t size));
extern void * MEM_AllocN P_((size_t count, size_t size));
extern void * MEM_Calloc P_((size_t count, size_t size));
extern void * MEM_Realloc P_((void * ptr, size_t size));
extern void * MEM_ReallocN P_((void * ptr, size_t count, size_t size));
extern void   MEM_Free P_((void * ptr));

/* utilities */
extern Bool   MEM_Equal P_((const void * p1, const void * p2, size_t size));

/* memory allocation interface */

typedef struct _MemHook MemHook;
typedef struct _MemContext MemContext;

typedef Bool   (*MemInit) P_((MemContext * ctx, const MemHook * next));
typedef void * (*MemAlloc) P_((MemContext * ctx, size_t size));
typedef void * (*MemRealloc) P_((MemContext * ctx, void * ptr, size_t size));
typedef void   (*MemFree) P_((MemContext * ctx, void * ptr));

/*
 * Memory hooks stack on top of each other. Once registered, the hook remains 
 * there forever because there's no simple and efficient way to keep track of
 * which memory block allocated by which hook, nor there's a need for it. 
 * There's a default "hook", always the last one, that actually calls OS 
 * specific memory allocation functions. 
 *
 * Even though this mechanism allows for arbitrary number of hooks, it's not 
 * expected that there will be more than one (on top of the default one)
 *
 * Memory hooks must be registered at startup, before one piece of memory
 * is allocated by MEM_Alloc
 */
typedef struct _MemProc {
    MemInit    memInit; 
    MemAlloc   memAlloc;        /* malloc */
    MemRealloc memRealloc;      /* realloc */
    MemFree    memFree;         /* free */
} MemProc;

extern const MemHook * MEM_Hook P_((const MemProc * proc, MemContext * ctx));
extern const MemHook * MEM_NextHook P_((const MemHook * hook));

extern void * MEM_Alloc2 P_((const MemHook * hook, size_t size));
extern void * MEM_Realloc2 P_((const MemHook * hook, void * ptr, size_t size));
extern void   MEM_Free2 P_((const MemHook * hook, void * ptr));

/* debugging tricks */

#if DEBUG

extern void  MEM_DebugStat P_((int (*print) P_((const Char * format, ... ))));
extern void  MEM_DebugDump P_((int (*print) P_((const Char * format, ... ))));
extern Bool  MEM_IsValidPointer P_((void * ptr));

#define MEM_DumpStat()          MEM_DebugStat(DEBUG_Trace)
#define MEM_Dump()              MEM_DebugDump(DEBUG_Trace)

#define ASSERT_VALID_POINTER(p) ASSERT(MEM_IsValidPointer(p))

#else /* !DEBUG */

#define MEM_DumpStat()           NOTHING
#define MEM_Dump()               NOTHING

#define ASSERT_VALID_POINTER(p)  NOTHING

#endif /* !DEBUG */

#define MEM_New(type)              ((type *)MEM_Alloc(sizeof(type)))
#define MEM_NewArray(type,n)       ((type *)MEM_AllocN(n,sizeof(type)))
#define MEM_ReallocArray(p,type,n) ((type *)MEM_ReallocN(p,n,sizeof(type)))

/*
 * NO_REALLOC means that MEM_Realloc should not be used, probably because 
 * memory reallocation is not supported by the platform.
 */
#if !defined(OS_MemRealloc) && !defined(NO_REALLOC)
#  define NO_REALLOC 1
#endif /* !OS_MemRealloc && !NO_REALLOC */

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_MEM_H_ */

/*
 * HISTORY:
 *
 * $Log: s_mem.h,v $
 * Revision 1.13  2010/06/26 09:26:41  slava
 * o use MEM_Equal instead of memcmp to resolve linking problem with some
 *   versions of Windows DDK
 *
 * Revision 1.12  2008/11/20 10:34:41  slava
 * o removed #ifdef _NT_KERNEL which isn't necessary anymore
 *
 * Revision 1.11  2006/10/29 22:49:06  slava
 * o MEM_NewArray, MEM_ReallocArray and MEM_Calloc should deal with
 *   possible block size overflow. Without checking for overflow, they
 *   may end up allocating less than requested amount of memory.
 *
 * Revision 1.10  2003/11/03 21:48:30  slava
 * o added MEM_Calloc function
 *
 * Revision 1.9  2003/11/01 14:20:54  slava
 * o do not include s_os.h from here, it can break compilation of kernel
 *   mode build, depending on the order of the include files. s_os.h should
 *   only be included from s_def.h
 *
 * Revision 1.8  2003/05/21 00:11:03  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.7  2003/05/09 18:25:44  slava
 * o fixed compilation warnings in the NT kernel mode build
 *
 * Revision 1.6  2002/12/30 15:52:45  slava
 * o renamed DebugTrace -> DEBUG_Trace, DebugAssert -> DEBUG_Assert,
 *   DebugAssertFormat -> DEBUG_AssertFormat
 *
 * Revision 1.5  2002/12/17 04:01:35  slava
 * o fixed a typo
 *
 * Revision 1.4  2002/08/12 05:13:27  slava
 * o use (void) rather than () to declare functions that have
 *   no arguments. The problem with () is that compilers interpret
 *   it as an *unspecified* argument list, but (void) clearly
 *   indicates that function takes no arguments. Improves compile
 *   time error checking
 *
 * Revision 1.3  2001/10/27 02:53:51  slava
 * o MEM_NewArray macro could produce unexpected results
 *
 * Revision 1.2  2001/10/22 00:58:01  slava
 * o fixed an error that broke release build
 *
 * Revision 1.1  2001/10/22 00:10:36  slava
 * o recrafted memory management.
 *
 * Local Variables:
 * mode:C
 * c-basic-offset:4
 * indent-tabs-mode: nil
 * End:
 */
