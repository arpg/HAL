/*
 * $Id: s_mutex.h,v 1.19 2009/11/17 00:14:24 slava Exp $
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

#ifndef _SLAVA_MUTEX_H_
#define _SLAVA_MUTEX_H_

#include "s_lock.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

typedef struct _Mutex Mutex;

/* platform specific mutex definition */
#if defined(_NT_KERNEL)

/* NOTE: this mutex can be used at DISPATCH_LEVEL as well as PASSIVE_LEVEL */
struct _Mutex {
    Lock lock;          /* common part for all synchronization objects */ 
    KSPIN_LOCK spin;    /* kernel spinlock */
    KIRQL irql;         /* IRQL at which spinlock was acquired */
    BOOLEAN dpc;        /* True if spinlock was acquired at DISPATCH_LEVEL */
    BOOLEAN acquired;   /* True if mutex is acquired (obsolete field) */
    BOOLEAN spare;      /* unused, always zero. Ensures proper alignment */
    PKTHREAD thread;    /* thread that owns the lock */
};

#elif defined(_USE_PTHREADS)

struct _Mutex {
    Lock lock;              /* common part for all synchronization objects */
    pthread_mutex_t mutex;  /* Posix mutex */
    pthread_t thread;       /* Thread that owns the lock */
};

#elif defined(_WIN32)

struct _Mutex {
    Lock lock;              /* common part for all synchronization objects */
    HANDLE handle;          /* Win32 mutex handle */
    DWORD thread;           /* Thread that owns the lock */
};

#else
#  error "One of the platform specific macros must be defined"
#endif

extern Mutex * MUTEX_Create P_((void));
extern void MUTEX_Delete P_((Mutex * m));
extern Bool MUTEX_Init P_((Mutex * m));
extern void MUTEX_Destroy P_((Mutex * m));
extern Bool MUTEX_TryLock P_((Mutex * m));
extern Bool MUTEX_Lock P_((Mutex * m));
extern Bool MUTEX_IsLocked P_((const Mutex * m));
extern void MUTEX_Unlock P_((Mutex * m));

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_MUTEX_H_ */

/*
 * HISTORY:
 *
 * $Log: s_mutex.h,v $
 * Revision 1.19  2009/11/17 00:14:24  slava
 * o introducing generic synchronization API. This is a pretty destructive
 *   checkin, it's not 100% backward compatible. The Lock type is now a
 *   generic lock. What used to be called Lock is now RWLock. Sorry.
 *
 * Revision 1.18  2009/10/05 19:11:02  slava
 * o implemented MUTEX_IsLocked() for Unix
 *
 * Revision 1.17  2009/10/05 14:53:15  slava
 * o added MUTEX_IsLocked() function
 *
 * Revision 1.16  2008/12/10 17:26:03  slava
 * o ASSERT that Win32 mutex isn't being used recursively, for consistency
 *   with other platforms.
 *
 * Revision 1.15  2005/02/20 20:31:29  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.14  2004/11/26 20:44:36  slava
 * o made #error messages consistent
 *
 * Revision 1.13  2004/06/20 06:08:35  slava
 * o rearranged the fields of the _Mutex structure for NT kernel so that it
 *   occupies 4 bytes less by packing multiple BOOLEANs into a single DWORD
 *
 * Revision 1.12  2003/12/04 04:48:55  slava
 * o replaced _LINUX_KERNEL_ with _LINUX_KERNEL for consistency with other
 *   platform defines such as _UNIX, _WIN32, _NT_KERNEL etc.
 *
 * Revision 1.11  2003/11/30 02:49:57  slava
 * o port to Linux kernel mode environment
 *
 * Revision 1.10  2003/11/02 17:27:12  slava
 * o added another field to the Mutex structure in NT kernel mode environment
 *   to keep track of the thread that has locked the mutex, primarily for
 *   debugging purposes
 *
 * Revision 1.9  2002/12/17 04:01:35  slava
 * o fixed a typo
 *
 * Revision 1.8  2002/08/12 05:13:27  slava
 * o use (void) rather than () to declare functions that have
 *   no arguments. The problem with () is that compilers interpret
 *   it as an *unspecified* argument list, but (void) clearly
 *   indicates that function takes no arguments. Improves compile
 *   time error checking
 *
 * Revision 1.7  2002/05/29 08:26:24  slava
 * o cleaned up the use of _WIN32 and _USE_PTHREADS constants
 * o removed MUTEX_InitModule
 *
 * Revision 1.6  2001/09/14 07:38:16  slava
 * o changed Mutex definition for Win32, making it type safe
 *
 * Revision 1.5  2001/08/12 18:11:10  slava
 * o added MUTEX_Create() and MUTEX_Delete() functions
 *
 * Revision 1.4  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.3  2001/05/27 11:45:51  slava
 * o port to NT kernel mode environment
 *
 * Revision 1.2  2000/12/17 16:09:47  slava
 * o added extern "C" {} so that slib could be used from a C++ program
 *
 * Revision 1.1  2000/08/19 04:48:58  slava
 * o initial checkin
 *
 * Local Variables:
 * c-basic-offset:4
 * indent-tabs-mode: nil
 * End:
 */
