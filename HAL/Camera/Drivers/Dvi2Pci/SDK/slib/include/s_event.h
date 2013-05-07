/*
 * $Id: s_event.h,v 1.16 2008/07/19 11:35:28 slava Exp $
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

#ifndef _SLAVA_EVENT_H_
#define _SLAVA_EVENT_H_

#include "s_def.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

typedef enum _WaitState {
    WAIT_STATE_OK = True,
    WAIT_STATE_ERROR = False,
    WAIT_STATE_TIMEOUT = MayBe
} WaitState;

typedef enum _EventState {
    EVENT_NOT_SIGNALED = 0,
    EVENT_SIGNALED
} EventState;

typedef struct _Event Event;

/* platform specific event definition */
#if defined(_NT_KERNEL)

struct _Event {
    KEVENT kevent;          /* kernel event object */
    KPRIORITY pinc;         /* priority increment */
};

#elif defined(_USE_PTHREADS)

typedef struct _PosixEvent {
    pthread_mutex_t mut;
    pthread_cond_t cond;
} PosixEvent;

struct _Event {
    PosixEvent posix;
    EventState state;
};

WaitState EVENT_NanoWait(Event * e, I64s nsec);

#elif defined(_WIN32)

struct _Event {
    HANDLE handle;
};

#elif defined(__SYMBIAN32__)

#include "s_mutex.h"

struct _Event {
    Klass RSemaphore * semaphore;
    Mutex mutex;
    EventState state;
    int waiters;
};

#else
#  error "One of the platform specific macros must be defined"
#endif

extern Event * EVENT_Create P_((void));
extern void EVENT_Delete P_((Event * e));
extern Bool EVENT_Init P_((Event * e));
extern void EVENT_Destroy P_((Event * e));
extern Bool EVENT_Set P_((Event * e));
extern Bool EVENT_Pulse P_((Event * e));
extern Bool EVENT_Reset P_((Event * e));
extern EventState EVENT_State P_((Event * e));
extern WaitState EVENT_Wait P_((Event * e));
extern WaitState EVENT_TimeWait P_((Event * e, long ms));

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_EVENT_H_ */

/*
 * HISTORY:
 *
 * $Log: s_event.h,v $
 * Revision 1.16  2008/07/19 11:35:28  slava
 * o added Posix-specific EVENT_NanoWait function
 * o removed Linux kernel specific defs (Linux kernel is no longer supported)
 *
 * Revision 1.15  2005/02/20 20:31:29  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.14  2004/11/26 20:44:36  slava
 * o made #error messages consistent
 *
 * Revision 1.13  2003/12/04 04:48:55  slava
 * o replaced _LINUX_KERNEL_ with _LINUX_KERNEL for consistency with other
 *   platform defines such as _UNIX, _WIN32, _NT_KERNEL etc.
 *
 * Revision 1.12  2003/12/04 04:30:54  slava
 * o event support for Linux kernel
 *
 * Revision 1.11  2003/11/20 06:21:00  slava
 * o added EVENT_State function
 *
 * Revision 1.10  2003/11/02 17:43:04  slava
 * o added support for events in NT kernel mode environment
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
 * Revision 1.7  2002/05/29 08:25:48  slava
 * o cleaned up the use of _WIN32 and _USE_PTHREADS constants
 * o removed EVENT_InitModule
 *
 * Revision 1.6  2001/09/14 07:35:52  slava
 * o removed SyncEvent definition which was not being used
 *
 * Revision 1.5  2001/08/06 07:13:16  slava
 * o changed WaitState definition to be compatible with the Bool type
 *
 * Revision 1.4  2001/07/26 05:56:09  slava
 * o added EVENT_Create() and EVENT_Delete() functions
 *
 * Revision 1.3  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.2  2000/12/17 16:09:47  slava
 * o added extern "C" {} so that slib could be used from a C++ program
 *
 * Revision 1.1  2000/08/19 04:48:58  slava
 * o initial checkin
 *
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
