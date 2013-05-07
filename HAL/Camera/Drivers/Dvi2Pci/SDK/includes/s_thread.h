/*
 * $Id: s_thread.h,v 1.21 2010/12/07 12:34:40 slava Exp $
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

#ifndef _SLAVA_THREAD_H_
#define _SLAVA_THREAD_H_

#include "s_def.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* 
 * NOTE: we are first checking for _USE_PTHREADS and then for _WIN32 so that
 * it's possible to compile against pthreads library on Win32. Not that it 
 * makes much sense...
 */
#if defined(_NT_KERNEL)
    typedef struct _ThrID* ThrID;
#elif defined(_USE_PTHREADS)
#  include <pthread.h>
   typedef pthread_t ThrID;
#elif defined(_WIN32)
   typedef struct _ThrData* ThrID;
#elif defined(__SYMBIAN32__)
   typedef TUint ThrID;
#else
#  error "One of the platform specific macros must be defined"
#endif

typedef struct _ThrKey * ThrKey;
typedef void  (*ThrProc) P_((void * arg));
typedef void  (*ThrClean) P_((void * value));

extern void   THREAD_InitModule P_((void));
extern void   THREAD_Shutdown P_((void));
extern void   THREAD_SetStackSize P_((size_t bytes));

/* threads */
extern Bool   THREAD_Create P_((ThrID * tid, ThrProc proc, void * arg));
extern Bool   THREAD_Detach P_((ThrID tid));
extern Bool   THREAD_Join P_((ThrID tid));
extern Bool   THREAD_IsSelf P_((ThrID tid));
extern ThrID  THREAD_Self P_((void));
extern void   THREAD_Exit P_((void));
extern void   THREAD_Yield P_((void));
extern void   THREAD_Sleep P_((long ms));

/* thread local storage */
extern ThrKey THREAD_CreateKey P_((ThrClean proc));
extern void   THREAD_DeleteKey P_((ThrKey key));
extern Bool   THREAD_CanSetValue P_((ThrKey key));
extern Bool   THREAD_SetValue P_((ThrKey key, void * value));
extern void * THREAD_GetValue P_((ThrKey key));

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_THREAD_H_ */

/*
 * HISTORY:
 *
 * $Log: s_thread.h,v $
 * Revision 1.21  2010/12/07 12:34:40  slava
 * o added THREAD_SetStackSize() function. The default amount of memory
 *   committed for the thread stack on Linux is ridiculously large (8-10MB)
 *   which may be a problem for embedded platforms. To make matters worse,
 *   these memory blocks are kept for reuse and not freed when the thread
 *   terminates. THREAD_SetStackSize() allows the caller to tweak this
 *   value and save some virtual memory.
 *
 * Revision 1.20  2009/10/21 13:22:09  slava
 * o added THREAD_IsSelf function
 *
 * Revision 1.19  2008/09/01 14:57:41  slava
 * o defined ThrID on Win32 as a pointer to struct _ThrData for better type
 *   safety
 *
 * Revision 1.18  2005/02/21 13:42:59  slava
 * o more porting to EPOC gcc compiler
 *
 * Revision 1.17  2005/02/20 20:31:29  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.16  2004/11/27 06:58:54  slava
 * o added support for NT kernel threads
 *
 * Revision 1.15  2003/06/01 18:42:18  slava
 * o fixed a compilation error if this file is included from an Objective-C
 *   program ('id' has special meaning in Objective-C)
 *
 * Revision 1.14  2003/05/21 20:14:11  slava
 * o added THREAD_Exit function
 *
 * Revision 1.13  2002/12/17 04:01:35  slava
 * o fixed a typo
 *
 * Revision 1.12  2002/08/23 11:02:09  slava
 * o added THREAD_Sleep
 *
 * Revision 1.11  2002/08/12 05:13:27  slava
 * o use (void) rather than () to declare functions that have
 *   no arguments. The problem with () is that compilers interpret
 *   it as an *unspecified* argument list, but (void) clearly
 *   indicates that function takes no arguments. Improves compile
 *   time error checking
 *
 * Revision 1.10  2002/05/23 04:04:32  slava
 * o more code shared by Posix and Win32 portions of the code
 *
 * Revision 1.9  2002/02/11 05:53:22  slava
 * o support for thread local storage
 * o preprocessor definitions (_USE_PTHREADS and _WIN32) are now being
 *   analyzed in that order, i.e. it's possible to compile against pthreads
 *   library on Win32. Not very useful but why not... by default on Windose
 *   it's still compiled against Win32 API
 *
 * Revision 1.8  2001/06/12 08:57:07  slava
 * o enabled thread support for Windows CE
 *
 * Revision 1.7  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.6  2001/05/18 22:29:54  slava
 * o slib has been (partially) ported to Windows CE
 *
 * Revision 1.5  2001/03/28 08:54:02  slava
 * o added THREAD_Yield()
 *
 * Revision 1.4  2000/12/17 16:09:47  slava
 * o added extern "C" {} so that slib could be used from a C++ program
 *
 * Revision 1.3  2000/08/20 20:37:40  slava
 * o re-wrote Win32 support to use HANDLEs rather than Win32 thread IDs
 * o implemented Win32 variants of THREAD_Join() and THREAD_Detach()
 *
 * Revision 1.2  2000/08/20 19:08:03  slava
 * o added THREAD_Detach() and THREAD_Join() functions
 *
 * Revision 1.1  2000/08/19 04:48:58  slava
 * o initial checkin
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
