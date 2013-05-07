/*
 * $Id: s_thread.c,v 1.42 2010/12/07 12:34:40 slava Exp $
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

/* Global variables */
size_t slibThreadStackSize = 0;

/*
 * THREAD_InitModule() increments this count, THREAD_Shutdown decrements it
 */
STATIC int THREAD_initcount = 0;

/**
 * Used mostly for debug ASSERTs only
 */
Bool THREAD_IsInited()
{
    return BoolValue(THREAD_initcount > 0);
}

/**
 * Initialize the module
 */
void THREAD_InitModule() 
{
    if ((THREAD_initcount++) == 0) {
        MEM_InitModule();
        THREAD_PlatformInitialize();
    }
}

/**
 * Shutdown the module
 */
void THREAD_Shutdown()
{
    ASSERT(THREAD_initcount > 0);
    if (THREAD_initcount == 1) {
        THREAD_PlatformShutdown();
        MEM_Shutdown();
        THREAD_initcount = 0;
    } else {
        THREAD_initcount--;
    }
}

/**
 * Sets the stack size (in bytes) allocated for the created threads stack.
 * Zero value means to use platform-specific default.
 */
void THREAD_SetStackSize(size_t bytes)
{
    slibThreadStackSize = bytes;
}

/*
 * HISTORY:
 *
 * $Log: s_thread.c,v $
 * Revision 1.42  2010/12/07 12:34:40  slava
 * o added THREAD_SetStackSize() function. The default amount of memory
 *   committed for the thread stack on Linux is ridiculously large (8-10MB)
 *   which may be a problem for embedded platforms. To make matters worse,
 *   these memory blocks are kept for reuse and not freed when the thread
 *   terminates. THREAD_SetStackSize() allows the caller to tweak this
 *   value and save some virtual memory.
 *
 * Revision 1.41  2008/05/10 10:21:18  slava
 * o removed a bunch of useless typedefs.
 *
 * Revision 1.40  2005/02/25 02:53:39  slava
 * o TLS code moved to platform specific area
 *
 * Revision 1.39  2005/02/19 03:33:48  slava
 * o oops, _TLS_SUPPORT macro must be undefined for NT kernel mode build
 *
 * Revision 1.38  2005/02/19 03:30:58  slava
 * o use _TLS_SUPPORT macto
 *
 * Revision 1.37  2005/02/19 00:37:41  slava
 * o separated platform specific code from platform independent code
 *
 * Revision 1.36  2004/11/28 02:48:31  slava
 * o added a few ASSERTs for NT kernel mode environment
 *
 * Revision 1.35  2004/11/27 16:57:12  slava
 * o introduced _USE_EXCEPTION_HANDLING preprocessor macro. In some cases the
 *   use of exception handling on Win32 creates conflicts (for example if slib
 *   is being linked with Symbian code running in the WINS emulator)
 *
 * Revision 1.34  2004/11/27 06:58:54  slava
 * o added support for NT kernel threads
 *
 * Revision 1.33  2003/05/23 15:55:32  slava
 * o corrected use of preprocessor definitions in THREAD_Exit
 *
 * Revision 1.32  2003/05/21 20:14:11  slava
 * o added THREAD_Exit function
 *
 * Revision 1.31  2003/05/21 00:11:04  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.30  2003/01/20 19:02:46  slava
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
 * Revision 1.29  2002/08/23 11:02:09  slava
 * o added THREAD_Sleep
 *
 * Revision 1.28  2002/07/26 00:33:06  slava
 * o explicitely define contention scope for the created POSIX thread
 *   as PTHREAD_SCOPE_SYSTEM. It doesn't matter on Linux, but some
 *   systems (i.e. Solaris) default to PTHREAD_SCOPE_PROCESS which
 *   means that such threads don't take advantage of multiple CPUs.
 *
 * Revision 1.27  2002/07/01 02:35:17  slava
 * o don't clear THREAD_inited flag until after we have done with cleanup
 *
 * Revision 1.26  2002/05/23 04:15:02  slava
 * o fixed compilation errors under Windoze
 *
 * Revision 1.25  2002/05/23 04:04:32  slava
 * o more code shared by Posix and Win32 portions of the code
 *
 * Revision 1.24  2002/05/15 01:30:14  slava
 * o added useful ASSERT
 *
 * Revision 1.23  2002/02/26 06:51:14  slava
 * o attempt to clean up handle management for Win32 portion of the code
 *
 * Revision 1.22  2002/02/11 07:16:28  slava
 * o destructor for thread local variable does not need to nullify the
 *   thread local value, it's being done automatically
 *
 * Revision 1.21  2002/02/11 06:39:11  slava
 * o fixed gcc compilation warnings
 *
 * Revision 1.20  2002/02/11 05:57:00  slava
 * o support for thread local storage. new functions are: THREAD_CreateKey,
 *   THREAD_DeleteKey, THREAD_CanSetValue, THREAD_SetValue, THREAD_GetValue
 *   and THREAD_Shutdown
 * o preprocessor definitions (_USE_PTHREADS and _WIN32) are now being
 *   analyzed in that order, i.e. it's possible to compile against pthreads
 *   library on Win32. Not very useful but why not... by default on Windose
 *   it's still compiled against Win32 API
 *
 * Revision 1.19  2001/11/28 02:11:20  slava
 * o Win32 version of THREAD_Detach() should close the thread handle.
 *   Since THREAD_Join is not supposed to be invoked for detached thread,
 *   it's a very right thing to do. Otherwise, "detached" handles never
 *   get closed.
 *
 * Revision 1.18  2001/11/24 19:43:29  slava
 * o more cleanup
 *
 * Revision 1.17  2001/11/24 19:39:19  slava
 * o cleaned up thread support for Unix platforms
 *
 * Revision 1.16  2001/09/25 04:08:37  slava
 * o fixed debug error message
 *
 * Revision 1.15  2001/09/17 00:18:34  slava
 * o WIN32_IsThread() was returning False after thread has exited, even
 *   though thread handle is still considered valid (i.e. is waitable).
 *   Solution - use GetExitCodeThread() rather than GetThreadPriority()
 *   to test the thread handle, because it works even after thread has
 *   terminated
 *
 * Revision 1.14  2001/09/14 07:19:38  slava
 * o Win32 variant of THREAD_Join() should close the thread handle
 *
 * Revision 1.13  2001/06/12 08:57:07  slava
 * o enabled thread support for Windows CE
 *
 * Revision 1.12  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.11  2001/05/18 22:36:57  slava
 * o THIS_FILE variable is no longer being used
 *
 * Revision 1.10  2001/03/28 08:54:02  slava
 * o added THREAD_Yield()
 *
 * Revision 1.9  2001/01/31 02:00:41  slava
 * o portability fixes. Now compiles on Solaris
 *
 * Revision 1.8  2001/01/08 13:18:02  slava
 * o replaced tabs with spaces
 *
 * Revision 1.7  2000/11/01 13:25:05  slava
 * o replaced TRUE/FALSE with True/False
 *
 * Revision 1.6  2000/11/01 10:25:14  slava
 * o replaced New, NewArray and ReallocArray macros with MEM_New,
 *   MEM_NewArray and MEM_ReallocArray, respectively
 *
 * Revision 1.5  2000/09/03 00:38:53  slava
 * o fixed bug which was causing THREAD_Create() return bad thread handle
 *   on Windoze due to race condition between the caller and newly created
 *   thread. The returned thread handle was taken from start->handle which
 *   by the time ResumeThread() returns, may already be deallocated by the
 *   other thread.
 *
 * Revision 1.4  2000/08/20 20:40:57  slava
 * o fixed compile error under Linux
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
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
