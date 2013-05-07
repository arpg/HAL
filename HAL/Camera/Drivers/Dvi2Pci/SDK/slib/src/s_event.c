/*
 * $Id: s_event.c,v 1.22 2005/02/19 00:37:40 slava Exp $
 *
 * Copyright (C) 2000-2005 by Slava Monich
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

#include "s_event.h"
#include "s_mem.h"

/**
 * Platform independent part of Event API.
 */

 /**
 * Allocates a new event and initializes it
 */
Event * EVENT_Create() 
{
    Event * e = MEM_New(Event);
    if (e) {
        if (EVENT_Init(e)) {
            return e;
        }
        MEM_Free(e);
    }
    return NULL;
}

/**
 * Deletes and deallocates the event previously allocated with 
 * EVENT_Create()
 */
void EVENT_Delete(Event * e) 
{
    if (e) {
        EVENT_Destroy(e);
        MEM_Free(e);
    }
}

/*
 * HISTORY:
 *
 * $Log: s_event.c,v $
 * Revision 1.22  2005/02/19 00:37:40  slava
 * o separated platform specific code from platform independent code
 *
 * Revision 1.21  2004/11/27 17:42:17  slava
 * o added comment about the use of KeInitializeEvent at IRQL DISPATCH_LEVEL
 *
 * Revision 1.20  2004/05/28 12:03:48  slava
 * o use NotificationEvent rather than SynchronizationEvent in NT kernel mode
 *   environment
 *
 * Revision 1.19  2004/03/25 19:06:00  slava
 * o fixed a few pedantic compilation warnings
 *
 * Revision 1.18  2003/12/04 04:30:54  slava
 * o event support for Linux kernel
 *
 * Revision 1.17  2003/11/20 06:21:00  slava
 * o added EVENT_State function
 *
 * Revision 1.16  2003/11/02 17:43:04  slava
 * o added support for events in NT kernel mode environment
 *
 * Revision 1.15  2003/05/21 00:11:03  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.14  2003/01/24 02:38:18  slava
 * o cleaned up ASSERT messages (don't need newline)
 *
 * Revision 1.13  2002/08/14 00:59:52  slava
 * o fixed a bug in timeout calculation in Unix variant of EVENT_TimeWait.
 *   This bug seriously affected the code that uses subsecond timeouts
 *
 * Revision 1.12  2002/07/27 15:32:54  slava
 * o cleaned up error messages in Win32 build
 *
 * Revision 1.11  2002/05/29 08:27:40  slava
 * o removed EVENT_InitModule
 *
 * Revision 1.10  2002/02/11 06:40:13  slava
 * o check _USE_PTHREADS rather than _UNIX to find out whether we use
 *   Posix threads
 *
 * Revision 1.9  2001/11/25 21:25:13  slava
 * o InitModule functions now invoke dependent InitModule functions. That is,
 *   LOCK_InitModule invokes MUTEX_InitModule and EVENT_InitModule, etc.
 *
 * Revision 1.8  2001/11/24 19:39:19  slava
 * o cleaned up thread support for Unix platforms
 *
 * Revision 1.7  2001/11/23 22:45:35  slava
 * o better diagnostics if WaitForSingleObject() fails
 *
 * Revision 1.6  2001/07/26 05:56:09  slava
 * o added EVENT_Create() and EVENT_Delete() functions
 *
 * Revision 1.5  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.4  2001/05/18 22:36:57  slava
 * o THIS_FILE variable is no longer being used
 *
 * Revision 1.3  2001/01/31 02:00:41  slava
 * o portability fixes. Now compiles on Solaris
 *
 * Revision 1.2  2000/11/01 13:25:05  slava
 * o replaced TRUE/FALSE with True/False
 *
 * Revision 1.1  2000/08/19 04:48:58  slava
 * o initial checkin
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
