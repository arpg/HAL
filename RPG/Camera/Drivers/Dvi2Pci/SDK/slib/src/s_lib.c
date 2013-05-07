/*
 * $Id: s_lib.c,v 1.19 2010/09/28 20:17:25 slava Exp $
 *
 * Copyright (C) 2001-2010 by Slava Monich
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

#include "s_util.h"
#include "s_libp.h"

/**
 * This function is invoked when one of the modules has failed to initialize.
 * Never returns. Prints an error message and calls abort()
 */
void SLIB_Abort(Str module)
{
    Error(TEXT("Cannot initialize %s module, exiting...\n"),module);
    ASSMSG1("Cannot initialize %s module",module);

    /* should panic in kernel mode? */
#ifndef __KERNEL__
#  ifdef _WIN32_WCE
    exit(-1);
#  else /* !_WIN32_WCE */
    abort();
#  endif /* !_WIN32_WCE */
#endif /* __KERNEL__ */
}

/*
 * HISTORY:
 *
 * $Log: s_lib.c,v $
 * Revision 1.19  2010/09/28 20:17:25  slava
 * o don't make things more complex than they should be
 *
 * Revision 1.18  2010/09/28 19:26:21  slava
 * o 64-bit Unix build issues
 *
 * Revision 1.17  2008/03/02 09:44:07  slava
 * o moved SLIB_InitModules() and SLIB_Shutdown() to the separate file
 *   s_init.c to break curcular dependency between source files. Almost
 *   every xxx_InitModule function was dependant on all other xxx_InitModule
 *   functions via SLIB_Abort which happened to be in the same file (and
 *   therefore, the same object module) as SLIB_InitModules.
 *
 * Revision 1.16  2008/01/05 09:41:35  slava
 * o WKQ and THREAD modules are available in kernel environment, at least for
 *   Windows kernel and so far we don't support any other kernels
 *
 * Revision 1.15  2007/06/03 23:33:50  slava
 *  o include "s_libp.h" containing definition of SLIB_Abort
 *
 * Revision 1.14  2003/11/30 02:49:58  slava
 * o port to Linux kernel mode environment
 *
 * Revision 1.13  2003/11/02 17:48:40  slava
 * o linking issues in NT kernel mode environment
 *
 * Revision 1.12  2003/05/21 00:11:04  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.11  2003/03/18 16:52:44  slava
 * o added Unicode build configurations for Windoze
 *
 * Revision 1.10  2003/01/25 02:52:50  slava
 * o fixed compilation warning under Windows CE
 *
 * Revision 1.9  2003/01/20 19:02:46  slava
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
 * Revision 1.8  2002/08/29 03:16:45  slava
 * o some reformatting
 *
 * Revision 1.7  2002/05/29 08:29:49  slava
 * o removed EVENT_InitModule and MUTEX_InitModule functions
 *
 * Revision 1.6  2002/02/11 05:55:03  slava
 * o invoked THREAD_Shutdown() from SLIB_Shutdown()
 *
 * Revision 1.5  2001/12/23 21:25:26  slava
 * o moved SLIB_Init from s_lib.c to s_net.c in order to avoid unnecessary
 *   linkage with winsock dll on Windoze
 *
 * Revision 1.4  2001/11/27 15:12:35  slava
 * o synchronize access to global data in the HASH module.
 * o added HASH_InitModule() function
 *
 * Revision 1.3  2001/11/27 06:06:58  slava
 * o added "work queue" module
 *
 * Revision 1.2  2001/11/25 01:44:11  slava
 * o fixed SLIB_InitModules() to initialize modules in correct order
 * o SLIB_Shutdown() does not need to call MEM_DumpStat(), etc. because
 *   they are invoked from MEM_Shutdown()
 *
 * Revision 1.1  2001/11/25 01:29:02  slava
 * o added SLIB_Init(), SLIB_InitModules() and SLIB_Shutdown() functions
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
