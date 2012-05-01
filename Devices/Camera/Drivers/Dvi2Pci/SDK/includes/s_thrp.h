/*
 * $Id: s_thrp.h,v 1.4 2010/12/07 12:34:41 slava Exp $
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

#ifndef _SLAVA_THREAD_PRIVATE_H_
#define _SLAVA_THREAD_PRIVATE_H_

#include "s_thread.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* Global variables */
extern size_t slibThreadStackSize;

/* Internal functions */
extern void THREAD_PlatformInitialize P_((void));
extern void THREAD_PlatformShutdown P_((void));
extern Bool THREAD_IsInited P_((void));

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_THREAD_PRIVATE_H_ */

/*
 * HISTORY:
 *
 * $Log: s_thrp.h,v $
 * Revision 1.4  2010/12/07 12:34:41  slava
 * o added THREAD_SetStackSize() function. The default amount of memory
 *   committed for the thread stack on Linux is ridiculously large (8-10MB)
 *   which may be a problem for embedded platforms. To make matters worse,
 *   these memory blocks are kept for reuse and not freed when the thread
 *   terminates. THREAD_SetStackSize() allows the caller to tweak this
 *   value and save some virtual memory.
 *
 * Revision 1.3  2005/02/25 02:53:39  slava
 * o TLS code moved to platform specific area
 *
 * Revision 1.2  2005/02/21 13:43:00  slava
 * o more porting to EPOC gcc compiler
 *
 * Revision 1.1  2005/02/19 00:37:41  slava
 * o separated platform specific code from platform independent code
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
