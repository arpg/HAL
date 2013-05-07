/*
 * $Id: s_init.c,v 1.1 2008/03/02 09:44:07 slava Exp $
 *
 * Copyright (C) 2001-2008 by Slava Monich
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

#include "s_lib.h"

/**
 * Initializes all slib modules. Similar to SLIB_Init(), the only difference
 * is that under Win32 this function does NOT initialize Winsock. 
 */
void SLIB_InitModules()
{
    MEM_InitModule();
    THREAD_InitModule();
    WKQ_InitModule();
#ifndef __KERNEL__
    LOCK_InitModule();
#endif /* __KERNEL__ */
    HASH_InitModule();
    RANDOM_InitModule();
}

/**
 * Deinitializes all slib modules, prints memory usage statistics
 */
void SLIB_Shutdown()
{
    RANDOM_Shutdown();
    HASH_Shutdown();
#ifndef __KERNEL__
    LOCK_Shutdown();
#endif /* __KERNEL__ */
    WKQ_Shutdown();
    THREAD_Shutdown();
    MEM_Shutdown();
}

/*
 * HISTORY:
 *
 * $Log: s_init.c,v $
 * Revision 1.1  2008/03/02 09:44:07  slava
 * o moved SLIB_InitModules() and SLIB_Shutdown() to the separate file
 *   s_init.c to break curcular dependency between source files. Almost
 *   every xxx_InitModule function was dependant on all other xxx_InitModule
 *   functions via SLIB_Abort which happened to be in the same file (and
 *   therefore, the same object module) as SLIB_InitModules.
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
