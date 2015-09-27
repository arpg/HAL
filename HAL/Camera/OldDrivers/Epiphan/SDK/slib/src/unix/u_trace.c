/*
 * $Id: u_trace.c,v 1.3 2009/03/25 22:30:50 slava Exp $
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

#include "s_util.h"

/**
 * Trace utilities for Unix environment
 */

#if DEBUG_TRACE

/**
 * Default assertion handler for Unix environment
 */
void DEBUG_AssertHandler(const char * msg, const char * file, long line)
{
    TRACE1("ASSERTION FAILED: %s\n",msg);
    TRACE1("FILE: %s\n",file);
    TRACE1("LINE: %lu\n",line);
}

#endif /* DEBUG_TRACE */

/**
 * This call has no effect on Unix platforms
 */
void PRINT_UseConsole(Bool enable)
{
    UNREF(enable);
}

/**
 * Dumps formatted string to debug console and stdout if such is available
 */
int PRINT_Stdout(Str format, va_list va)
{
    int n = Vfprintf(stdout, format, va);
    fflush(stdout);
    return n;
}

/**
 * Dumps formatted string to debug console and stderr if such is available
 */
int PRINT_Stderr(Str format, va_list va)
{
    int n = Vfprintf(stderr, format, va);
    fflush(stderr);
    return n;
}

/*
 * HISTORY:
 *
 * $Log: u_trace.c,v $
 * Revision 1.3  2009/03/25 22:30:50  slava
 * o made it easier to compile release version of slib with debug trace
 *
 * Revision 1.2  2008/11/05 12:15:08  slava
 * o made assert handler configurable, cleaned up the assertion handling
 *   code a bit, moved system specific code to system-specific directories
 *
 * Revision 1.1  2006/03/12 08:29:40  slava
 * o moved some of the print functions from s_trace.c to platform specific
 *   area, reducing number #ifdefs in s_trace.c
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
