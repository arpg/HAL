/*
 * $Id: s_trace.c,v 1.40 2010/11/19 08:50:51 slava Exp $
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

#include "s_buf.h"
#include "s_util.h"
#include "s_strbuf.h"

#define PRINTABLE(c) (IsPrint(c) ? (c) : '.')
#define BYTES_PER_LINE  16

STATIC int PRINT_mask = DEFAULT_PRINT_MASK;

/*==========================================================================*
 *              T R A C E
 *==========================================================================*/

/**
 * Sets trace mask
 */
void PRINT_SetMask(int mask)
{
    ASSERT(!(mask & (~PRINT_ALL)));
    PRINT_mask = mask;
}

/**
 * Returns current trace mask
 */
int PRINT_GetMask()
{
    return PRINT_mask;
}

/**
 * Normal non-debug output (PRINT_NORMAL)
 */
int PRINT_Output(Str format, ... )
{
    int n = 0;
    if (PRINT_mask & PRINT_NORMAL) {
        va_list va;
        va_start(va, format);
        n = PRINT_Stdout(format, va);
        va_end(va);
    }
    return n;
}

/**
 * Normal non-debug output (PRINT_NORMAL)
 */
int PRINT_OutputVa(Str format, va_list va)
{
    if (PRINT_mask & PRINT_NORMAL) {
        return PRINT_Stdout(format, va);
    } else {
        return 0;
    }
}

/**
 * Verbose trace (PRINT_VERBOSE)
 */
int PRINT_Verbose(Str format, ... )
{
    int n = 0;
    if (PRINT_mask & PRINT_VERBOSE) {
        va_list va;
        va_start(va, format);
        n = PRINT_Stdout(format, va);
        va_end(va);
    }
    return n;
}

/**
 * Verbose trace (PRINT_VERBOSE)
 */
int PRINT_VerboseVa(Str format, va_list va)
{
    if (PRINT_mask & PRINT_VERBOSE) {
        return PRINT_Stdout(format, va);
    } else {
        return 0;
    }
}

/**
 * Error trace (PRINT_ERROR)
 */
int PRINT_Error(Str format, ... )
{
    int n = 0;
    if (PRINT_mask & PRINT_ERROR) {
        va_list va;
        va_start(va, format);
        n = PRINT_Stderr(format, va);
        va_end(va);
    }
    return n;
}

/**
 * Error trace (PRINT_ERROR)
 */
int PRINT_ErrorVa(Str format, va_list va)
{
    if (PRINT_mask & PRINT_ERROR) {
        return PRINT_Stderr(format, va);
    } else {
        return 0;
    }
}

/**
 * Warning trace (PRINT_WARNING)
 */
int PRINT_Warning(Str format, ... )
{
    int n = 0;
    if (PRINT_mask & PRINT_WARNING) {
        va_list va;
        va_start(va, format);
        n = PRINT_Stderr(format, va);
        va_end(va);
    }
    return n;
}

/**
 * Warning trace (PRINT_WARNING)
 */
int PRINT_WarningVa(Str format, va_list va)
{
    if (PRINT_mask & PRINT_WARNING) {
        return PRINT_Stderr(format, va);
    } else {
        return 0;
    }
}

/**
 * Dump data into the stream 
 */
void PRINT_Dump2(PrintProc p, const void* buf, size_t count,
                 size_t off, size_t max)
{
    size_t i,j;
    const size_t maxCount = MIN(count,max);
    const unsigned char* data = (unsigned char*)buf;
    Char line[100];              /* actually, we fit into 80 characters */

    for (i=0; i<maxCount; i += BYTES_PER_LINE) {
        Char tmp[8];
        const Char * sep = TEXT("    ");
        Sprintf(line,
#ifdef __LONG_64__
                TEXT("   %04lx: "), (unsigned long)
#else  /* ! __LONG_64__ */
                TEXT("   %04x: "), (unsigned int)
#endif /* ! __LONG_64__ */
                (i+off));

        /* hex bytes */
        for (j=i; j < (i+BYTES_PER_LINE); j++) {
            if (j < maxCount) {
                Sprintf(tmp, TEXT("%02x "), (unsigned int)data[j]);
                StrCat(line, tmp);
            } else if (j == maxCount && j < count) {
                StrCat(line, TEXT(" ..."));
                sep = TEXT("   ");
            } else {
                StrCat(line, TEXT("   "));
            }
        }

        /* ASCII letters */
        StrCat(line, sep);
        for (j =i; j<(i+BYTES_PER_LINE); j++) {
            if (j<maxCount) {
                Sprintf(tmp, TEXT("%c"), PRINTABLE(data[j]));
                StrCat(line, tmp);
            } else {
                StrCat(line, TEXT(" "));
            }
        }

        p(TEXT("%s\n"), line);
    }

    /* if the data was truncated, print the final ... */
    if (i < count) {
        Sprintf(line,
#ifdef __LONG_64__
                TEXT("   %04lx: "), (unsigned long)
#else  /* ! __LONG_64__ */
                TEXT("   %04x: "), (unsigned int)
#endif /* ! __LONG_64__ */
                (i+off));
        StrCat(line, TEXT("..."));
        p(TEXT("%s\n"), line);
    }
}

void PRINT_Dump(PrintProc p, const void * buf, size_t count, size_t off)
{
    PRINT_Dump2(p, buf, count, off, (size_t)(-1));
}

/*==========================================================================*
 *              D E B U G
 *==========================================================================*/

#if DEBUG_TRACE

/**
 * Yet another assertion handler.
 */
void DEBUG_AssertFormat(const char * file, long line, Str format, ...)
{
    char msg[512];
#ifdef UNICODE
    Char msg2[COUNT(msg)];
#else /* !UNICODE */
#  define msg2 msg
#endif /* UNICODE */

#if defined(_WIN32) && !defined(_NT_KERNEL)
    ULONG lastError = GetLastError();
#  ifdef _USE_EXCEPTION_HANDLING
    __try {
#  endif /* _USE_EXCEPTION_HANDLING */
#endif /* _WIN32) && !_NT_KERNEL */
        int n;
        va_list va;
        va_start(va, format);

        /* format the message */
        n = Vsnprintf(msg2,COUNT(msg2),format,va);
#ifdef UNICODE
        msg[0] = 0;
        msg2[COUNT(msg2)-1] = 0;
        wcstombs(msg, msg2, COUNT(msg));
        msg[COUNT(msg)-1] = 0;
#endif /* UNICODE */

        /* if the output string is REALLY long, terminate it with ... */
        if (n >= COUNT(msg)-1 || n < 0) {
            n = COUNT(msg)-1;
            msg[n--] = 0;
            msg[n--] = '.'; 
            msg[n--] = '.'; 
            msg[n--] = '.';
            msg[n--] = ' ';
        } else {
            msg[COUNT(msg)-1] = 0;
        }

        va_end(va);
#if defined(_WIN32) && !defined(_NT_KERNEL)
#  ifdef _USE_EXCEPTION_HANDLING
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        /* NOTE: ASSMSG1 macro would recursively invoke this function */
        TRACE1("EXCEPTION %08lX in DEBUG_AssertFormat\n",GetExceptionCode());
        ASSMSG("EXCEPTION in DEBUG_AssertFormat");
    }
#  endif /* _USE_EXCEPTION_HANDLING */
    DEBUG_Assert(msg, file, line);
    SetLastError(lastError);
#else
    DEBUG_Assert(msg, file, line);
#endif /* _WIN32) && !_NT_KERNEL */
}

#  ifdef _UNICODE
#    undef DEBUG_Trace
#    undef DEBUG_TraceVa
#    undef PRINT_Stderr

/**
 * Debug trace
 */
int DEBUG_TraceU(Str format, ... )
{
    int n = 0;
    if (PRINT_mask & PRINT_DEBUG) {
        va_list va;
        va_start(va, format);
        n = PRINT_StderrU(format, va);
        va_end(va);
    }
    return 0;
}

/**
 * Yes another debug trace
 */
int DEBUG_TraceVaU(Str format, va_list va)
{
    int n = 0;
    if (PRINT_mask & PRINT_DEBUG) {
        n = PRINT_StderrU(format, va);
    }
    return 0;
}

#  endif /* _UNICODE */

/**
 * Debug trace
 */
int DEBUG_Trace(const char*  format, ... )
{
    int n = 0;
    if (PRINT_mask & PRINT_DEBUG) {
        va_list va;
        va_start(va, format);
        n = PRINT_Stderr(format, va);
        va_end(va);
    }
    return 0;
}

/**
 * Yes another debug trace
 */
int DEBUG_TraceVa(const char* format, va_list va)
{
    int n = 0;
    if (PRINT_mask & PRINT_DEBUG) {
        n = PRINT_Stderr(format, va);
    }
    return 0;
}

/*
 * Pointer to the actual assertion handler.
 */
DebugAssertProc slibDebugAssertHandler = DEBUG_AssertHandler;

/**
 * Assertion handler.
 */
void DEBUG_Assert(const char * msg, const char * file, long line)
{
    DebugAssertProc assertHandler = slibDebugAssertHandler;
    if (assertHandler) {
        assertHandler(msg, file, line);
    }
}

#endif /* DEBUG_TRACE */

/*
 * HISTORY:
 *
 * $Log: s_trace.c,v $
 * Revision 1.40  2010/11/19 08:50:51  slava
 * o removed yyerror from slib
 *
 * Revision 1.39  2009/05/23 10:14:00  slava
 * o fixed Unicode build
 *
 * Revision 1.38  2009/03/25 22:08:04  slava
 * o made it easier to compile release version of slib with debug trace
 *
 * Revision 1.37  2008/11/20 11:52:13  slava
 * o allow use of non-UNICODE console output functions in UNICODE build
 *
 * Revision 1.36  2008/11/20 08:26:25  slava
 * o switched NT kernel build to UNICODE
 *
 * Revision 1.35  2008/11/05 12:15:08  slava
 * o made assert handler configurable, cleaned up the assertion handling
 *   code a bit, moved system specific code to system-specific directories
 *
 * Revision 1.34  2006/11/03 16:35:39  slava
 * o added PRINT_Dump2 which can truncate the data and provide visual
 *   indication that the data have been truncated (which is slightly
 *   different from simply using MIN(count,maxCount) as number of bytes
 *   to dump).
 *
 * Revision 1.33  2006/03/12 08:29:40  slava
 * o moved some of the print functions from s_trace.c to platform specific
 *   area, reducing number #ifdefs in s_trace.c
 *
 * Revision 1.32  2005/02/20 20:31:30  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.31  2005/02/16 06:01:52  slava
 * o renamed TRACE_xxx functions and macros to PRINT_xxx. Macros are provided
 *   for backward compatibility
 *
 * Revision 1.30  2004/12/26 18:22:19  slava
 * o support for CodeWarrior x86 compiler
 *
 * Revision 1.29  2004/12/03 04:29:20  slava
 * o fixed a bug in TRACE_Dump, it was reading past the end of the buffer
 *
 * Revision 1.28  2004/11/27 16:57:12  slava
 * o introduced _USE_EXCEPTION_HANDLING preprocessor macro. In some cases the
 *   use of exception handling on Win32 creates conflicts (for example if slib
 *   is being linked with Symbian code running in the WINS emulator)
 *
 * Revision 1.27  2003/12/11 15:54:50  slava
 * LINUX_Printk attempts to make sure that the log level tag does not show up
 * in the middle of the debug string
 *
 * Revision 1.26  2003/12/04 04:48:55  slava
 * o replaced _LINUX_KERNEL_ with _LINUX_KERNEL for consistency with other
 *   platform defines such as _UNIX, _WIN32, _NT_KERNEL etc.
 *
 * Revision 1.25  2003/12/01 13:19:52  slava
 * o changed Linux kernel trace levels
 *
 * Revision 1.24  2003/11/30 02:49:58  slava
 * o port to Linux kernel mode environment
 *
 * Revision 1.23  2003/11/02 17:47:41  slava
 * o made TRACE_Error available in NT kernel mode environment
 *
 * Revision 1.22  2003/10/13 18:18:29  slava
 * o added TRACE_OutputVa, TRACE_VerboseVa, TRACE_WarningVa and TRACE_ErrorVa
 *
 * Revision 1.21  2003/07/11 15:04:13  slava
 * o renamed Error, Warning, Output, Verbose and Dump functions into
 *   TRACE_Error, TRACE_Warning, TRACE_Output, TRACE_Verbose and TRACE_Dump,
 *   respecively. The old names were too likely to get into conflict with
 *   non-slib symbols if slib is being integrated into a large project
 *   (which is exactly what has just happened). For backward compatibility,
 *   the old names (Error, Warning, Output, Verbose and Dump) are still
 *   defined as macros which can be disabled by defining TRACE_MACROS_DEFINED
 *   prior to including slib headers.
 *
 * Revision 1.20  2003/05/29 06:18:59  slava
 * o fixed compilation warning on Mac OS X
 *
 * Revision 1.19  2003/03/18 16:52:44  slava
 * o added Unicode build configurations for Windoze
 *
 * Revision 1.18  2002/12/30 15:52:46  slava
 * o renamed DebugTrace -> DEBUG_Trace, DebugAssert -> DEBUG_Assert,
 *   DebugAssertFormat -> DEBUG_AssertFormat
 *
 * Revision 1.17  2002/08/27 11:52:03  slava
 * o some reformatting
 *
 * Revision 1.16  2001/10/29 03:37:17  slava
 * o when popping up debug assert message boxes under windows CE, better use
 *   MB_TOPMOST flag. Otherwise, if on Pocket PC one of those message boxes
 *   gets under another window, there's no way to get it back to front
 *
 * Revision 1.15  2001/08/11 17:59:49  slava
 * o another change related to possible MessageBox failure on Window CE.
 *   If such thing happens, do not attempt to debug break because it may
 *   also kill the system.
 *
 * Revision 1.14  2001/08/11 17:52:47  slava
 * o MessageBox on CE throws exception if invoked very early, before the
 *   windows subsystem has been initialized. This may prevent CE device
 *   from booting up. Need to catch such exceptions.
 *
 * Revision 1.13  2001/08/06 07:08:35  slava
 * o under Windows, the assert handler now dumps the name of the executable
 *   file used to create the current process
 *
 * Revision 1.12  2001/08/04 01:11:51  slava
 * o renamed debug log file on WinCE from "debug.log" to "debuglog.txt"
 *
 * Revision 1.11  2001/07/02 13:39:02  slava
 * o Windows CE specific part of DebugAssert now pops up a message box.
 *
 * Revision 1.10  2001/06/22 09:15:33  slava
 * o made Dump() work on Windows CE
 *
 * Revision 1.9  2001/06/12 08:53:08  slava
 * o added TRACE_UseConsole() - most useful on Windows CE where console
 *   output is disabled by default.
 *
 * Revision 1.8  2001/06/08 08:27:21  slava
 * o use TEXT macro
 *
 * Revision 1.7  2001/06/08 08:17:50  slava
 * o added wrappers for character classification functions
 *
 * Revision 1.6  2001/06/08 07:54:43  slava
 * o fixed a compilation warning
 *
 * Revision 1.5  2001/06/08 04:21:13  slava
 * o exported TRACE_Stdout() and TRACE_Stderr() functions
 *
 * Revision 1.4  2001/06/05 15:05:50  slava
 * o don't do console output under Windows CE until I figure out how to
 *   detect console apps under CE
 *
 * Revision 1.3  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.2  2001/05/27 11:45:51  slava
 * o port to NT kernel mode environment
 *
 * Revision 1.1  2001/05/26 21:51:44  slava
 * o moved trace functions to s_trace.c
 *
 * Local Variables:
 * c-basic-offset:4
 * indent-tabs-mode: nil
 * End:
 */
