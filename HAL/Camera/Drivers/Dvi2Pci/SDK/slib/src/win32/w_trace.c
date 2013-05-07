/*
 * $Id: w_trace.c,v 1.5 2010/07/04 13:44:25 slava Exp $
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

#include "s_util.h"

/**
 * Trace utilities for Win32 environment
 */

#ifndef _WIN32
#  error "_WIN32 must be defined to compile Win32 code"
#endif

#ifndef _WIN32_WCE
#   include <crtdbg.h> /* _CrtDbgReport */
#endif  /* !_WIN32_WCE */

STATIC Bool WIN32_stdout = MayBe;
STATIC Bool WIN32_stderr = MayBe;

/**
 * Enables or disables console trace.
 */
void PRINT_UseConsole(Bool enable)
{
    WIN32_stdout = (enable ? True : MayBe);
    WIN32_stderr = (enable ? True : MayBe);
}

/**
 * Determines if standard output stream is available for the process.
 */
STATIC Bool WIN32_HaveStdout()
{
    if (WIN32_stdout == MayBe) {        
        WIN32_stdout = 
#ifdef _WIN32_WCE
            False;
#else /* !_WIN32_WCE */
            (GetStdHandle(STD_OUTPUT_HANDLE) != INVALID_HANDLE_VALUE);
#endif /* !_WIN32_WCE */
    }
    return WIN32_stdout;
}

/**
 * Determines if standard error stream is available for the process.
 */
STATIC Bool WIN32_HaveStderr()
{
    if (WIN32_stderr == MayBe) {
        WIN32_stderr = 
#ifdef _WIN32_WCE
            False;
#else /* !_WIN32_WCE */
            (GetStdHandle(STD_ERROR_HANDLE) != INVALID_HANDLE_VALUE);
#endif /* !_WIN32_WCE */
    }
    return WIN32_stderr;
}

#if DEBUG_TRACE

/**
 * Default assertion handler for Windows platform
 */
void DEBUG_AssertHandler(const char * msg, const char * file, long line)
{

#if defined(_WIN32_WCE) || !DEBUG
    static long ass = 0;            /* counter of active ASSERT boxes */
#endif /* _WIN32_WCE */
    ULONG lastError = GetLastError();

    Char pname[MAX_PATH];
    pname[0] = 0;
    GetModuleFileName(NULL,pname,COUNT(pname)-1);
    pname[COUNT(pname)-1] = 0;

    TRACE1("ASSERTION FAILED: %hs\n",msg);
    TRACE1("PROGRAM: %s\n",pname);
    TRACE1("FILE: %hs\n",file);
    TRACE1("LINE: %lu\n",line);

    /*
     * _CrtDbgReport is defined in the debug version of MSVCRT. If we are
     * compiling release build with debug trace, we usually don't have it
     */
#if defined(_WIN32_WCE) || !DEBUG
    if (InterlockedIncrement(&ass) > 1) {
        /* nested ASSERTs? */
        DebugBreak();
    } else {
        int n;
        StrBuf128 buf;
        STRBUF_InitBufXXX(&buf);
        STRBUF_AppendFormat(&buf.sb,TEXT("ASSERTION FAILED: %hs\n"),msg);
        STRBUF_AppendFormat(&buf.sb,TEXT("PROGRAM: %s\n"),pname);
        STRBUF_AppendFormat(&buf.sb,TEXT("FILE: %hs\n"),file);
        STRBUF_AppendFormat(&buf.sb,TEXT("LINE: %lu\n"),line);
        __try {
            Str msg = buf.sb.s;
            UINT flags = MB_OKCANCEL | MB_ICONHAND | MB_TOPMOST;
            n = MessageBox(NULL,msg,TEXT("Debug?"),flags);
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            /* don't try to debug break, just log the event */
            TRACE1("MessageBox exception %08lX\n",GetExceptionCode());
            n = IDCANCEL;
        }
        STRBUF_Destroy(&buf.sb);
        if (n == IDOK) {
            DebugBreak();
        }
    }
    InterlockedDecrement(&ass);
#else
    if (_CrtDbgReport(_CRT_ASSERT, file, line, NULL, msg) == 1) {
        _CrtDbgBreak(); 
    } 
#endif /* _WIN32_WCE */
    SetLastError(lastError);
}

#endif /* DEBUG_TRACE */

#ifdef _UNICODE
#  undef PRINT_Stderr
#  undef PRINT_Stdout

#if DEBUG_TRACE

/**
 * Dumps formatted message to the debug console
 */
STATIC int WIN32_DebugPrintU(Str szFormat, va_list va)
{

#ifdef _WIN32_WCE
    static Bool writeDebugLog = True;   /* turns off "debug.log" file */
#endif /* !_WIN32_WCE */

    /* 
     * save last error as some of the routines that we invoke may change 
     * it OutputDebugString is one of them
     */
    ULONG lastError = GetLastError();
    int n = 0;

    /* must use exception handling on Windows CE */
#if defined(_USE_EXCEPTION_HANDLING) || defined(_WIN32_WCE)
    __try 
#endif /* _USE_EXCEPTION_HANDLING || _WIN32_WCE */
    {
        Char szBuffer[1020];    /* limitation of OutputDebugString */
        n = Vsnprintf(szBuffer,COUNT(szBuffer),szFormat,va);

        /* if output string is REALLY long, terminate it with ... */
        if (n >= COUNT(szBuffer)-1 || n < 0) {
            n = COUNT(szBuffer)-1;
            szBuffer[n--] = 0;
            if (szFormat[StrLen(szFormat)-1] == '\n') {
                szBuffer[n--] = '\n';
            }
            szBuffer[n--] = '.'; 
            szBuffer[n--] = '.'; 
            szBuffer[n--] = '.';
            szBuffer[n--] = ' ';
        } else {
            szBuffer[COUNT(szBuffer)-1] = 0;
        }

        /* send it to the debug console */
        OutputDebugString(szBuffer);

#ifdef _WIN32_WCE
        /* under Windows CE, dump it to a file too */
        if (writeDebugLog) {
            FILE* log = _tfopen(TEXT("debuglog.txt"), TEXT("at"));
            if (log) {
                _fputts(szBuffer, log);
                fclose(log);
            }
        }
#endif /* !_WIN32_WCE */
    }
#if defined(_USE_EXCEPTION_HANDLING) || defined(_WIN32_WCE)
    __except(EXCEPTION_EXECUTE_HANDLER) {
        ASSMSG1("EXCEPTION %08lX in WIN32_DebugPrint",GetExceptionCode());
    }
#endif /* _USE_EXCEPTION_HANDLING || _WIN32_WCE */

    /* restore last error */
    SetLastError(lastError);
    return 0;
}

#endif /* DEBUG_TRACE */

/**
 * Dumps formatted string to debug console and stdout if such is available
 */
int PRINT_StdoutU(Str format, va_list va)
{
    int n = 0;
#if DEBUG_TRACE
    n = WIN32_DebugPrintU(format, va);
#endif /* DEBUG_TRACE */
    if (!WIN32_HaveStdout()) return n;
    n = Vfprintf(stdout, format, va);
    fflush(stdout);
    return n;
}

/**
 * Dumps formatted string to debug console and stderr if such is available
 */
int PRINT_StderrU(Str format, va_list va)
{
    int n = 0;
#if DEBUG_TRACE
    n = WIN32_DebugPrintU(format, va);
#endif /* DEBUG_TRACE */
    if (!WIN32_HaveStderr()) return n;
    n = Vfprintf(stderr, format, va);
    fflush(stderr);
    return n;
}

#endif /* _UNICODE */

#if DEBUG_TRACE

/**
 * Dumps formatted message to the debug console
 */
STATIC int WIN32_DebugPrint(const char* szFormat, va_list va)
{
    /* 
     * save last error as some of the routines that we invoke may change 
     * it OutputDebugString is one of them
     */
    ULONG lastError = GetLastError();
    int n = 0;

#if defined(_USE_EXCEPTION_HANDLING)
    __try 
#endif /* _USE_EXCEPTION_HANDLING */
    {
        char szBuffer[1020];    /* limitation of OutputDebugString */
        n = vsnprintf(szBuffer,COUNT(szBuffer),szFormat,va);

        /* if output string is REALLY long, terminate it with ... */
        if (n >= COUNT(szBuffer)-1 || n < 0) {
            n = COUNT(szBuffer)-1;
            szBuffer[n--] = 0;
            if (szFormat[strlen(szFormat)-1] == '\n') {
                szBuffer[n--] = '\n';
            }
            szBuffer[n--] = '.'; 
            szBuffer[n--] = '.'; 
            szBuffer[n--] = '.';
            szBuffer[n--] = ' ';
        } else {
            szBuffer[COUNT(szBuffer)-1] = 0;
        }

        /* send it to the debug console */
        OutputDebugStringA(szBuffer);
    }
#if defined(_USE_EXCEPTION_HANDLING)
    __except(EXCEPTION_EXECUTE_HANDLER) {
        ASSMSG1("EXCEPTION %08lX in WIN32_DebugPrint",GetExceptionCode());
    }
#endif /* _USE_EXCEPTION_HANDLING */

    /* restore last error */
    SetLastError(lastError);
    return 0;
}

#endif /* DEBUG_TRACE */

/**
 * Dumps formatted string to debug console and stdout if such is available
 */
int PRINT_Stdout(const char* format, va_list va)
{
    int n = 0;
#if DEBUG_TRACE
    n = WIN32_DebugPrint(format, va);
#endif /* DEBUG_TRACE */
    if (!WIN32_HaveStdout()) return n;
    n = vfprintf(stdout, format, va);
    fflush(stdout);
    return n;
}

/**
 * Dumps formatted string to debug console and stderr if such is available
 */
int PRINT_Stderr(const char* format, va_list va)
{
    int n = 0;
#if DEBUG_TRACE
    n = WIN32_DebugPrint(format, va);
#endif /* DEBUG_TRACE */
    if (!WIN32_HaveStderr()) return n;
    n = vfprintf(stderr, format, va);
    fflush(stderr);
    return n;
}

/*
 * HISTORY:
 *
 * $Log: w_trace.c,v $
 * Revision 1.5  2010/07/04 13:44:25  slava
 * o UNICODE build issues
 *
 * Revision 1.4  2009/05/26 06:19:44  slava
 * o do not reference _CrtDbgReport in release build even if debug trace is on
 *
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
